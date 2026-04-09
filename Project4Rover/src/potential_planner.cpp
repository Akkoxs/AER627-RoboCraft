#include "potential_planner.h"
#include <math.h>

// ============================================================
// ATTRACTIVE FORCE (WORLD FRAME)
// ============================================================
void potential_planner::calc_attractive(const Pose3D& rover, double f[2]) {
    f[0] = k_att * (x_goal[0] - rover.t[0]);
    f[1] = k_att * (x_goal[1] - rover.t[1]);
}

// ============================================================
// REPULSIVE FORCE (WORLD FRAME)
// ============================================================
void potential_planner::calc_repulsive(const Pose3D& rover, obstacle& obs, double f[2]) {

    double rho[3];
    double d = obs.closest_point(rover, rho);

    f[0] = 0.0;
    f[1] = 0.0;

    if (d <= 0.001 || d >= r0) return;

    // prevent singularity explosion
    if (d < 0.05) d = 0.05;

    double inv_d = 1.0 / d;

    double rep_mag =
        k_rep *
        (inv_d - 1.0 / r0) *
        (inv_d * inv_d);

    // push AWAY from obstacle
    f[0] = rep_mag * (rho[0] / d);
    f[1] = rep_mag * (rho[1] / d);
}

// ============================================================
// CLAMP
// ============================================================
double potential_planner::clamp(double val, double limit) {
    if (val >  limit) return  limit;
    if (val < -limit) return -limit;
    return val;
}

// ============================================================
// MAIN PLANNER
// ============================================================
void potential_planner::calc_velocity(
    const Pose3D& rover,
    obstacle** obs,
    int n_obs,
    double v_out[3]
) {

    // --------------------------------------------------------
    // 1. Sum forces in WORLD FRAME
    // --------------------------------------------------------
    double f_world[2] = {0.0, 0.0};
    double f_tmp[2];

    // attractive
    calc_attractive(rover, f_tmp);
    f_world[0] += f_tmp[0];
    f_world[1] += f_tmp[1];

    // repulsive
    for (int i = 0; i < n_obs; i++) {
        calc_repulsive(rover, *obs[i], f_tmp);
        f_world[0] += f_tmp[0];
        f_world[1] += f_tmp[1];
    }

    // --------------------------------------------------------
    // 2. Convert WORLD force -> BODY force
    //    (proper rotation using matrix, NOT atan2 hack)
    // --------------------------------------------------------
    double f_body_x =
        rover.R[0][0] * f_world[0] +
        rover.R[1][0] * f_world[1];

    double f_body_y =
        rover.R[0][1] * f_world[0] +
        rover.R[1][1] * f_world[1];

    // --------------------------------------------------------
    // 3. Velocity output mapping
    // --------------------------------------------------------
    v_out[0] = clamp(f_body_x, max_v);   // forward
    v_out[1] = 0.0;                      // no strafing (diff drive)
    v_out[2] = clamp(f_body_y, max_w);   // rotation from lateral force
}