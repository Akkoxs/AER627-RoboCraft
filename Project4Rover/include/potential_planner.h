#pragma once

#include "Pose3D.h"
#include "obstacle.h"

// Potential Field Planner
// Computes a velocity command (v_out) from the current rover pose,
// goal position, and a list of obstacles.
//
// All positions are in mm, in the world frame (relative to origin tag).
// v_out[0] = forward velocity (m/s)
// v_out[2] = angular velocity (rad/s)

class potential_planner {
public:
    // Tuning parameters - adjust these for your field
    double k_att  = 0.0005;   // Attractive gain (pulls toward goal)
    double k_rep  = 5000.0;   // Repulsive gain (pushes away from obstacles)
    double k_rot  = 2000.0;   // Rotational gain (helps escape local minima)
    double r0     = 400.0;    // Repulsion boundary distance (mm) - obstacles only repel within this range
    double max_v  = 0.5;      // Max forward velocity (m/s)
    double max_w  = 2.0;      // Max angular velocity (rad/s)

    // Goal position in world frame (mm)
    double x_goal[2] = {0.0, 0.0};

    // Constructor
    potential_planner() {}

    // Set goal position in world frame (mm)
    void set_goal(double x, double y) {
        x_goal[0] = x;
        x_goal[1] = y;
    }

    // Main function: compute velocity command from current rover pose and obstacles
    // rover    : current rover pose in world frame (T_1B)
    // obs      : array of obstacle pointers
    // n_obs    : number of obstacles
    // v_out    : output velocity [vx, vy, omega] - vy is always 0 for diff drive
    void calc_velocity(const Pose3D& rover, obstacle** obs, int n_obs, double v_out[3]);

private:
    // Compute attractive force in body frame
    void calc_attractive(const Pose3D& rover, double f[2]);

    // Compute repulsive + rotational force in body frame for one obstacle
    void calc_repulsive(const Pose3D& rover, obstacle& obs, double f[2]);

    // Clamp a value between -limit and +limit
    static double clamp(double val, double limit);
};
