#include "vex.h"
#include "rover_wheel.h"
#include "vex_rover_motor.h"
#include "potential_planner.h"
#include "obstacle.h"

using namespace vex;

// ============================================================
// HARDWARE
// ============================================================
brain Brain;
inertial Brain_Inertial;
aivision ai(PORT3, aivision::ALL_TAGS);

// ============================================================
// CALIBRATION
// ============================================================
avs_calibration calib = {
    {218.34, 218.66},
    {165.62, 132.44},
    {240, 320}
};

const double tag_size = 0.045;

// ============================================================
// MAP
// ============================================================
extern vex_apriltag_pose tag_map[];

// ============================================================
// STATE
// ============================================================
static Pose3D rover_pose;
static potential_planner planner;

// obstacles
static obstacle obs1(65.0);
static obstacle obs2(65.0);
static obstacle* obstacles[] = { &obs1, &obs2 };

// ============================================================
// SIMPLE VISION POSE (UNCHANGED STRUCTURALLY)
// ============================================================
struct VisionTag {
    int id;
    double x;
    double y;
};

bool get_tag(VisionTag& out) {
    ai.takeSnapshot(aivision::ALL_TAGS);
    if (ai.objectCount == 0) return false;

    auto obj = ai.objects[0];
    out.id = obj.id;
    out.x  = obj.centerX;
    out.y  = obj.centerY;
    return true;
}

bool update_rover_pose(Pose3D& out) {
    VisionTag vt;
    if (!get_tag(vt)) return false;
    if (vt.id < 1 || vt.id > 14) return false;

    auto& map_tag = tag_map[vt.id - 1];

    Pose3D T_1m;
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            T_1m.R[r][c] = map_tag.R[r * 3 + c];

    for (int i = 0; i < 3; i++)
        T_1m.t[i] = map_tag.t[i];

    double angle = (vt.x - 160.0) * 0.002;

    Pose3D T_Bm;
    T_Bm.setIdentity();
    T_Bm.t[0] = cos(angle);
    T_Bm.t[1] = sin(angle);
    T_Bm.t[2] = 0;

    out = T_1m * T_Bm.inverse();
    return true;
}

// ============================================================
// MAIN
// ============================================================
int main() {

    // ----------------------------
    // DRIVE
    // ----------------------------
    static vex_rover_motor motor_L(PORT6);
    static vex_rover_motor motor_R(PORT10);

    static std_wheel wheel_L(0.125, 0.041,  M_PI/2, 0, 1, &motor_L);
    static std_wheel wheel_R(0.125, 0.041, -M_PI/2, 0, 1, &motor_R);

    static rover_wheel* wheels[] = { &wheel_L, &wheel_R };
    static kinematic_drivetrain dt(wheels);

    // ----------------------------
    // GOAL
    // ----------------------------
    planner.set_goal(tag_map[11].t[0], tag_map[11].t[1]);

    // ----------------------------
    // OBSTACLES
    // ----------------------------
    obs1.tag = tag_map[12];
    obs2.tag = tag_map[13];

    // ----------------------------
    // IMU
    // ----------------------------
    Brain_Inertial.calibrate();
    while (Brain_Inertial.isCalibrating())
        wait(50, msec);

    rover_pose.setIdentity();

    // ============================================================
    // MAIN LOOP
    // ============================================================
    while (true) {

        // ----------------------------
        // update pose (if available)
        // ----------------------------
        update_rover_pose(rover_pose);

        // ----------------------------
        // PLANNER OUTPUT (TRUST THIS)
        // ----------------------------
        double v_out[3] = {0, 0, 0};
        planner.calc_velocity(rover_pose, obstacles, 2, v_out);

        // ----------------------------
        // IMU heading ONLY for frame alignment
        // ----------------------------
        double heading =
            (2.0 * M_PI) -
            Brain_Inertial.heading(deg) * M_PI / 180.0;

        dt.set_heading(heading);

        // ----------------------------
        // SAFETY FILTER (VERY IMPORTANT)
        // ----------------------------
        if (isnan(v_out[0]) || isnan(v_out[2])) {
            v_out[0] = 0;
            v_out[2] = 0;
        }

        // ----------------------------
        // NO DOUBLE HEADING CONTROL
        // (THIS WAS YOUR JITTER BUG)
        // ----------------------------

        // clamp forward safety
        if (v_out[0] < 0) v_out[0] = 0;

        // mild damping for rotation stability
        v_out[2] *= 0.7;

        // ----------------------------
        // DRIVE
        // ----------------------------
        dt.calc_wheel_speeds_abs(v_out);

        wait(20, msec);
    }
}