#include "vex.h"
#include "vex_apriltag.h"
#include "vex_apriltag_types.h"
#include "rover_wheel.h"
#include "vex_rover_motor.h"
#include "potential_planner.h"
#include "obstacle.h"

using namespace vex;

brain         Brain;
inertial      Brain_Inertial;
aivision      ai(PORT3, aivision::ALL_TAGS);

avs_calibration calib = {
    {218.34, 218.66},
    {165.62, 132.44},
    {240,    320   }
};

const double tag_size = 0.045;

extern vex_apriltag_pose tag_map[];

// ============================================================
// SCENE CONFIGURATION
// Edit these when you reshoot with different tag assignments.
// All indices are (tag_id - 1).
// ============================================================
static const int    GOAL_TAG_INDEX  = 11;      // tag id 12
static const int    OBS1_TAG_INDEX  = 12;      // tag id 13
static const int    OBS2_TAG_INDEX  = 13;      // tag id 14
static const double OBS1_RADIUS_M   = 0.065;  // metres
static const double OBS2_RADIUS_M   = 0.065;
static const double MAX_SPEED_RADS  = 0.15 * 200.0 * 2.0 * M_PI / 60.0; // 15% of max (~3.14 rad/s)

// Tags used purely for localization (boundary markers + origin).
// Obstacle and goal tags are excluded so their poses are never
// used as the rover's position estimate.
static const int LOCALIZATION_TAG_IDS[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
static const int NUM_LOCALIZATION_TAGS  = 11;

// ============================================================
// STATE
// ============================================================
static Pose3D rover_pose;
static potential_planner planner;

static obstacle obs1(OBS1_RADIUS_M);
static obstacle obs2(OBS2_RADIUS_M);
static obstacle* obstacles[] = { &obs1, &obs2 };

// ============================================================
// DIAGNOSTICS
// ============================================================
static int    diag_visible_tag_id = -1;
static double diag_rover_x        = 0.0;
static double diag_rover_y        = 0.0;
static double diag_heading_deg    = 0.0;
static double diag_v_fwd          = 0.0;
static double diag_v_rot          = 0.0;
static bool   diag_pose_valid     = false;
static int    diag_tag_count      = 0;

// ============================================================
// SCREEN THREAD
// ============================================================
int screenTask()
{
    while (true) {
        Brain.Screen.clearScreen();
        Brain.Screen.setFont(mono12);

        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("=== ROVER DIAGNOSTICS ===");

        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Pose: %s",
            diag_pose_valid ? "VALID    " : "DEAD RECK");

        Brain.Screen.setCursor(3, 1);
        Brain.Screen.print("X:%6.3fm  Y:%6.3fm",
            diag_rover_x, diag_rover_y);

        Brain.Screen.setCursor(4, 1);
        Brain.Screen.print("Heading: %6.1f deg", diag_heading_deg);

        Brain.Screen.setCursor(5, 1);
        Brain.Screen.print("Fwd:%5.3f  Rot:%5.3f",
            diag_v_fwd, diag_v_rot);

        Brain.Screen.setCursor(6, 1);
        Brain.Screen.print("Tags visible: %d", diag_tag_count);

        Brain.Screen.setCursor(7, 1);
        if (diag_visible_tag_id >= 0)
            Brain.Screen.print("Active tag: ID %d   ", diag_visible_tag_id);
        else
            Brain.Screen.print("Active tag: none    ");

        Brain.Screen.setCursor(8, 1);
        Brain.Screen.print("Goal:  tag %d", GOAL_TAG_INDEX + 1);

        Brain.Screen.setCursor(9, 1);
        Brain.Screen.print("Obs1:  tag %d  r=%.3fm",
            OBS1_TAG_INDEX + 1, OBS1_RADIUS_M);

        Brain.Screen.setCursor(10, 1);
        Brain.Screen.print("Obs2:  tag %d  r=%.3fm",
            OBS2_TAG_INDEX + 1, OBS2_RADIUS_M);

        Brain.Screen.setCursor(11, 1);
        Brain.Screen.print("R00:%.2f R01:%.2f", rover_pose.R[0][0], rover_pose.R[0][1]);

        wait(100, msec);
    }
    return 0;
}

// ============================================================
// HELPER: raw vex_apriltag_pose (R[3][3], t in mm) -> Pose3D
// ============================================================
static Pose3D raw_tag_to_pose(const vex_apriltag_pose& tag)
{
    Pose3D p;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++)
            p.R[i][j] = tag.R[i][j];
        p.t[i] = tag.t[i] * 0.001; // mm -> m
    }
    return p;
}

// ============================================================
// LOCALIZATION FILTER
// ============================================================
static bool is_localization_tag(int id)
{
    for (int i = 0; i < NUM_LOCALIZATION_TAGS; i++) {
        if (LOCALIZATION_TAG_IDS[i] == id) return true;
    }
    return false;
}

// ============================================================
// POSE UPDATE
// ============================================================
static bool update_rover_pose(Pose3D& out)
{
    ai.takeSnapshot(aivision::ALL_TAGS);
    diag_tag_count = ai.objectCount;

    if (ai.objectCount == 0) {
        diag_visible_tag_id = -1;
        return false;
    }

    for (int i = 0; i < ai.objectCount; i++) {
        auto& obj = ai.objects[i];
        int id = obj.id;

        // Only localize from boundary markers and origin.
        // Skip goal and obstacle tags.
        if (!is_localization_tag(id)) continue;

        vex_apriltag_pose cam_pose;
        if (calculate_tag_pose(&obj, &calib, tag_size, &cam_pose) != 0)
            continue;

        Pose3D T_world_tag = raw_tag_to_pose(tag_map[id - 1]);
        Pose3D T_cam_tag   = raw_tag_to_pose(cam_pose);
        out = T_world_tag * T_cam_tag.inverse();

        diag_visible_tag_id = id;
        diag_rover_x        = out.t[0];
        diag_rover_y        = out.t[1];
        diag_pose_valid     = true;
        return true;
    }

    diag_visible_tag_id = -1;
    diag_pose_valid     = false;
    return false;
}

// ============================================================
// MAIN
// ============================================================
int main()
{
    // ----------------------------
    // DRIVE
    // ----------------------------
    static vex_rover_motor motor_L(PORT6);
    static vex_rover_motor motor_R(PORT10);

    // Set base velocity via the public interface (rad/s)
    motor_L.set_velocity(MAX_SPEED_RADS);
    motor_R.set_velocity(MAX_SPEED_RADS);

    static std_wheel wheel_L(0.125, 0.041,  M_PI / 2, 0, 1, &motor_L);
    static std_wheel wheel_R(0.125, 0.041, -M_PI / 2, 0, 1, &motor_R);

    static rover_wheel* wheels[] = { &wheel_L, &wheel_R, nullptr };
    static kinematic_drivetrain dt(wheels);

    // ----------------------------
    // SCENE SETUP
    // ----------------------------
    obs1.tag.pose = raw_tag_to_pose(tag_map[OBS1_TAG_INDEX]);
    obs2.tag.pose = raw_tag_to_pose(tag_map[OBS2_TAG_INDEX]);

    Pose3D goal_pose = raw_tag_to_pose(tag_map[GOAL_TAG_INDEX]);
    planner.set_goal(goal_pose.t[0], goal_pose.t[1]);

    // ----------------------------
    // IMU
    // ----------------------------
    Brain_Inertial.calibrate();
    while (Brain_Inertial.isCalibrating()) {
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Calibrating IMU...");
        wait(50, msec);
    }

    rover_pose.setIdentity();

    // Start screen thread after IMU is ready
    vex::thread screenThread(screenTask);

    // ============================================================
    // MAIN LOOP
    // ============================================================
    while (true) {
        update_rover_pose(rover_pose);

        double v_out[3] = {0.0, 0.0, 0.0};
        planner.calc_velocity(rover_pose, obstacles, 2, v_out);

        double heading = Brain_Inertial.heading(deg) * (M_PI / 180.0);


        dt.set_heading(heading);
        diag_heading_deg = Brain_Inertial.heading(deg);

        // Safety: reject NaN
        if (isnan(v_out[0]) || isnan(v_out[2])) {
            v_out[0] = 0.0;
            v_out[2] = 0.0;
        }

        // No reversing
        if (v_out[0] < 0.0) v_out[0] = 0.0;

        // Dampen rotation
        v_out[2] *= 0.7;

        // Clamp to speed limit
        if (v_out[0] >  MAX_SPEED_RADS) v_out[0] =  MAX_SPEED_RADS;
        if (v_out[2] >  MAX_SPEED_RADS) v_out[2] =  MAX_SPEED_RADS;
        if (v_out[2] < -MAX_SPEED_RADS) v_out[2] = -MAX_SPEED_RADS;

        diag_v_fwd = v_out[0];
        diag_v_rot = v_out[2];

        dt.calc_wheel_speeds_abs(v_out);

        wait(20, msec);
    }
}

// #include "vex.h"
// #include "rover_wheel.h"
// #include "vex_rover_motor.h"
// #include "potential_planner.h"
// #include "obstacle.h"

// using namespace vex;

// // ============================================================
// // HARDWARE
// // ============================================================
// brain Brain;
// inertial Brain_Inertial;
// aivision ai(PORT3, aivision::ALL_TAGS);

// // ============================================================
// // CALIBRATION
// // ============================================================
// avs_calibration calib = {
//     {218.34, 218.66},
//     {165.62, 132.44},
//     {240, 320}
// };

// const double tag_size = 0.045;

// // ============================================================
// // MAP
// // ============================================================
// extern vex_apriltag_pose tag_map[];

// // ============================================================
// // STATE
// // ============================================================
// static Pose3D rover_pose;
// static potential_planner planner;

// // obstacles
// static obstacle obs1(65.0);
// static obstacle obs2(65.0);
// static obstacle* obstacles[] = { &obs1, &obs2 };

// // ============================================================
// // SIMPLE VISION POSE (UNCHANGED STRUCTURALLY)
// // ============================================================
// struct VisionTag {
//     int id;
//     double x;
//     double y;
// };

// bool get_tag(VisionTag& out) {
//     ai.takeSnapshot(aivision::ALL_TAGS);
//     if (ai.objectCount == 0) return false;

//     auto obj = ai.objects[0];
//     out.id = obj.id;
//     out.x  = obj.centerX;
//     out.y  = obj.centerY;
//     return true;
// }

// bool update_rover_pose(Pose3D& out) {
//     VisionTag vt;
//     if (!get_tag(vt)) return false;
//     if (vt.id < 1 || vt.id > 14) return false;

//     auto& map_tag = tag_map[vt.id - 1];

//     Pose3D T_1m;
//     for (int r = 0; r < 3; r++)
//         for (int c = 0; c < 3; c++)
//             T_1m.R[r][c] = map_tag.R[r * 3 + c];

//     for (int i = 0; i < 3; i++)
//         T_1m.t[i] = map_tag.t[i];

//     double angle = (vt.x - 160.0) * 0.002;

//     Pose3D T_Bm;
//     T_Bm.setIdentity();
//     T_Bm.t[0] = cos(angle);
//     T_Bm.t[1] = sin(angle);
//     T_Bm.t[2] = 0;

//     out = T_1m * T_Bm.inverse();
//     return true;
// }

// // ============================================================
// // MAIN
// // ============================================================
// int main() {

//     // ----------------------------
//     // DRIVE
//     // ----------------------------
//     static vex_rover_motor motor_L(PORT6);
//     static vex_rover_motor motor_R(PORT10);

//     static std_wheel wheel_L(0.125, 0.041,  M_PI/2, 0, 1, &motor_L);
//     static std_wheel wheel_R(0.125, 0.041, -M_PI/2, 0, 1, &motor_R);

//     static rover_wheel* wheels[] = { &wheel_L, &wheel_R };
//     static kinematic_drivetrain dt(wheels);

//     // ----------------------------
//     // GOAL
//     // ----------------------------
//     planner.set_goal(tag_map[11].t[0], tag_map[11].t[1]);

//     // ----------------------------
//     // OBSTACLES
//     // ----------------------------
//     obs1.tag = tag_map[12];
//     obs2.tag = tag_map[13];

//     // ----------------------------
//     // IMU
//     // ----------------------------
//     Brain_Inertial.calibrate();
//     while (Brain_Inertial.isCalibrating())
//         wait(50, msec);

//     rover_pose.setIdentity();

//     // ============================================================
//     // MAIN LOOP
//     // ============================================================
//     while (true) {

//         // ----------------------------
//         // update pose (if available)
//         // ----------------------------
//         update_rover_pose(rover_pose);

//         // ----------------------------
//         // PLANNER OUTPUT (TRUST THIS)
//         // ----------------------------
//         double v_out[3] = {0, 0, 0};
//         planner.calc_velocity(rover_pose, obstacles, 2, v_out);

//         // ----------------------------
//         // IMU heading ONLY for frame alignment
//         // ----------------------------
//         double heading =
//             (2.0 * M_PI) -
//             Brain_Inertial.heading(deg) * M_PI / 180.0;

//         dt.set_heading(heading);

//         // ----------------------------
//         // SAFETY FILTER (VERY IMPORTANT)
//         // ----------------------------
//         if (isnan(v_out[0]) || isnan(v_out[2])) {
//             v_out[0] = 0;
//             v_out[2] = 0;
//         }

//         // ----------------------------
//         // NO DOUBLE HEADING CONTROL
//         // (THIS WAS YOUR JITTER BUG)
//         // ----------------------------

//         // clamp forward safety
//         if (v_out[0] < 0) v_out[0] = 0;

//         // mild damping for rotation stability
//         v_out[2] *= 0.7;

//         // ----------------------------
//         // DRIVE
//         // ----------------------------
//         dt.calc_wheel_speeds_abs(v_out);

//         wait(20, msec);
//     }
// }