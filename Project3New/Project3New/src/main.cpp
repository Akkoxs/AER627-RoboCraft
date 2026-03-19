/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       kai-s                                                     */
/*    Created:      3/2/2026, 6:49:56 PM                                      */
/*    Description:  EXP project                                               */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;

// general 
float deg2rads = 3.14159265358979323/180.0;
float J1_gearRatio = 24.0f/12.0f;
float J2_gearRatio = 24.0f/8.0f;

// definitions 
brain Brain;
motor motorJ1(PORT1, false); // shoulder
motor motorJ2(PORT2, false); // elbow
motor motorPen(PORT6, false); // pen

// tuning params
float PEN_DOWN_ANGLE = 0.0;
float PEN_UP_ANGLE = 20;
int SETTLE_MS = 80;
int PEN_SETTLE_MS = 150;

// physical params
float L1 = 175; // mm
float L2 = 145; // mm

// state machine
enum State {STARTUP, DRAWING, END};
State currentState = STARTUP;
const char* stateNames[] = {"STARTUP", "DRAWING", "END"};

// global params 
float currentXPos = 0.0;
float currentYPos = 0.0;
int currentRow = 0;
int totalRows = 0;
bool penDown = false;
float currentJ1 = 0.0;
float currentJ2 = 0.0;

// ── HARDCODED TRAJECTORY ──────────────────────────────────────────────────────
struct TrajectoryPoint {
    float J1;
    float J2;
    int   pen;
};

static const TrajectoryPoint trajectory[] = {
    {-8.848f,   -156.8111f, 0},
    {-8.848f,   -156.8111f, 1},
    {-8.848f,   -156.8111f, 1},
    {-8.848f,   -156.8111f, 1},
    {-8.848f,   -156.8111f, 1},
    {-8.848f,   -156.8111f, 1},
    {-8.848f,   -156.8111f, 1},
    {-8.848f,   -156.8111f, 1},
    {-8.848f,   -156.8111f, 1},
    {-8.848f,   -156.8111f, 1},
    {-8.848f,   -156.8111f, 1},
    {-8.848f,   -156.8111f, 1},
    {-8.848f,   -156.8111f, 1},
    {-8.848f,   -156.8111f, 1},
    {-8.848f,   -156.8111f, 1},
    {-8.848f,   -156.8111f, 1},
    {-6.0528f,  -154.8502f, 1},
    {-3.5074f,  -152.8025f, 1},
    {-1.161f,   -150.6827f, 1},
    { 1.0262f,  -148.5016f, 1},
    { 3.0853f,  -146.2672f, 1},
    { 5.0407f,  -143.9854f, 1},
    { 6.9119f,  -141.6602f, 1},
    { 8.7148f,  -139.2946f, 1},
    {10.4619f,  -136.8905f, 1},
    {12.164f,   -134.449f,  1},
    {13.8297f,  -131.9704f, 1},
    {15.4665f,  -129.4547f, 1},
    {17.0809f,  -126.9013f, 1},
    {18.6784f,  -124.309f,  1},
    {20.2639f,  -121.6765f, 1},
    {18.2951f,  -120.7379f, 1},
    {16.4304f,  -119.6927f, 1},
    {14.6738f,  -118.5439f, 1},
    {13.0283f,  -117.2946f, 1},
    {11.4956f,  -115.9475f, 1},
    {10.0769f,  -114.5054f, 1},
    { 8.7726f,  -112.9705f, 1},
    { 7.5822f,  -111.3451f, 1},
    { 6.5052f,  -109.631f,  1},
    { 5.5401f,  -107.8295f, 1},
    { 4.6857f,  -105.9416f, 1},
    { 3.9402f,  -103.968f,  1},
    { 3.3019f,  -101.9085f, 1},
    { 2.7691f,   -99.7627f, 1},
    { 2.3403f,   -97.5294f, 1},
    { 0.1652f,   -99.7627f, 1},
    {-2.0241f,  -101.9085f, 1},
    {-4.231f,   -103.968f,  1},
    {-6.4587f,  -105.9416f, 1},
    {-8.7099f,  -107.8295f, 1},
    {-10.9872f, -109.631f,  1},
    {-13.2927f, -111.3451f, 1},
    {-15.6284f, -112.9705f, 1},
    {-17.9956f, -114.5054f, 1},
    {-20.3952f, -115.9475f, 1},
    {-22.8276f, -117.2946f, 1},
    {-25.2924f, -118.5439f, 1},
    {-27.7885f, -119.6927f, 1},
    {-30.314f,  -120.7379f, 1},
    {-32.8662f, -121.6765f, 1},
    {-32.8603f, -124.309f,  1},
    {-32.7286f, -126.9013f, 1},
    {-32.4584f, -129.4547f, 1},
    {-32.0345f, -131.9704f, 1},
    {-31.4388f, -134.449f,  1},
    {-30.6501f, -136.8905f, 1},
    {-29.6432f, -139.2946f, 1},
    {-28.3883f, -141.6602f, 1},
    {-26.8501f, -143.9854f, 1},
    {-24.9872f, -146.2672f, 1},
    {-22.7511f, -148.5016f, 1},
    {-20.0856f, -150.6827f, 1},
    {-16.9271f, -152.8025f, 1},
    {-13.2055f, -154.8502f, 1},
    {-8.848f,   -156.8111f, 1},
    {-8.848f,   -156.8111f, 0},
};

static const int TRAJECTORY_SIZE = sizeof(trajectory) / sizeof(trajectory[0]);
// ─────────────────────────────────────────────────────────────────────────────

void UpdateXY(float J1_deg, float J2_deg) {
    currentXPos = L1 * cos(J1_deg*deg2rads) + L2 * cos(J1_deg*deg2rads + J2_deg*deg2rads);
    currentYPos = L1 * sin(J1_deg*deg2rads) + L2 * sin(J1_deg*deg2rads + J2_deg*deg2rads);
}

void SetPen(bool down) {
    float target = down ? PEN_DOWN_ANGLE : PEN_UP_ANGLE;
    motorPen.spinToPosition(target, degrees, true);
    penDown = down;
    wait(PEN_SETTLE_MS, msec);
}

void MoveJoints(float J1_deg, float J2_deg) {
    motorJ1.spinToPosition(J1_deg * J1_gearRatio, degrees, false);
    motorJ2.spinToPosition(J2_deg * J2_gearRatio, degrees, false);
    while (motorJ1.isSpinning() || motorJ2.isSpinning()) {
        wait(10, msec);
    }
    wait(SETTLE_MS, msec);
}

void StartUp() {
    motorJ1.setVelocity(50, percent);
    motorJ2.setVelocity(50, percent);
    motorPen.setVelocity(25, percent);

    motorJ1.setStopping(hold);
    motorJ2.setStopping(hold);
    motorPen.setStopping(hold);

    motorJ1.resetPosition();
    motorJ2.resetPosition();
    motorPen.resetPosition();

    penDown = true;
    totalRows = TRAJECTORY_SIZE;
}

void DrawingStep() {
    static int index = 0;

    if (index < TRAJECTORY_SIZE) {
        const TrajectoryPoint& pt = trajectory[index];

        currentJ1 = pt.J1;
        currentJ2 = pt.J2;
        bool wantDown = (pt.pen == 1);

        if (!wantDown && penDown)  SetPen(false);
        MoveJoints(pt.J1, pt.J2);
        UpdateXY(pt.J1, pt.J2);
        if (wantDown && !penDown)  SetPen(true);

        currentRow = index + 1;
        index++;
    } else {
        // finished — return home
        SetPen(false);
        MoveJoints(0.0, 0.0);
        UpdateXY(0.0, 0.0);

        currentState = END;
        index = 0; // reset for potential replay
    }
}

void End() {
    wait(500, msec);
}

void EvaluateState(State state) {
    switch (state) {
        case STARTUP:
            StartUp();
            currentState = DRAWING;
            break;
        case DRAWING:
            DrawingStep();
            break;
        case END:
            End();
            break;
        default:
            Brain.Screen.clearScreen();
            Brain.Screen.printAt(5, 55, "UNKNOWN STATE");
            break;
    }
}

void UpdateScreen() {
    Brain.Screen.clearScreen();
    Brain.Screen.setFont(mono12);
    Brain.Screen.printAt(40, 25, "CURRENT STATE");
    Brain.Screen.printAt(40, 35, stateNames[currentState]);
    Brain.Screen.printAt(40, 45, "X: %2.0f mm  Y: %2.0f mm", currentXPos, currentYPos);
    Brain.Screen.printAt(40, 55, "J1: %2.1f  J2: %2.1f", currentJ1, currentJ2);
    Brain.Screen.printAt(40, 65, "Pen: %d", penDown);
    Brain.Screen.printAt(40, 75, "Row: %d / %d", currentRow, totalRows);
    Brain.Screen.printAt(40, 85, "Hardcoded trajectory");
}

int screenTask() {
    while (true) {
        UpdateScreen();
        wait(100, msec);
    }
    return 0;
}

int main() {
    vex::thread screenThread(screenTask);
    while (true) {
        EvaluateState(currentState);
        wait(20, msec);
    }
}