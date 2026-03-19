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

//CSV row format
//J1_deg, J2_deg, pen      
//(pen: 1 = down, 0 = up)

//general 
float deg2rads = M_PI/180.0;

//defintions 
brain Brain;
motor motorJ1(PORT1, ratio18_1, false);   // shoulder
motor motorJ2(PORT2, ratio18_1, false);   // elbow
motor motorPen(PORT3, ratio18_1, false);

//tuning params
float PEN_DOWN_ANGLE = 45.0;   // degrees to lower pen
float PEN_UP_ANGLE =  0.0;   // degrees to raise pen
int SETTLE_MS =  80;    // ms to wait after joint move
int PEN_SETTLE_MS = 150;    // ms to wait after pen move
const char* SD_FILE = "/trajectory.csv";

//physical params
float L1 = 3*63.5; //mm
float L2 = 2*63.5; //mm

//state machine
enum State {STARTUP, DRAWING, END};
State currentState = STARTUP; //inital state 
const char* stateNames[] = {"STARTUP", "DRAWING", "END"};

//pen tip position in mm 
//using FK: tip = base + L1*(cos J1) + L2*(cos(J1+J2))
float currentXPos = 0.0;
float currentYPos = 0.0;
int currentRow = 0;
int totalRows = 0;
bool penIsDown = false;

//reads in lines from the .csv file 
bool ParseLine(const char* line, float& j1, float& j2, int& pen) {
    return sscanf(line, "%f,%f,%d", &j1, &j2, &pen) == 3;
}

//updates the current pen tip position using FK
void UpdateXY(float J1_deg, float J2_deg) {
    currentXPos = L1 * cos(J1_deg*deg2rads) + L2 * cos(J1_deg*deg2rads + J2_deg*deg2rads);
    currentYPos = L1 * sin(J1_deg*deg2rads) + L2 * sin(J1_deg*deg2rads + J2_deg*deg2rads);
}

//controls whether pen is up or down
void SetPen(bool down) {
    float target;
    if (down == true) {
        target = PEN_DOWN_ANGLE;
    } 
    else {
        target = PEN_UP_ANGLE;
    }   

    motorPen.spinToPosition(target, degrees, true); //blocking flag = true
    penIsDown = down; //update pen state
    wait(PEN_SETTLE_MS, msec);
}

void MoveJoints(float J1_deg, float J2_deg) {
    motorJ1.spinToPosition(J1_deg, degrees, false); //blocking flag = false
    motorJ2.spinToPosition(J2_deg, degrees, false);   

    while (motorJ1.isSpinning() || motorJ2.isSpinning()) { //basically the equivalent to the blocking flag 
        wait(10, msec);
    }
    wait(SETTLE_MS, msec);
}

//make sure its in optimal position to start, no zeroing 
void StartUp() {
    motorJ1.resetPosition();
    motorJ2.resetPosition();
    motorPen.resetPosition();
    SetPen(false);
}

void Drawing() {
    FILE* file_ptr = fopen(SD_FILE, "r");

    //find address of file on the SD card, if cant find, let us know
    if (file_ptr == nullptr) {
        Brain.Screen.clearScreen();
        Brain.Screen.printAt(5, 55, "SD error: cant find file");
        wait(3000, msec);
        return;
    }

    //count how many rows are in the .csv file so we can track progress
    totalRows = 0;
    char tempLine[64]; //this string gets filled up and dumped, its just for counting
    while (fgets(tempLine, sizeof(tempLine), file_ptr) != nullptr) {
        totalRows++;
    }

    //after we done counting, reset back at the top
    rewind(file_ptr);
    currentRow = 0;
 
    while (fgets(tempLine, sizeof(tempLine), file_ptr) != nullptr) {
        float J1, J2;
        int pen;
 
        bool lineIsValid = ParseLine(tempLine, J1, J2, pen);
        if (!lineIsValid) {
            continue; //skip blank or malformed lines
        }

        bool wantDown = (pen == 1);
 
        //to ensure we always lift the pen before moving 
        if (!wantDown && penIsDown) {
            SetPen(false);
        }
 
        MoveJoints(J1, J2);
        UpdateXY(J1, J2);
 
        //ensuring to always lower the pen after moving 
        if (wantDown && !penIsDown) {
            SetPen(true);
        }
 
        currentRow++;
    }
    fclose(file_ptr);
 
    //lift pen and go back to start point
    SetPen(false);
    MoveJoints(0.0, 0.0);
    UpdateXY(0.0, 0.0);
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
            Drawing();
            currentState = END;
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

void UpdateScreen(){
    Brain.Screen.clearScreen();
    Brain.Screen.setFont(mono12);
    Brain.Screen.printAt(55, 45, "CURRENT STATE");
    Brain.Screen.printAt(40, 55, stateNames[currentState]);
    Brain.Screen.printAt(55,65, "X: %2.0f mm", currentXPos);
    Brain.Screen.printAt(55,75, "Y: %2.0f mm", currentYPos);
    Brain.Screen.printAt(55, 85, "Row: %d / %d", currentRow, totalRows);
}

int screenTask(){
    while(true){
        UpdateScreen();
        wait(100, msec); 
    }
    return 0;
}

//entry
int main() {
    vex::thread screenThread(screenTask);

    while(true) {
        
        EvaluateState(currentState);

        wait(20, msec);
    }
}



