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
float deg2rads = 3.14159265358979323/180.0;
//float J1_gearRatio = 12.0f/24.0f;
//float J2_gearRatio = 8.0f/24.0f;
float J1_gearRatio = 24.0f/12.0f;
float J2_gearRatio = 24.0f/8.0f;

//defintions 
brain Brain;
motor motorJ1(PORT1, false); // shoulder
motor motorJ2(PORT2, false); // elbow
motor motorPen(PORT6, false); //pen

//tuning params
float PEN_DOWN_ANGLE = 0.0;   // degrees to lower pen
float PEN_UP_ANGLE = 20;   // degrees to raise pen
int SETTLE_MS = 80;    // ms to wait after joint move
int PEN_SETTLE_MS = 150;    // ms to wait after pen move
const char* SD_FILE = "/trajectory.csv";
//   /square/trajectory.csv
//   /sigma/trajectory.csv
//   /backwards_R/trajectory.csv
//   /star/trajectory.csv
//   /house/trajectory.csv

//physical params
float L1 = 175; //mm
float L2 = 145; //mm

//state machine
enum State {STARTUP, DRAWING, END};
State currentState = STARTUP; //inital state 
const char* stateNames[] = {"STARTUP", "DRAWING", "END"};

//global params 
float currentXPos = 0.0;
float currentYPos = 0.0;
int currentRow = 0;
int totalRows = 0;
bool penDown = false;
float currentJ1 = 0.0;
float currentJ2 = 0.0;
char currentLine[128] = "";

//reads in lines from the .csv file 
bool ParseLine(const char* line, float& J1, float& J2, int& pen) {
    return sscanf(line, "%f,%f,%d", &J1, &J2, &pen) == 3;}

//updates the current pen tip position using FK
//using FK: tip = base + L1*(cos J1) + L2*(cos(J1+J2))
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
    penDown = down; //update pen state
    wait(PEN_SETTLE_MS, msec);
}

void MoveJoints(float J1_deg, float J2_deg) {
    motorJ1.spinToPosition(J1_deg * J1_gearRatio, degrees, false); // non-blocking
    motorJ2.spinToPosition(J2_deg * J2_gearRatio, degrees, false); // non-blocking

    // wait until both motors reach target to be released from while()
    while (motorJ1.isSpinning() || motorJ2.isSpinning()) {
        wait(10, msec);
    }

    wait(SETTLE_MS, msec);
}

//make sure its in optimal position to start, no zeroing 
void StartUp() {

    //set velocities
    motorJ1.setVelocity(50, percent);
    motorJ2.setVelocity(50, percent);
    motorPen.setVelocity(25, percent);

    //set braking
    motorJ1.setStopping(hold);
    motorJ2.setStopping(hold);
    motorPen.setStopping(hold);

    //reset encoders & set pen
    motorJ1.resetPosition();
    motorJ2.resetPosition();
    motorPen.resetPosition();
    penDown = true;
}

void DrawingStep() {
    //static vars persist through iterations 
    static int bufferPos = 0;
    static char buffer[12000];
    static bool initialized = false;
    static int fileSize = 0;

    if (!initialized) {

        fileSize = Brain.SDcard.size(SD_FILE);  // fetch size from SD card
        Brain.SDcard.loadfile(SD_FILE, (uint8_t*)buffer, fileSize);
        buffer[fileSize] = '\0';

        //count lines for progress
        totalRows = 0;
        for (int i = 0; i < fileSize; i++){
            if (buffer[i] == '\n') {
                totalRows++;
            }
        }

        //edge case where if the file doesnt end in a \n which means the last line was not counted
        if (fileSize > 0 && buffer[fileSize - 1] != '\n'){
            totalRows++;
        } 
        
        bufferPos = 0;
        currentRow = 0;
        initialized = true;
    }

    //if we are still in the file 
    if (bufferPos < fileSize) {
        float J1, J2;
        int pen;
        int charsRead = 0;

        if (sscanf(buffer + bufferPos, "%f,%f,%d%n", &J1, &J2, &pen, &charsRead) == 3) {
            currentJ1 = J1;
            currentJ2 = J2;

            // copy raw line into currentLine for display
            int i = 0;
            while (buffer[bufferPos + i] != '\n' && buffer[bufferPos + i] != '\0' && i < 127) {
                currentLine[i] = buffer[bufferPos + i];
                i++;
            }
            currentLine[i] = '\0';

            bool wantDown = (pen == 1);
            if (!wantDown && penDown) SetPen(false);
            MoveJoints(J1, J2);
            UpdateXY(J1, J2);
            if (wantDown && !penDown) SetPen(true);
        }

        // advance to next line regardless of parse success
        while (bufferPos < fileSize && buffer[bufferPos] != '\n') bufferPos++;
        bufferPos++; // skip '\n'
        currentRow++;
    }
    // if (bufferPos < fileSize) {
    //     //extract next line manually into a temp buffer
    //     char lineBuffer[128]; ///temp array for reading in lines
    //     int lineLen = 0;

    //     //copies a single line into the buffer 
    //     //while we are still in the file and have not reached the end of the current line 
    //     while (bufferPos < fileSize && buffer[bufferPos] != '\n') {
    //         char c = buffer[bufferPos++]; //store char 
    //         if (c != '\r' && lineLen < 127) { //if we havent reached end of line
    //             lineBuffer[lineLen++] = c; //read in the char
    //         }
    //     }
    //     bufferPos++; // to skip the '\n'
    //     lineBuffer[lineLen] = '\0';

    //     float J1, J2;
    //     int pen;
    //     if (ParseLine(lineBuffer, J1, J2, pen)) { //proceeds only if a line is successfully parsed 
    //         bool wantDown = (pen == 1); //wantDown is if pen == 1 directly from the .csv 

    //         currentJ1 = J1;
    //         currentJ2 = J2;

    //         int i = 0;
    //         while (lineBuffer[i] != '\0' && i < 127) {
    //             currentLine[i] = lineBuffer[i];
    //             i++;
    //         }
    //         currentLine[i] = '\0';

    //         if (!wantDown && penDown){
    //             SetPen(false);
    //         } 

    //         MoveJoints(J1, J2);
    //         UpdateXY(J1, J2);

    //         if (wantDown && !penDown) {
    //             SetPen(true);
    //         }
    //     currentRow++;
    //     }
    // }

    else {
    //finished drawing
    SetPen(false);
    MoveJoints(0.0, 0.0);
    UpdateXY(0.0, 0.0);

    currentState = END;
    initialized = false;
    bufferPos = 0;
    fileSize = 0;
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
            //currentState = END;
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
    Brain.Screen.printAt(40, 25, "CURRENT STATE");
    Brain.Screen.printAt(40, 35, stateNames[currentState]);
    Brain.Screen.printAt(40, 45, "X: %2.0f mm  Y: %2.0f mm", currentXPos, currentYPos);
    Brain.Screen.printAt(40, 55, "J1: %2.1f  J2: %2.1f", currentJ1, currentJ2);
    Brain.Screen.printAt(40, 65, "Pen: %d", penDown);
    Brain.Screen.printAt(40, 75, "Row: %d / %d", currentRow, totalRows);
    Brain.Screen.printAt(40, 85, currentLine);
    Brain.Screen.printAt(40, 95, "SD: ");
    Brain.Screen.printAt(65, 95, Brain.SDcard.isInserted() ? "OK" : "NOT FOUND");
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


// void Drawing() {
//     if (!Brain.SDcard.isInserted()) {
//         Brain.Screen.clearScreen();
//         Brain.Screen.printAt(5, 55, "SD NOT INSERTED");
//         wait(2000, msec);
//         return;
//     }

//     if (!Brain.SDcard.exists(SD_FILE)) {
//         Brain.Screen.clearScreen();
//         Brain.Screen.printAt(5, 55, "FILE NOT FOUND");
//         wait(2000, msec);
//         return;
//     }

//     int fileSize = Brain.SDcard.size(SD_FILE);
//     static char buffer[12000]; // make sure this is bigger than your file
//     if (fileSize >= sizeof(buffer)) {
//         Brain.Screen.clearScreen();
//         Brain.Screen.printAt(5, 55, "FILE TOO BIG");
//         wait(2000, msec);
//         return;
//     }

//     Brain.SDcard.loadfile(SD_FILE, (uint8_t*)buffer, fileSize);
//     buffer[fileSize] = '\0'; // null terminate

//     // FIRST PASS: count total lines
//     totalRows = 0;
//     for (int i = 0; i < fileSize; i++) {
//         if (buffer[i] == '\n') totalRows++;
//     }
//     if (fileSize > 0 && buffer[fileSize - 1] != '\n') totalRows++; // last line no \n

//     currentRow = 0;
//     char* line = strtok(buffer, "\n");

//     while (line != NULL) {
//         // remove trailing \r if present
//         int len = strlen(line);
//         if (len > 0 && line[len - 1] == '\r') line[len - 1] = '\0';

//         float J1, J2;
//         int pen;

//         if (sscanf(line, "%f,%f,%d", &J1, &J2, &pen) == 3) {
//             bool wantDown = (pen == 1);

//             if (!wantDown && penDown) SetPen(false);
//             MoveJoints(J1, J2);
//             UpdateXY(J1, J2);
//             if (wantDown && !penDown) SetPen(true);
//         }

//         currentRow++;
//         line = strtok(NULL, "\n");
//     }

//     // return to home
//     SetPen(false);
//     MoveJoints(0.0, 0.0);
//     UpdateXY(0.0, 0.0);
// }

