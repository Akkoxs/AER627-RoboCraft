/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       kai-s                                                     */
/*    Created:      1/29/2026, 5:19:46 PM                                     */
/*    Description:  EXP project                                               */
/*                                                                            */
/*----------------------------------------------------------------------------*/

//PORTS
//PORT10 = VERTICAL PRISMATIC
//PORT7 = CLAW 
//PORT8 = OPTICAL SENSOR
//PORT6 = HORIZONTAL ROTARY 
//PORT9 = HOPPER 

#include "vex.h"

using namespace vex;

//general
brain Brain;
controller Controller = controller();
float pi = 3.14159265358979323846264; 
float Deg2rads = pi/180;
float motorStallCurrent = 80;//placeholder (percent)

//state machine
enum State {STARTUP, 
            DISPENSE_BLOCK, 
            TRAVERSE_TO_PICKUP_ZONE,
            DESCEND_TO_BLOCK,
            GRAB, 
            ASCEND, 
            TRAVERSE_TO_RED_ZONE,
            TRAVERSE_TO_BLUE_ZONE,
            DESCEND_TO_STACK,
            RELEASE, 
            END};

//array of pointers to an array of constant chars
const char* stateNames[] = {
    "STARTUP",
    "DISPENSE_BLOCK",
    "TRAVERSE_TO_PICKUP_ZONE",
    "DESCEND_TO_BLOCK",
    "GRAB",
    "ASCEND",
    "TRAVERSE_TO_RED_ZONE",
    "TRAVERSE_TO_BLUE_ZONE",
    "DESCEND_TO_STACK",
    "RELEASE",
    "END"};

State currentState = STARTUP;
bool newBlock;

//optical sensor
optical Optical = optical(PORT8);
color detectedColor;
int detectionRange = 100; 
//Optical.objectLost(func);

//gantry
motor horizontalMotor = motor(PORT6, false);
bool horizontalHomingFlag = false;
float currentHorizontalPos;
float wheelRadius = 0.04445; //m
float pickupPos_horizontal = 280; //mm

//claw
motor verticalMotor = motor(PORT10, false);
bool verticalHomingFlag = false;
float currentVerticalPos; 
float gearRadius = 0.01375; //m
float pickupPos_vertical = 64; //mm
float verticalPosMax = 67; //mm
float targetHeight;

motor clawMotor = motor(PORT7, false);
bool clawHomingFlag = false;
float currentClawPos;
float clawDefaultPos = 30;
float clawClampPos = 63; //deg

//hopper
motor hopMotor = motor(PORT9, false);
int blocksDispensed = 0;
float hopAngDisp = 0.0;
float hopAngStep = 90.0;

//blocks and stacks
bool atBlueStack;
bool atRedStack;
bool blockInClaws;
bool blockFailedGrab;
int redStackAmt = 0;
int blueStackAmt = 0;
float blockDim = 30; //mm
float blockBufferSpace = 2; //mm
float redZonePos = 480; //mm
float blueZonePos = 30; //mm
float redStackHeight = 0; //mm
float blueStackHeight = 0; //mm

void UpdateScreen(){
    Brain.Screen.clearScreen();
    Brain.Screen.setFont(mono12);
    Brain.Screen.printAt(55, 45, "CURRENT STATE");
    Brain.Screen.printAt(40, 55, stateNames[currentState]);
    Brain.Screen.printAt(55,65, "X: %2.0f mm", currentHorizontalPos);
    Brain.Screen.printAt(55,75, "Y: %2.0f mm", currentVerticalPos);
    if (Optical.color() == red){
        Brain.Screen.printAt(55,85, "RED");
    }
    else if (Optical.color() == blue){
        Brain.Screen.printAt(55,85, "BLUE");
    }
    else{
        Brain.Screen.printAt(55,85, "OTHER");
    }
}

// void Testing(){
//     Brain.Screen.clearScreen();
//     Brain.Screen.setFont(mono12);
//     currentHorizontalPos = (horizontalMotor.position(degrees)*wheelRadius*Deg2rads);
//     currentVerticalPos = (verticalMotor.position(degrees)*gearRadius*Deg2rads);
//     currentClawPos = clawMotor.position(degrees);
//     Brain.Screen.printAt(55, 45, "CURRENT STATE");
//     Brain.Screen.printAt(55, 55, "%2.0f mm", currentHorizontalPos*1000);
//     Brain.Screen.printAt(55, 65, "%2.0f mm", currentVerticalPos*1000);
//     Brain.Screen.printAt(55, 75, "%2.0f deg", currentClawPos);

//     if(Optical.isNearObject()){
//         Optical.setLight(ledState::on);
//         if(Optical.color() == red){
//             Brain.Screen.printAt(55, 85, "RED");
//         }
//         else if(Optical.color() == blue){
//             Brain.Screen.printAt(55, 85, "BLUE");
//         }
//         else{
//             Brain.Screen.printAt(55, 85, "OTHER");
//         }
//     }

//     if (Controller.ButtonR1.pressing()){
//         horizontalMotor.spin(vex::reverse);
//         }
//     else if (Controller.ButtonR2.pressing()){
//         horizontalMotor.spin(vex::fwd);
//     }
//     else {
//         horizontalMotor.stop();
//     }

//     if (Controller.ButtonL1.pressing()){
//         verticalMotor.spin(vex::fwd);
//         }
//     else if (Controller.ButtonL2.pressing()){
//         verticalMotor.spin(vex::reverse);
//     }
//     else {
//         verticalMotor.stop();
//     }

//     if (Controller.ButtonA.pressing()){
//         clawMotor.spin(vex::fwd);
//         }
//     else if (Controller.ButtonB.pressing()){
//         clawMotor.spin(vex::reverse);
//     }
//     else{
//         clawMotor.stop();
//     }

//     if (Controller.ButtonUp.pressing()){
//         hopMotor.spinFor(vex::forward, 100, degrees);
//     }
//     else if (Controller.ButtonDown.pressing()){
//         hopMotor.spinFor(vex::reverse, 100, degrees);
//     }

//     if(hopMotor.current(percent) > motorStallCurrent){
//         hopMotor.spinFor(vex::reverse, 100, degrees);
//         hopMotor.spinFor(vex::forward, 100, degrees);
//     }
// }

void ZeroMotor(motor& Motor, bool& homingFlag, directionType direction){
    //while the motor is not stalling, spin in a certain direction
    while(Motor.current(percent) < motorStallCurrent){
        Motor.spin(direction);
    }
    //when motor stalls, stop motor and home it, set flag to true
    Motor.stop();
    Motor.setPosition(0, degrees);
    homingFlag = true;
}

 void NotAtStack(){
     atRedStack = false;
     atBlueStack = false;
 }

 void AtRedZone(){
     atRedStack = true;
     atBlueStack = false;
 }

 void AtBlueZone(){
     atRedStack = false;
     atBlueStack = true;
 }

void StartUp(){
    UpdateScreen();
    
    //braking mode set
    horizontalMotor.setStopping(hold);
    verticalMotor.setStopping(hold);
    clawMotor.setStopping(hold);
    hopMotor.setStopping(hold);

    //set velocities
    horizontalMotor.setVelocity(15, percent);
    verticalMotor.setVelocity(10, percent);
    clawMotor.setVelocity(65, percent);
    hopMotor.setVelocity(30, percent);

    //by design should start at 0 deg position.
    hopMotor.setPosition(0, degrees);

    //led power 
    Optical.setLightPower(50, percent);

    //home motors 
    ZeroMotor(horizontalMotor, horizontalHomingFlag, vex::fwd);
    ZeroMotor(verticalMotor, verticalHomingFlag, vex::reverse);
    ZeroMotor(clawMotor, clawHomingFlag, vex::reverse);

    //claw is in smaller form factor when it travels 
    while (clawMotor.position(degrees) < clawDefaultPos){
        clawMotor.spin(vex::fwd);
    }
    clawMotor.stop();

    //do not allow to escape method until all motors are homed 
    while(!horizontalHomingFlag || !verticalHomingFlag || !clawHomingFlag){
        wait(20, msec);
    }
}

void DispenseBlock(){

    if(blocksDispensed >= 6){
        currentState = END;
        return;
    }
        
    blocksDispensed++;
    float targetAngle = hopAngStep * blocksDispensed;
    float prevAngle = hopAngStep * (blocksDispensed - 1);
    bool completedStep = false;
    
    while(hopMotor.position(degrees) < targetAngle){
        // check if we've successfully completed 100 deg
        if(hopMotor.position(degrees) >= prevAngle + 100){
            completedStep = true;
        }

        // if(hopMotor.current(percent) > motorStallCurrent){
        //     if(completedStep){
        //         // stalling on next block, just back off
        //         hopMotor.spinFor(vex::reverse, 100, degrees);
        //         break;
        //     }
        //     else{
        //         // stalling on current block, do full correction
        //         hopMotor.spinFor(vex::reverse, 100, degrees);
        //         hopMotor.spinFor(vex::forward, 100, degrees);
        //     }
        //}
        hopMotor.spin(vex::forward);
    }
    hopMotor.stop();
    hopAngDisp = hopMotor.position(degrees);
}

void TraverseToPickupZone(){

    if (atRedStack){
        while(currentHorizontalPos >= pickupPos_horizontal){
            currentHorizontalPos = abs(horizontalMotor.position(degrees)*wheelRadius*Deg2rads*1000);
            horizontalMotor.spin(vex::fwd);
        }
    }
    else if(atBlueStack){
        while(currentHorizontalPos <= pickupPos_horizontal){
            currentHorizontalPos = abs(horizontalMotor.position(degrees)*wheelRadius*Deg2rads*1000);
            horizontalMotor.spin(vex::reverse);
        }
    }
    else{
       while(currentHorizontalPos <= pickupPos_horizontal){
            currentHorizontalPos = abs(horizontalMotor.position(degrees)*wheelRadius*Deg2rads*1000);
            horizontalMotor.spin(vex::reverse);
        } 
    }
    
    //reverse is away from home fwd is towards home 
    horizontalMotor.stop();
}

void DescendToBlock(){
    while(currentVerticalPos <= pickupPos_vertical){
        currentVerticalPos = abs(verticalMotor.position(degrees)*gearRadius*Deg2rads*1000);
        verticalMotor.spin(vex::fwd);
        //fwd is down
        //rev is up 
    }
    verticalMotor.stop();
}

void Grab(){
    while(currentClawPos < clawClampPos){
        currentClawPos = clawMotor.position(degrees);
        clawMotor.spin(vex::fwd);
    }
    clawMotor.stop();
    //Optical.setLight(ledState::on);

    while(Optical.color() != red && Optical.color() != blue){
        detectedColor = Optical.color();
        wait(20, msec);
    }

    detectedColor = Optical.color();
    blockInClaws = true;
    blockFailedGrab = false;
}

void Ascend(){
    while(currentVerticalPos > 1.0){
            currentVerticalPos = abs(verticalMotor.position(degrees)*gearRadius*Deg2rads*1000);
        verticalMotor.spin(vex::reverse);
    }
    verticalMotor.stop();
    currentVerticalPos = 0;
}

void TraverseToRedZone(){
    while(currentHorizontalPos < redZonePos){
        currentHorizontalPos = abs(horizontalMotor.position(degrees)*wheelRadius*Deg2rads*1000);
        horizontalMotor.spin(vex::reverse); // away from home
    }
    horizontalMotor.stop();
    //Optical.setLight(ledState::off);
    AtRedZone();
}

void TraverseToBlueZone(){
    while(currentHorizontalPos > blueZonePos){
        currentHorizontalPos = abs(horizontalMotor.position(degrees)*wheelRadius*Deg2rads*1000);
        horizontalMotor.spin(vex::fwd);
    }
    horizontalMotor.stop();
    //Optical.setLight(ledState::off);
    AtBlueZone();
}

void DescendToStack(float stackHeight){
    while(currentVerticalPos < stackHeight){
        currentVerticalPos = abs(verticalMotor.position(degrees)*gearRadius*Deg2rads*1000);
        verticalMotor.spin(vex::fwd);
    }
    verticalMotor.stop();
}

void Release(){
    while(currentClawPos > clawDefaultPos){
        currentClawPos = clawMotor.position(degrees);
        clawMotor.spin(vex::reverse);
    }
    clawMotor.stop();
    blockInClaws = false;
    detectedColor = black; //colour reset 
}

void End(){
    horizontalMotor.stop();
    verticalMotor.stop();
    clawMotor.stop();
    hopMotor.stop();
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(55, 55, "COMPLETE");
}

void EvaluateState(State state){
    switch (state){
        case STARTUP: 
            StartUp();
            currentState = DISPENSE_BLOCK;
            break;
        
        case DISPENSE_BLOCK: 
            DispenseBlock();
            currentState = TRAVERSE_TO_PICKUP_ZONE;
            break;

        case TRAVERSE_TO_PICKUP_ZONE:
            TraverseToPickupZone();
            NotAtStack(); 
            currentState = DESCEND_TO_BLOCK;
            break;

        case DESCEND_TO_BLOCK:
            DescendToBlock();
            currentState = GRAB;
            break;

        case GRAB:
            Grab();
            currentState = ASCEND;
            break;

        case ASCEND:        
            Ascend();

            if (blockInClaws){
                if (detectedColor == red){
                    currentState = TRAVERSE_TO_RED_ZONE;
                }
                else if (detectedColor == blue){
                    currentState = TRAVERSE_TO_BLUE_ZONE;
                }
            }
            else if(blockFailedGrab){
                currentState = DESCEND_TO_BLOCK;
            }
            else{
                currentState = DISPENSE_BLOCK;
            }
            
            break;

        case TRAVERSE_TO_RED_ZONE: 
            TraverseToRedZone();
            currentState = DESCEND_TO_STACK;
            break;
         
        case TRAVERSE_TO_BLUE_ZONE: 
            TraverseToBlueZone();
            currentState = DESCEND_TO_STACK;
            break;

        case DESCEND_TO_STACK: 
        if(atRedStack){
            if (redStackAmt == 0){
                targetHeight = pickupPos_vertical;
                redStackAmt++;
            }
            else{
                targetHeight = pickupPos_vertical - redStackAmt * (blockDim);
                redStackAmt++;
            }
            DescendToStack(targetHeight);
        } 
        else if(atBlueStack){
            if (blueStackAmt == 0){
                targetHeight = pickupPos_vertical;
                blueStackAmt++;
            }
            else{
                targetHeight = pickupPos_vertical - blueStackAmt * (blockDim);
                blueStackAmt++;
            }
            DescendToStack(targetHeight);
        }
        currentState = RELEASE;
        break;

        case RELEASE: 
            Release();
            
            if (blueStackAmt + redStackAmt >= 6){
                currentState = END;
            }
            
            else{
                currentState = ASCEND;
            }
            break;

        case END: 
            End();
            break;

        default: 
            Brain.Screen.clearScreen();
            Brain.Screen.printAt(55, 55, "UNKNOWN STATE");
            break;
    }
}

int screenTask(){
    while(true){
        UpdateScreen();
        wait(100, msec); 
    }
    return 0;
}

int main() {
    vex::thread screenThread(screenTask);

    while(true) {
        
        EvaluateState(currentState);
        //Testing();

        wait(20, msec);
    }

}





