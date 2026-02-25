/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       kai-s                                                     */
/*    Created:      1/29/2026, 5:19:46 PM                                     */
/*    Description:  EXP project                                               */
/*                                                                            */
/*----------------------------------------------------------------------------*/

//PORTS
//PORT5 = VERTICAL PRISMATIC
//PORT1 = CLAW 
//PORT2 = OPTICAL SENSOR
//PORT3 = HORIZONTAL ROTARY 
//PORT6 = HOPPER 

#include "vex.h"

using namespace vex;

//general
brain Brain;
controller Controller = controller();
optical Optical = optical(PORT2);
float pi = 3.14159265358979323846264; 
float rads2Deg = pi/180;

//state machine
enum State {STARTUP, 
            DISPENSE_BLOCK, 
            TRAVERSE_TO_PICKUP_ZONE,
            DESCEND,
            GRAB, 
            ASCEND, 
            TRAVERSE_TO_RED_ZONE,
            TRAVERSE_TO_BLUE_ZONE,
            RELEASE, 
            END};

//array of pointers to an array of constant chars
const char* stateNames[] = {
    "STARTUP",
    "DISPENSE_BLOCK",
    "TRAVERSE_TO_PICKUP_ZONE",
    "DESCEND",
    "GRAB",
    "ASCEND",
    "TRAVERSE_TO_RED_ZONE",
    "TRAVERSE_TO_BLUE_ZONE",
    "RELEASE",
    "END"};

State currentState;
bool newBlock;

//optical sensor
color detectedColor;
//Optical.objectLost(func);


//gantry
motor horizontalMotor = motor(PORT3, false);
bool horizontalHomingFlag = false;

//claw
motor verticalMotor = motor(PORT5, false);
bool verticalHomingFlag = false;

motor clawMotor = motor(PORT1, false);


//hopper
motor hopMotor = motor(PORT6, false);
int hopStep = 0;
float hopAngDisp = 0.0;
float hopAngStep = 90.0;

//misc 



void StartUp(){
    UpdateScreen();
    hopMotor.setPosition(0, degrees);
    horizontalMotor.setPosition(10, degrees);
    verticalMotor.setPosition(10, degrees);

    //ZeroMotor(horizontalMotor, )
}

void UpdateScreen(){
    Brain.Screen.clearScreen();
    Brain.Screen.setFont(mono12);
    Brain.Screen.printAt(55, 45, "CURRENT STATE");
    Brain.Screen.printAt(55, 55, stateNames[currentState]);
}

void EvaluateState(State state){
    switch (state){
        case 0: //STARTUP 
            //
            break;
        
        case 1: //DISPENSE BLOCK
            //
            break;

        case 2: //TRAVERSE TO PICKUP ZONE
            //
            break;

        case 3: //DESCEND
            //
            break;

        case 4: //GRAB
            //
            break;

        case 5: //ASCEND
            //
            break;

        case 6: //TRAVERSE TO RED ZONE
            //
            break;
         
        case 7: //TRAVERSE TO BLUE ZONE
            //
            break;

        case 8: //RELEASE
            //
            break;

        case 9: //END
            //
            break;

        default: 
            //
            break;
    }
}


void ZeroMotor(motor& Motor, bool& homingFlag){
    Motor.stop();
    Motor.setPosition(0, degrees);
    homingFlag = true;
}

void DispenseBlock(){
    hopStep++;
    
    while (hopAngDisp!= 90*hopStep){
        hopMotor.spin(vex::forward);
    }
        
}

int main() {

    StartUp();


    while(true) {
        
        EvaluateState(currentState);
        UpdateScreen();




        
        wait(20, msec);
    }

}





