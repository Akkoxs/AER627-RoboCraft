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
float pi = 3.14159265358979323846264; 
float rads2Deg = pi/180;

//state flags 
bool newBlock;

//hopper
motor hopMotor = motor(PORT6, false);
int hopStep = 0;
float hopAngDisp = 0.0;
float hopAngStep = 90.0;

void StartUp(){
    hopMotor.setPosition(0, degrees);

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

    while(1) {
       
        // Allow other tasks to run
        this_thread::sleep_for(10);
    }

}





