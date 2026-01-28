/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       kai-s                                                     */
/*    Created:      1/20/2026, 1:03:35 PM                                     */
/*    Description:  EXP project                                               */
/*                                                                            */
/*----------------------------------------------------------------------------*/

//CONTROLS
//Prismatic Joint - L1 = FWD L2 = BACKWD
//Rotary Joint - R1 = CW R2 = CCW

#include "vex.h"

using namespace vex;

//general
brain Brain;
controller Controller = controller();
float pi = 3.14159265358979323846264; 
float rads2Deg = pi/180;

//Prismatic motor 
motor PrismaticMotor = motor(PORT1, false); 
bumper PrismaticBumper = bumper(Brain.ThreeWirePort.A);
float prisDisp = 0.0;
float prisMaxDisp = 0.06; //m
float gearRadius = 0.01375; //m
bool prismaticHomed = false;

//rotary motor
motor RotaryMotor = motor(PORT2, true); 
bumper RotaryBumper = bumper(Brain.ThreeWirePort.B);
float rotAngDisp = 0.0;
float rotMaxAngDisp = 350; //placeholder
bool rotaryHomed = false;

//pointer lore
//here, we are passing in the actual memory location of the args Motor and homingFlag, they are not local to this method, this will modify what you pass in.
void ZeroMotor(motor& Motor, bool& homingFlag){
    Motor.stop();
    Motor.setPosition(0, degrees);
    homingFlag = true;
    }

int main() {
    //Setup brain screen
    Brain.Screen.clearScreen();
    Brain.Screen.setFont(mono12);
    Brain.Screen.print("ROTARY:R1/R2|MAX=XX");
    Brain.Screen.newLine();
    Brain.Screen.print("PRISMATIC:L1/L2|MAX=XX");
    Brain.Screen.newLine();

    Brain.Screen.printAt(20, 45, "ROTARY");
    Brain.Screen.printAt(90, 45, "PRISMATIC");

    //register method calls to buttons 
    //learned something new:
    //normally bumper.pressed() expects a function that takes no args and returns nothing 
    //since we have 2 args, we can wrap the 2 arg method in another that takes none and requires no name, called a lambda function
    //lambda functions are also called anonymous because they have no name and can be executed inline like below.
    RotaryBumper.pressed([]{ZeroMotor(RotaryMotor, rotaryHomed);});
    PrismaticBumper.pressed([]{ZeroMotor(PrismaticMotor, prismaticHomed);});

    //setting brake type of motors (hold in whatever position it stops in)
    PrismaticMotor.setStopping(hold); 
    RotaryMotor.setStopping(hold);

    //speed of motors
    RotaryMotor.setVelocity(50, percent);
    PrismaticMotor.setVelocity(50, percent);

    //homing initial conditions 
    RotaryMotor.setPosition(10, degrees);
    PrismaticMotor.setPosition(10, degrees);

    //main loop
    while(true) {
        
        //rotary controls 
        if(!rotaryHomed){ //first start homing 
            if(RotaryBumper.pressing()){
                ZeroMotor(RotaryMotor, rotaryHomed);
            }
            else{
                RotaryMotor.spin(reverse);
            }
        }
        else { //once homing flag is up, give us control 
            if (Controller.ButtonR1.pressing() && !(PrismaticBumper.pressing())){
                PrismaticMotor.spin(vex::forward);
            }
            else if (Controller.ButtonR2.pressing() && rotAngDisp < rotMaxAngDisp){
                PrismaticMotor.spin(reverse);
            }
            else {
                PrismaticMotor.stop();
            }
        }

        //prismatic controls 
        if(!prismaticHomed){ //first start homing 
            if(PrismaticBumper.pressing()){
                ZeroMotor(PrismaticMotor, prismaticHomed);
            }
            else{
                PrismaticMotor.spin(reverse);
            }
        }
        else { //once homing flag is up, give us control 
            if (Controller.ButtonL1.pressing() && prisDisp < prisMaxDisp){
                PrismaticMotor.spin(vex::forward);
            }
            else if (Controller.ButtonL2.pressing() && !(PrismaticBumper.pressing())){
                PrismaticMotor.spin(reverse);
            }
            else {
                PrismaticMotor.stop();
            }
        }
        
        //limit indication
        if (rotAngDisp <= 0 || rotAngDisp >= rotMaxAngDisp){
            Brain.Screen.printAt(20, 70, "LIM");
        }
        else{
            Brain.Screen.printAt(20, 70, "        ");
        }

        if (prisDisp <= 0 || prisDisp >= prisMaxDisp){
            Brain.Screen.printAt(90, 70, "LIM");
        }
        else{
            Brain.Screen.printAt(90, 70, "        ");
        }

        
        //ang caluclation HERE
        prisDisp = (PrismaticMotor.position(degrees)*gearRadius*(rads2Deg));

        //print current values
        Brain.Screen.printAt(20, 55, "%.3f deg", rotAngDisp);
        Brain.Screen.printAt(90, 55, "%.3f m", prisDisp);

        wait(20, msec);
    }
}