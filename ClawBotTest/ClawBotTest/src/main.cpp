/*----------------------------------------------------------------------------------------*/
/*                                                                                        */
/*    Project:          ClawBot Controller With Events                                    */
/*    Module:           main.cpp                                                          */
/*    Author:           VEX                                                               */
/*    Description:      The Left up/down Controller Axis (3) will control the             */
/*                      speed of the left motor.                                          */
/*                      The Right up/down Controller Axis (2) will control the            */
/*                      speed of the right motor.                                         */
/*                      The Left up/down Controller Buttons will control the Arm.         */
/*                      The Right up/down Controller Buttons will control the Claw.       */
/*                                                                                        */
/*    Configuration:    Left Motor in Port 6                                              */
/*                      Right Motor in Port 10                                            */
/*                      Arm Motor in Port 3                                               */
/*                      Claw Motor in Port 4                                              */
/*                      Controller                                                        */
/*                                                                                        */
/*----------------------------------------------------------------------------------------*/

// Include the VEX Library
#include "vex.h"
  
// Allows for easier use of the VEX Library
using namespace vex;

// Brain should be defined by default
brain Brain;

// Robot configuration code.
inertial BrainInertial = inertial();

motor LeftMotor = motor(PORT6, false);
motor RightMotor = motor(PORT10, true);
motor ArmMotor = motor(PORT3, false);
motor ClawMotor = motor(PORT4, false);

controller Controller = controller();

// Callback function when Controller ButtonL1 is pressed
void onButtonL1Press() {
  // Spinning the ArmMotor in forward raises the Arm
  ArmMotor.spin(forward);

  // Wait until ButtonL1 is released
  while (Controller.ButtonL1.pressing()) {
    wait(20, msec);
  }

  ArmMotor.stop();
}

// Callback function when Controller ButtonL2 is pressed
void onButtonL2Press() {
  // Spinning the ArmMotor in reverse lowers the Arm
  ArmMotor.spin(reverse);

  // Wait until ButtonL2 is released
  while (Controller.ButtonL2.pressing()) {
    wait(20, msec);
  }

  ArmMotor.stop();
}

// Callback function when Controller ButtonR1 is pressed
void onButtonR1Press() {
  // Spinning the ClawMotor forward closes the Claw
  ClawMotor.spin(forward);

  // Wait until ButtonR1 is released
  while (Controller.ButtonR1.pressing()) {
    wait(20, msec);
  }

  ClawMotor.stop();
}

// Callback function when Controller ButtonR2 is pressed
void onButtonR2Press() {
  // Spinning the ClawMotor in reverse opens the Claw
  ClawMotor.spin(reverse);

  // Wait until ButtonR2 is released
  while (Controller.ButtonR2.pressing()) {
    wait(20, msec);
  }

  ClawMotor.stop();
}

int main() {
  // Begin project code
  
  // Register event handlers and pass callback functions
  Controller.ButtonL1.pressed(onButtonL1Press);
  Controller.ButtonL2.pressed(onButtonL2Press);
  Controller.ButtonR1.pressed(onButtonR1Press);
  Controller.ButtonR2.pressed(onButtonR2Press);

  // Set default motor stopping behavior and velocity
  ArmMotor.setStopping(hold);
  ClawMotor.setStopping(hold);
  ArmMotor.setVelocity(60, percent);
  ClawMotor.setVelocity(30, percent);

  // Loop to check Controller Axis positions and set motor velocity
  while (true) {
    LeftMotor.setVelocity(Controller.Axis3.position(), percent);
    RightMotor.setVelocity(Controller.Axis2.position(), percent);

    LeftMotor.spin(forward);
    RightMotor.spin(forward);

    wait(20, msec);
  }
}
