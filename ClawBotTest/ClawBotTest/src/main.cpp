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
#include "vex.h" //don't touch the vex header file in the include directory 
  
// Allows for easier use of the VEX Library
using namespace vex;

// Brain should be defined by default
brain Brain; //brain reference 

// Robot configuration code.
inertial BrainInertial = inertial(); //reference to the brains internal inertial sensor, keeps track of heading, orientation, acceleration, gyros, etc.

//here we are referencing and definining the different motors
//the motor() class takes in two arguments (PORT#, isReversed)
//the PORT corresponds to the physical numbered ports on the brain. the isReversed defines if the motor spins in its defined positive direction 
motor LeftMotor = motor(PORT6, false); 
motor RightMotor = motor(PORT10, true);
motor ArmMotor = motor(PORT3, false);
motor ClawMotor = motor(PORT4, false);

controller Controller = controller(); //referencing the controller.

// Callback function when Controller ButtonL1 is pressed
void onButtonL1Press() {
  // Spinning the ArmMotor in forward raises the Arm
  ArmMotor.spin(forward); //access spin() method within ArmMotor (an object of the motor class), which spins the motor 

  // Wait until ButtonL1 is released
  //this is kinda like a capture loop, this onButtonL1Press() method is called when the button is pressed on the controller
  //this loop ensures that the code does not proceed while the button is pressed down, it checks this condition every 20ms
  //when the button is eventually let go, the code exits this while loop and proceeds to the ArmMotor.stop()
  while (Controller.ButtonL1.pressing()) { 
    wait(20, msec);
  }

  ArmMotor.stop();
}

// Callback function when Controller ButtonL2 is pressed
void onButtonL2Press() {
  // Spinning the ArmMotor in reverse lowers the Arm
  //important to understand that even if a motor is defined in the reverse direction, you can still spin it in the reverse if required.
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

//On startup of the program, this is what actually runs, none of the methods above would run if we didn't call them INSIDE this main()
// apparently main is an int type method because it returns a 1 or 0 based on whether it successfuly completed or not to the OS.
// you'll see that other functions above are of the type void (they don't return anything)
int main() { 
  // Begin project code
  
  // Register event handlers and pass callback functions
  // here we are basically taking the controller (an object of the controller class we defined above), and accessing its ButtonL1 class
  // and then calling its pressed() method and assigning our own method to it, 
  //we are basically telling the controller that when ButtonL1 is pressed, to call our method onButtonL1Press (we defined above)
  Controller.ButtonL1.pressed(onButtonL1Press);
  Controller.ButtonL2.pressed(onButtonL2Press);
  Controller.ButtonR1.pressed(onButtonR1Press);
  Controller.ButtonR2.pressed(onButtonR2Press);

  // Set default motor stopping behavior and velocity
  //the setStopping method sets the motor to a certain brakeType (an enum)
  //theres a few braketypes:
  //coast - (default) allows motor to gradually stop
  //brake - stops motor immediatly 
  //hold - stops the motor immediatly and holds the motor in whatever position it was stopped in.
  ArmMotor.setStopping(hold); 
  ClawMotor.setStopping(hold);

  //this sets the velocity of hte motor, below we are setting it to a percentage of the maximum speed
  ArmMotor.setVelocity(60, percent);
  ClawMotor.setVelocity(30, percent);

  // Loop to check Controller Axis positions and set motor velocity
  while (true) { //while true means we always enter this loop and there is no exit condition
    //there are a bunch of axes defined in the controller class:
      //Axis 1: L & R on RHS joystick
      //Axis 2: U & D on RHS joytstick 
      //Axis 3: U & D on LHS joystick
      //Axis 4: L & R on LHS joystick

    //here we are saying, set the velocity of the left motor to the position of the joystick (an integer), and apply it as a percentage of maxspeed.
    //for example if we put the joystick down half way it might return an integer of -0.5, then we convert it to -50% of max speed to go backwards  
    LeftMotor.setVelocity(Controller.Axis3.position(), percent); 
    RightMotor.setVelocity(Controller.Axis2.position(), percent);

    //after setting the velocities of the motors, we spin them
    LeftMotor.spin(forward);
    RightMotor.spin(forward);

    //wait a bit and then repeat the loop (get controller stick positions -> apply to wheels -> repeat)
    //this happens forevever until the program is shut down (because there is no exit condition)
    wait(20, msec);
  }
}
