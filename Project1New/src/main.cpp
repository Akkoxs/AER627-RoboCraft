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
#include "vex_apriltag.h"

using namespace vex;

//general
brain Brain;
controller Controller = controller();
float pi = 3.14159265358979323846264; 
float rads2Deg = pi/180;

//AVS 
//Calib1 = 20pics 
//Calib2 = 30pics
aivision ai(PORT3, aivision::ALL_TAGS);
avs_calibration calib1 = {.focal_length={218.341554, 218.666466}, .principal_point = {165.629286, 132.445691}, .image_size = {240, 320}};
avs_calibration calib2 = {.focal_length={216.034167, 215.967106}, .principal_point = {165.998499, 133.968668}, .image_size = {240, 320}};
static const double tag_size_A = 0.045; //m
static const double tag_size_B = 0.022; //m
static const double tag_size_C = 0.016; //m
static const double tag_size_D = 0.010; //m

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
float N_small = 48.0;
float N_large = 60.0;
//float backLashComp = 2.0; //degrees due to play in arm

//pointer lore
//here, we are passing in the actual memory location of the args Motor and homingFlag, they are not local to this method, this will modify what you pass in.
void ZeroMotor(motor& Motor, bool& homingFlag){
    Motor.stop();
    Motor.setPosition(0, degrees);
    homingFlag = true;
    }

// jenright print_pose method 
static void print_pose(vex_apriltag_pose *the_pose) {
    printf("Tag ID: %d\n", the_pose->id);
    printf("Translation: X:%lf Y:%lf Z:%lf\n", the_pose->t[0], the_pose->t[1], the_pose->t[2]);
    printf("X Axis: [%lf, %lf, %lf]\n", the_pose->R[0][0], the_pose->R[0][1], the_pose->R[0][2]);
    printf("Y Axis: [%lf, %lf, %lf]\n", the_pose->R[1][0], the_pose->R[1][1], the_pose->R[1][2]);
    printf("Z Axis: [%lf, %lf, %lf]\n", the_pose->R[2][0], the_pose->R[2][1], the_pose->R[2][2]);
}

//jenright april tag capture method, copied from template project 
int AprilTagCapture(){  
    while (true) {

        ai.takeSnapshot(aivision::ALL_TAGS);
        Brain.Screen.setFont(mono12);
        Brain.Screen.printAt(45, 90, "                       ");
        Brain.Screen.printAt(45, 90, "Tags Found: %d\n", ai.objectCount);

        if (ai.objectCount > 0) {
            vex_apriltag_pose my_pose;

            //using tag size A & calib1 params by default 
            int ret = calculate_tag_pose(
                &ai.objects[0],
                &calib1,
                tag_size_A,
                &my_pose
            );

            if (ret == 0) {
                print_pose(&my_pose);
            } 
            else {
                printf("Unable to compute homography\n");
            }
        }

        this_thread::sleep_for(1000);
    }

    return 0;
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

    //start april tag recognition thread
    thread apriltagThread(AprilTagCapture);

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
    RotaryMotor.setVelocity(45, percent);
    PrismaticMotor.setVelocity(25, percent);

    //homing initial conditions, forced to start homing. 
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
                RotaryMotor.spin(vex::forward); //this is actually the reverse homing motion bc the motor is flipped. 
            }
        }
        else { //once homing flag is up, give us control 
            if (Controller.ButtonR1.pressing() && !(RotaryBumper.pressing())){
                RotaryMotor.spin(vex::forward);
            }
            else if (Controller.ButtonR2.pressing() && rotAngDisp < rotMaxAngDisp){
                RotaryMotor.spin(reverse);
            }
            else {
                RotaryMotor.stop();
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

        //displayed position calculations 
        rotAngDisp = abs(RotaryMotor.position(degrees)*(N_small/N_large)*(N_large/N_large));
        prisDisp = (PrismaticMotor.position(degrees)*gearRadius*(rads2Deg));
        Brain.Screen.printAt(20, 55, "%3.0f deg", rotAngDisp);
        Brain.Screen.printAt(90, 55, "%2.0f mm", prisDisp*1000);

        wait(20, msec);
    }
}