/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       jenright                                                  */
/*    Created:      12/16/2025, 4:09:50 PM                                    */
/*    Description:  EXP project                                               */
/*    This is a simple project demonstrating AprilTag recognition.            */
/*    You should modify several aspects of the project for better operation   */
/*    At present, only a single tag is processed but you can easily change this*/
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include "vex_apriltag.h"

using namespace vex;

// A global instance of vex::brain used for printing to the EXP brain screen
vex::brain Brain;

// A global instance of the AI Vision sensor, configured to look for AprilTags. 
// You will need to change the port assignment to match your build
vex::aivision ai(PORT2, aivision::ALL_TAGS);
// A global object representing the calibration of the AVS. This is automatically generated from Matlab and can be copy/pasted.
avs_calibration my_calib = {.focal_length = {226.962562, 226.798419}, .principal_point = {159.685513, 103.705088}, .image_size = {240, 320}};

//Change this based on the measured tag size you are using.
static const double default_tag_size = 0.02; // meters

// Dump the decoded pose information to the debug console
static void print_pose(vex_apriltag_pose *the_pose)
{
    printf("Tag: %d\n", the_pose->id);
    printf("t: %lf %lf %lf\n", the_pose->t[0], the_pose->t[1], the_pose->t[2]);
    for (size_t i = 0; i < 3; i++)
    {
        printf("R: %lf %lf %lf\n", the_pose->R[i][0], the_pose->R[i][1], the_pose->R[i][2]);
    }
}

int main()
{

    Brain.Screen.printAt(2, 30, "Hello EXP");

    while (1)
    {

        // Allow other tasks to run
        auto ai_tags = ai.takeSnapshot(aivision::ALL_TAGS);
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Tags Found: %d\n", ai.objectCount);
        if (ai.objectCount > 0)
        {
            // Should really do this for each detected tag
            vex_apriltag_pose my_pose;
            int ret = calculate_tag_pose(&ai.objects[0], &my_calib, default_tag_size, &my_pose);

            if (ret == 0)
            {
                print_pose(&my_pose);
            }
            else
            {
                printf("Unable to compute homography\n");
            }
        }
        else
        {
            // No tags found
        }

        this_thread::sleep_for(1000);
    }
}
