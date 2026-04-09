/*
///////////////////////////////////////////////////////
// Simple class to extract full pose information from Apriltags 
// detected by Vex Robotics AI-Vision Sensor. 

  vex_apriltag  © 2025 by John Enright is licensed under CC BY-NC-SA 4.0. 
  To view a copy of this license, 
  visit https://creativecommons.org/licenses/by-nc-sa/4.0/

  Note: The license simply covers the portions of the code used to glue together Vex 
  projects and the UMich AprilTag library. This does not affect the distribution license of
  the other code.

*/
#ifndef __vex_apriltag
#define __vex_apriltag

#pragma once

//#include "matd.h"
//#include "apriltag_pose.h"
#include "vex.h"

#define BAD_TAG_HOMOGRAPHY (-1)

typedef struct {
    double R[3][3];
    double t[3];
    int id;
} vex_apriltag_pose;

typedef struct {
    double focal_length[2];
    double principal_point[2];
    double image_size[2];
} avs_calibration;

int calculate_tag_pose(const vex::aivision::object *tag_obj, avs_calibration *calib, double tag_size, vex_apriltag_pose *pose);


#endif
