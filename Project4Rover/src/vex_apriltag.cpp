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

#include "vex_apriltag.h"
#include "matd.h"
#include "apriltag_aux.h"
#include "apriltag_pose.h"

int calculate_tag_pose(const vex::aivision::object *tag_obj, avs_calibration *calib, double tag_size, vex_apriltag_pose *pose)
{
    apriltag_detection_t mydet;
    apriltag_detection_info_t myinfo;
    apriltag_pose_t mypose;

    // Copy details from the object
    mydet.c[0] = tag_obj->centerX;
    mydet.c[1] = tag_obj->centerY;
    // tag corners
    mydet.p[0][0] = tag_obj->tag.x[0];
    mydet.p[1][0] = tag_obj->tag.x[1];
    mydet.p[2][0] = tag_obj->tag.x[2];
    mydet.p[3][0] = tag_obj->tag.x[3];

    mydet.p[0][1] = tag_obj->tag.y[0];
    mydet.p[1][1] = tag_obj->tag.y[1];
    mydet.p[2][1] = tag_obj->tag.y[2];
    mydet.p[3][1] = tag_obj->tag.y[3];
    mydet.H = NULL;
    mydet.id = tag_obj->id;

    myinfo.det = &mydet;
    myinfo.cx = calib->principal_point[0];
    myinfo.cy = calib->principal_point[1];
    myinfo.fx = calib->focal_length[0];
    myinfo.fy = calib->focal_length[1];
    myinfo.tagsize = tag_size;

    update_homography(&mydet);
    if (mydet.H)
    {
        // Success
        estimate_tag_pose(&myinfo, &mypose);
        // Copy information from matd
        pose->id = myinfo.det->id;
        for (size_t m = 0; m < 3; m++)
        {       
            pose->t[m] = MATD_EL(mypose.t,m,0);

            for (size_t n = 0; n < 3; n++)
            {
                pose->R[m][n] = MATD_EL(mypose.R, m,n);
            }
            
        }
        

        matd_destroy(mydet.H);
        matd_destroy(mypose.R);
        matd_destroy(mypose.t);
        return 0;
    }
    else
    {
        // Failed to calculate homography
        return BAD_TAG_HOMOGRAPHY;
    }
}
