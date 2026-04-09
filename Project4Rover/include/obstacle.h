#pragma once

#include "Pose3D.h"
#include "vex_apriltag_types.h"

class obstacle {
    public:
    //Constructor
    obstacle(double dx, double dy); // This defines a rectangular object.
    obstacle(double r); //This defines a circular object
 
    //Properties
    vex_apriltag_pose tag; // This is the pose relative to the origin.
    double size[2];

    //Methods
    double closest_point(const Pose3D& rover, double rho[3]);

};