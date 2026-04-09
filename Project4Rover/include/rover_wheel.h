/*
    Simple wheel kinematics classes for mobile robot chassis

  vex_apriltag  © 2025 by John Enright is licensed under CC BY-NC-SA 4.0.
  To view a copy of this license,
  visit https://creativecommons.org/licenses/by-nc-sa/4.0/

*/

#pragma once

#include "rover_motor.h"
#include <math.h>

// This is a base class for rover use. Its member functions define most of the raw behaviours we want from the wheels
class rover_wheel
{

public:
    rover_wheel(double L = 1., double R = 0.1, double alpha = 0., double beta = 0., double gear_ratio = 1.);
    rover_wheel(double L = 1., double R = 0.1, double alpha = 0., double beta = 0., double gear_ratio = 1., rover_motor *motor = NULL);

    // Calculate the constraint vector for vehicle motion
    virtual int calc_kinematics(void) = 0; // This must remain virtual

    // This calculates the necessary wheel speed based on the desired vehicle motion
    double calc_kinematic_speed(double vel[3]);

    // Start the wheel moving at the specified speed.
    int set_speed(double phi_dot_rps);
    // Stop the wheel
    int stop(void);
    // Read current speed
    double get_speed(void);
    // Readback the current (net) rotation of the wheel
    double get_rotation(void);
    // Reset the rotation value to zero
    void reset_rotation(void);
    // Make an assignment between a motor object and the wheel mounting
    int assign_motor(rover_motor *motor);

    // Setters
    // If you change these after the object correction, you will need to call the calc_kinematics() member function to commit the changes
    void set_L(double L);
    void set_R(double R);
    void set_alpha(double alpha);
    void set_beta(double beta);
    void set_gear_ratio(double gear_ratio); // wheel_speed/motor_speed
    // Getters
    double get_L(void);
    double get_R(void);
    double get_alpha(void);
    double get_beta(void);
    double get_gear_ratio(void);

private:
    // Default data properties for class
    double m_L;          // distance from body origin (m)
    double m_R;          // Wheel diameter (m)
    double m_alpha;      // Wheel position angle (from Chassis x-axis)
    double m_beta;       // Angle between wheel vector and rotational axis.
    double m_gear_ratio; // Ratio between motor speed and wheel speed (wheel_speed/motor_speed)
    // double phi{0};
    // bool m_fixed_beta{true};

protected:
    double m_w[3];  // Constraint vector
    double m_J2{0}; // Scaling based on wheel size
    rover_motor *m_motor{NULL};
};

// Derived class for standard wheel. Should also work with steerable wheels.
class std_wheel : public rover_wheel
{
    public:
    std_wheel(double L = 1., double R = 0.1, double alpha = 0., double beta = 0., double gear_ratio = 1.);
    std_wheel(double L = 1., double R = 0.1, double alpha = 0., double beta = 0., double gear_ratio = 1., rover_motor *motor = NULL);
    int calc_kinematics(void);
};

// Omnidirectional (or meccanum) wheel objects.
// Gamma is the angle between the plane of the wheel and the roller axis. You should gauge this looking down at the *bottom* rollers, from above the robot.

class omni_wheel : public rover_wheel
{
public:
    omni_wheel(double L = 1., double R = 0.1, double alpha = 0., double beta = 0., double gamma = M_PI * .25, double gear_ratio = 1.);
    omni_wheel(double L = 1., double R = 0.1, double alpha = 0., double beta = 0., double gamma = M_PI * .25, double gear_ratio = 1., rover_motor *motor = NULL);
    int calc_kinematics(void);
    // Getter/Setter
    void set_gamma(double gamma);
    double get_gamma(void);

private:
    double m_gamma;
};

// Use this to define a compile-time dimensioned array of wheel pointers (NULL terminated).

#define wheel_list(...) {__VA_ARGS__, NULL}

// This object combines an arbitrary number of wheel objects into a functioning drivetrain.
class kinematic_drivetrain
{
public:
    kinematic_drivetrain(rover_wheel **wheel_list);
    void set_heading(double new_heading_rad);
    double get_heading(void);
    // Apply motion relative to globel axes
    void calc_wheel_speeds_abs(double velocity_abs[3]);
    // Apply velocity motion relative to rover axes
    void calc_wheel_speeds_rel(double velocity_rel[3]);

    int get_num_wheels(void);

private:
    rover_wheel **m_wheels;
    int m_num_wheels;
    double m_max_speed{200. / 60. * M_TWOPI};
    double m_heading{0.};
    bool m_is_init{false};
};