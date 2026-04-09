/*
    Simple motor classes for mobile robot chassis and vex robot kits

  vex_apriltag  © 2025 by John Enright is licensed under CC BY-NC-SA 4.0.
  To view a copy of this license,
  visit https://creativecommons.org/licenses/by-nc-sa/4.0/

*/
#pragma once

#include "rover_motor.h"
#include "vex.h"

using namespace vex;

// Derived object to interface with the kinematic drivetrain objects AND vex motors.
class vex_rover_motor : public rover_motor
{
public:
    vex_rover_motor(int32_t port_num); // Constructor
    // Start the motors moving at the configured speed
    int start(void);
    // Stop the motion
    int stop(void);
    // Move to specified angle. The call is currently blocking.
    bool move_to_position(double tgt_pos_rad);
    // These send commands to the motors, not just storing member property values.
    // Set the current internal rotation position
    void set_position(double new_pos_rad);
    // Return the current position of the motor.
    double get_position(void);
    // This call sets the default velocity to use during motion
    void set_velocity(double omega_rad);
    // This returns the actual, current velocity
    double get_velocity(void);

private:
    int32_t m_motor_port;
    vex::motor m_motor;
};