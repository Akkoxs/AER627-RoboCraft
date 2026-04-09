/*
    Simple virtual motor classes for mobile robot chassis

  vex_apriltag  © 2025 by John Enright is licensed under CC BY-NC-SA 4.0.
  To view a copy of this license,
  visit https://creativecommons.org/licenses/by-nc-sa/4.0/

*/
#pragma once

// This is an abstract class for interfacing with drive motors.
class rover_motor
{
public:
  virtual int start(void);
  virtual int stop(void);
  virtual bool move_to_position(double tgt_pos_rad);
  virtual void set_position(double new_pos_rad);
  virtual double get_position(void);
  virtual void set_velocity(double omega_rad); // Sets velocity, but does not start the motor
  virtual double get_velocity(void);
};