/*
    Simple wheel kinematics classes for mobile robot chassis

  vex_apriltag  © 2025 by John Enright is licensed under CC BY-NC-SA 4.0.
  To view a copy of this license,
  visit https://creativecommons.org/licenses/by-nc-sa/4.0/

*/

#include "rover_wheel.h"

// #include "EmbeddedMath.hpp"
#include <math.h>
#include <iostream>

// Could do this with an iterator, but this is probably fine.
#define drive_train_wheels(varname, list) \
  rover_wheel **varname = list;           \
  *varname;                               \
  varname++

// namespace Eigen = EmbeddedMath;  // Alias EmbeddedMath to Eigen
rover_wheel::rover_wheel(double L, double R, double alpha, double beta, double gear_ratio) : rover_wheel(L, R, alpha, beta, gear_ratio, (rover_motor *)NULL)
{
  // Nothing else to do
}
rover_wheel::rover_wheel(double L, double R, double alpha, double beta, double gear_ratio, rover_motor *motor)
{
  set_L(L);
  set_R(R);
  set_alpha(alpha);
  set_beta(beta);
  set_gear_ratio(gear_ratio);
  if (motor)
  {
    assign_motor(motor);
  }
}

// Set the velocity and and start the motor moving if necessary.
int rover_wheel::set_speed(double phi_dot_rps)
{
  if (m_motor)
  {
    m_motor->set_velocity(phi_dot_rps);
    m_motor->start();
    return 0;
  }
  else
  {
    return -1;
  }
}

double rover_wheel::calc_kinematic_speed(double vel[3])
{
  if (vel) // Make sure that vel is not NULL
  {

    // Inner product of velocity and constraint vector
    return vel[0] * m_w[0] + vel[1] * m_w[1] + vel[2] * m_w[2];
  }
  else
  {
    return 0.;
  }
}

int rover_wheel::stop(void)
{
  if (m_motor)
  {
    m_motor->stop();
    return 0;
  }
  else
  {
    return -1;
  }
}
double rover_wheel::get_speed(void)
{
  if (m_motor)
  {
    return m_motor->get_velocity();
  }
  else
  {
    return NAN;
  }
}
double rover_wheel::get_rotation(void)
{
  if (m_motor)
  {
    return m_motor->get_position();
  }
  else
  {
    return NAN;
  }
}
void rover_wheel::reset_rotation(void)
{
  if (m_motor)
  {
    m_motor->set_position(0.);
  }
}
int rover_wheel::assign_motor(rover_motor *motor)
{
  if (motor)
  {
    m_motor = motor;
    return 0;
  }
  else
  {
    m_motor = NULL;
    return -1;
  }

  // Any other sanity checking?
  return (0);
}

// Setters
void rover_wheel::set_L(double L)
{
  m_L = L;
}
void rover_wheel::set_R(double R)
{
  m_R = R;
}
void rover_wheel::set_alpha(double alpha)
{
  m_alpha = alpha;
}
void rover_wheel::set_beta(double beta)
{
  m_beta = beta;
}
void rover_wheel::set_gear_ratio(double gear_ratio)
{
  m_gear_ratio = gear_ratio;
}
// Getters
double rover_wheel::get_L(void)
{
  return m_L;
}
double rover_wheel::get_R(void)
{
  return m_R;
}
double rover_wheel::get_alpha(void)
{
  return m_alpha;
}
double rover_wheel::get_beta(void)
{
  return m_beta;
}
double rover_wheel::get_gear_ratio(void)
{
  return m_gear_ratio;
}

//////////////////////////////////////////////////////////////////
// Standard (fixed) wheel
std_wheel::std_wheel(double L, double R, double alpha, double beta, double gear_ratio) : std_wheel(L, R, alpha, beta, gear_ratio, NULL)
{
}
std_wheel::std_wheel(double L, double R, double alpha, double beta, double gear_ratio, rover_motor *motor) : rover_wheel(L, R, alpha, beta, gear_ratio, motor)
{
}

// Calculate kinematic constraint vector
int std_wheel::calc_kinematics(void)
{
  double ab, b;
  b = get_beta();
  ab = b + get_alpha();
  m_J2 = get_gear_ratio() * get_R();
  m_w[0] = sin(ab) / m_J2;
  m_w[1] = -cos(ab) / m_J2;
  m_w[2] = -get_L() * cos(b) / m_J2;
  return 0;
}
/////////////////////////////////////////////////////////////////
// Omni-wheel  class
omni_wheel::omni_wheel(double L, double R, double alpha, double beta, double gamma, double gear_ratio) : omni_wheel(L, R, alpha, beta, gamma, gear_ratio, (rover_motor *)NULL)
{
  // nothing else to do here.
}

omni_wheel::omni_wheel(double L, double R, double alpha, double beta, double gamma, double gear_ratio, rover_motor *motor) : rover_wheel(L, R, alpha, beta, gear_ratio, motor)
{
  set_gamma(gamma);
}
void omni_wheel::set_gamma(double gamma)
{
  m_gamma = gamma;
}
double omni_wheel::get_gamma(void)
{
  return m_gamma;
}

int omni_wheel::calc_kinematics()
{
  double abg, bg;
  bg = get_beta() + get_gamma();
  abg = bg + get_alpha();
  m_J2 = get_gear_ratio() * get_R() * cos(get_gamma());
  m_w[0] = sin(abg) / m_J2;
  m_w[1] = -cos(abg) / m_J2; // originally negative, but I think we invert that to get proper motion
  m_w[2] = -get_L() * cos(bg) / m_J2;
  return 0;
}

// Drivetrain implementation
kinematic_drivetrain::kinematic_drivetrain(rover_wheel **wheel_list)
{
  m_num_wheels = 0;
  m_wheels = wheel_list; // register list address
  for (drive_train_wheels(ptr, m_wheels))
  {
    (*ptr)->calc_kinematics(); // Make sure this is up to date
    m_num_wheels++;
  }
  m_is_init = true;
}

void kinematic_drivetrain::set_heading(double new_heading_rad)
{
  m_heading = new_heading_rad;
}
double kinematic_drivetrain::get_heading(void)
{
  return m_heading;
}

// Command drivetrain for absolute motion
void kinematic_drivetrain::calc_wheel_speeds_abs(double velocity_abs[3])
{
  // Rotate velocity into body frame
  double tmp_vel[3];
  double ctheta, stheta;
  ctheta = cos(get_heading());
  stheta = sin(get_heading());
  tmp_vel[0] = ctheta * velocity_abs[0] + stheta * velocity_abs[1];
  tmp_vel[1] = -stheta * velocity_abs[0] + ctheta * velocity_abs[1];
  tmp_vel[2] = velocity_abs[2];
  // Apply this computed relative velocity
  calc_wheel_speeds_rel(tmp_vel);
}
int kinematic_drivetrain::get_num_wheels(void)
{
  return m_num_wheels;
}

// Core function to generate drive commands
void kinematic_drivetrain::calc_wheel_speeds_rel(double velocity_rel[3])
{
  if (m_num_wheels <= 0)
  {
    return;
  }

  double *tmp_speeds = new double[m_num_wheels];
  double max_vel = 0.;
  // Calculate the kinematics speeds
  for (size_t i = 0; i < m_num_wheels; i++)
  {
    double tmp = m_wheels[i]->calc_kinematic_speed(velocity_rel);

    if (fabs(tmp) > max_vel)
    {
      max_vel = fabs(tmp);
    }
    tmp_speeds[i] = tmp;
  }

  // Scale speeds if needed
  if (max_vel > m_max_speed)
  {

    double vel_scale = 1.;
    vel_scale = m_max_speed / max_vel;
    for (size_t i = 0; i < m_num_wheels; i++)
    {
      tmp_speeds[i] = vel_scale * tmp_speeds[i];
    }
  }
  // Send speed commands to motors
  if (m_is_init)
  {
    for (size_t i = 0; i < m_num_wheels; i++)
    {
      m_wheels[i]->set_speed(tmp_speeds[i]);
    }
  }

  delete[] tmp_speeds;
}
