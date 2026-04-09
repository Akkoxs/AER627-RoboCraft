/*
    Simple motor classes for mobile robot chassis and vex robot kits (implementation)

  vex_apriltag  © 2025 by John Enright is licensed under CC BY-NC-SA 4.0.
  To view a copy of this license,
  visit https://creativecommons.org/licenses/by-nc-sa/4.0/

*/

#include "vex_rover_motor.h"
#include "vex.h"

using namespace vex;
const static double DEG2RAD = 0.0174532925199433;
// constructor
vex_rover_motor::vex_rover_motor(int32_t port_num) : m_motor{port_num, false}
{
  m_motor_port = port_num;
}

int vex_rover_motor::start(void)
{
  m_motor.spin(vex::forward);
  return 0;
}

int vex_rover_motor::stop(void)
{
  m_motor.stop(vex::brake); // I think this will stop the motor.
  return 0;
}

bool vex_rover_motor::move_to_position(double tgt_pos_rad)
{
  return m_motor.spinToPosition(tgt_pos_rad / DEG2RAD, vex::deg, true);
}
void vex_rover_motor::set_position(double new_pos_rad)
{
  m_motor.setPosition(new_pos_rad / DEG2RAD, vex::deg);
}
double vex_rover_motor::get_position(void)
{
  return m_motor.position(vex::deg) * DEG2RAD;
}
void vex_rover_motor::set_velocity(double omega_rad)
{
  m_motor.setVelocity(omega_rad / DEG2RAD, vex::dps);
}
double vex_rover_motor::get_velocity(void)
{
  return m_motor.velocity(vex::dps) * DEG2RAD;
}