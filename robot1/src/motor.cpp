/*
 * motor.cpp
 *
 *      Author: robotemperor
 */
#include <robot1/motor.h>

DcMotorForRaspberryPi::DcMotorForRaspberryPi()
{
}
DcMotorForRaspberryPi::DcMotorForRaspberryPi(int encoder_pulse_per_rotation, int control_freqency, int channel):
encoer_pulse_per_rotation_(encoder_pulse_per_rotation),
control_freqency_(control_freqency),
channel_(channel)

{
  encoder_pulse1 = 0;
  encoder_pulse2 = 0;

  encoder_pulse_position1 = 0;
  encoder_pulse_position2 = 0;

  p_gain_position_ = 0;
  p_gain_speed_ = 0.5;

  pwm_value_motor = 0;

  direction = 0;
  check_position_control= 0;
  onoff = 0;

  speed_motor = 0;
  angle_motor = 0;
  result_rpm = 0;

}
DcMotorForRaspberryPi::~DcMotorForRaspberryPi()
{

}
void DcMotorForRaspberryPi::speed_controller(int desired_speed)
{
  double static_encoder_pulse = 0;
  double speed_error = 0;
  double control = 0;

  static_encoder_pulse = (encoder_pulse1+ encoder_pulse2);
  encoder_pulse1 = 0;
  encoder_pulse2 = 0;
  result_rpm =  (((static_encoder_pulse)*60*control_freqency_)/(encoer_pulse_per_rotation_*channel_));// digital low pass filter  // basic 4 ch



  speed_error = desired_speed  -  result_rpm ;
  control = ( p_gain_speed_ * speed_error);

  pwm_value_motor = (pwm_value_motor + control);

  if (pwm_value_motor > 512)
  {
    pwm_value_motor = 512;
  }

  if (pwm_value_motor < 0)
  {
    pwm_value_motor = 0;
  }

}
