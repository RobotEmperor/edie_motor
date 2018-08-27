/*
 * motor_algorithm.cpp
 *
 *      Author: robotemperor
 */
#include <robot1/motor_algorithm.h>

//////////////////////////////////////////////////////////////////////////////
TrajectoryGenerator::TrajectoryGenerator()
{
  pre_desired_value = 0;
  current_desired_value = 0;
  out_value = 0;
  time_count = 0;
  tra_done_check = 0;

}
TrajectoryGenerator::~TrajectoryGenerator()
{

}
double TrajectoryGenerator::trapezoidal_function(double desired_value, double time)
{
  if(current_desired_value != desired_value)
    {
      time_count = 0;
      tra_done_check = false;
      pre_desired_value = out_value;
    }
    current_desired_value = desired_value;

    time_count = time_count + 0.01;

    if(time_count >= time)
    {
      tra_done_check = true;
      pre_desired_value = desired_value;
      return pre_desired_value;
    }

    if(pre_desired_value != desired_value && tra_done_check == false)
    {
      if(pre_desired_value > desired_value) // 하강 트레젝토리 y = -at + b
      {
        out_value = -((pre_desired_value - desired_value)/time)*time_count + pre_desired_value;
        return out_value;
      }
      if(pre_desired_value < desired_value)// 상승 트레젝토리 y = at + b
      {
        out_value = ((desired_value - pre_desired_value)/time)*time_count + pre_desired_value;
        return out_value;
      }
    }
    else
    {
      return pre_desired_value;
    }
}
//////////////////////////////////////////////////////////////////////////////
void initialize()
{
  motor1 = new DcMotorForRaspberryPi(399,100,2);
  motor2 = new DcMotorForRaspberryPi(399,100,2);

  current_desried_speed_motor1 = 0;
  current_desried_speed_motor2 = 0;
}
void motor1_encoder_1(void)
{
  motor1->encoder_pulse1 ++;
  //  encoder_pulse_motor1_position ++;
}
void motor1_encoder_2(void)
{
  motor1->encoder_pulse2++;
  //  encoder_pulse_motor2_position ++;
}

void motor2_encoder_1(void)
{
  motor2->encoder_pulse1 ++;
  //  encoder_pulse_motor1_position ++;
}
void motor2_encoder_2(void)
{
  motor2->encoder_pulse2 ++;
  //  encoder_pulse_motor2_position ++;
}
void motor_callback1(const robot1::motor_cmd::ConstPtr& msg)
{
  /*motor_direction_motor1 = msg->motor_desired_direction;
  speed_motor1 = msg->motor_desired_rpm;
  angle_motor1 = msg->motor_desired_angle;
  motor_onoff_motor1 = msg->motor_onoff;*/
}
void motor_callback2(const robot1::motor_cmd::ConstPtr& msg)
{
  /*  motor_direction_motor2 = msg->motor_desired_direction;
  speed_motor2 = msg->motor_desired_rpm;
  angle_motor2 = msg->motor_desired_angle;
  motor_onoff_motor2 = msg->motor_onoff;*/
}
//test
void motor_theta_dist_callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  motor1->speed_motor = msg->data[0];
  motor2->speed_motor = msg->data[1];

  //encoder_pulse_motor1_position = 0;
  //encoder_pulse_motor2_position = 0;

  //check_position_control1 = false;
  //check_position_control2 = false;
}
/*void algorithm(double angle, double distance)
{
  static int motion_sequence = 1;

  if(check_position_control1 == true && check_position_control2 == true && motion_sequence == 1)
  {
    motion_sequence ++;
    encoder_pulse_motor1_position = 0;
    encoder_pulse_motor2_position = 0;
    check_position_control1 = false;
    check_position_control2 = false;
    printf("Motion change \n");
  }
  else if(check_position_control1 == true && check_position_control2 == true && motion_sequence == 2)
  {
    motion_sequence ++;
    angle_control_done_msg.data = "done";
    angle_control_done_pub.publish(angle_control_done_msg);
    angle_motor1 = 0;
    angle_motor2 = angle_motor1;
    printf("Motion done! \n");
  }
  else if(check_position_control1 == true && check_position_control2 == true && motion_sequence == 3)
  {
    printf("waiting! \n");
  }
  else if(check_position_control1 == false && check_position_control2 == false && motion_sequence == 3)
  {
    motion_sequence = 1;
    printf("motion init! \n");
  }
  else
  {
    printf("running! \n");
  }


  switch(motion_sequence)
  {
  case 1 :
  {
    position_max_output_common = 20;

    angle_motor1 = (int) fabs(angle*2);
    angle_motor2 = angle_motor1;

    if(angle < 0)
    {
      motor_direction_motor1 = false;
      motor_direction_motor2 = false;
    }
    else
    {
      motor_direction_motor1 = true;
      motor_direction_motor2 = true;
    }
    break;
  }
  case 2 :
  {
    position_max_output_common = 80;
    angle_motor1 = (int) ((fabs(distance)/0.035)*180)/M_PI;
    angle_motor2 = angle_motor1;

    if(distance > 0)
    {
      motor_direction_motor1 = true;
      motor_direction_motor2 = false;
    }
    else
    {
      motor_direction_motor1 = false;
      motor_direction_motor2 = true;
    }

    break;
  }
  default :
    break;
  }
}*/

//////////////////////////////////////////////////////////////////////////////
/*int position_control(int* encoder_read_position, int desired_position, int max_out_put, bool* check)
{
  int static_encoder_pulse = 0;
  int result_position  = 0;
  int position_error = 0;
  int control = 0;
  double P_gain_position = 0.5;
  static int speed_output = 0;

  static_encoder_pulse = *encoder_read_position;
  result_position = (int) (static_encoder_pulse*(399/360));// pulse 11 gear ratio 34

  if(desired_position <= result_position)
  {
    //printf("result_position = %d \n ", result_position);
    static_encoder_pulse = 0;
    speed_output = 0;
 *check = true;
    return 0; // 0
  }

  position_error = desired_position - result_position;


  control = (int)( P_gain_position * (double) position_error);

  if(control > 0)
  {
    if(control > 2)
    {
      control = 2;
    }

    speed_output = (speed_output + control);

    if (speed_output > max_out_put)
    {
      speed_output = max_out_put;
    }

  }
  else if(control < 0)
  {

    if(control < - 2)
    {
      control = - 2;
    }

    speed_output =  (speed_output +  control);

    if (speed_output < max_out_put)
    {
      speed_output = 0;
    }
  }
  return speed_output;
}*/

/////////////////////////////////////////////////////////////////////////////////////
void motor_control(int id, int motor_line1, int motor_line2, bool direction, int desired_speed_rpm, int angle, bool on_off)
{
  if(on_off == true)
  {
    if(desired_speed_rpm == 0 && angle == 0)
    {
      //  digitalWrite(motor_line1,LOW);
      //  digitalWrite(motor_line2,LOW);
    }
    else
    {
      if(direction == true)//CW
      {
        digitalWrite(motor_line1,HIGH);
        //  digitalWrite(motor_line2,LOW);
      }
      else if (direction == false)//CCW
      {
        digitalWrite(motor_line1,LOW);
        //  digitalWrite(motor_line2,HIGH);
      }
    }

    switch (id)
    {
    case 1 :
      //if(angle != 0)
      //  {
      //    desired_speed_rpm = position_control(&encoder_pulse_motor1_position, angle,  position_max_output_common, &check_position_control1);
      //  }
      motor1->speed_controller(desired_speed_rpm);
      break;
    case 2 :
      //if(angle != 0)
      //{
      //desired_speed_rpm = position_control(&encoder_pulse_motor2_position, angle,  position_max_output_common, &check_position_control2);
      //}
      motor2->speed_controller(desired_speed_rpm);
      break;
    default :
      break;
    }
  }

  if(on_off == false)
  {
    pwmWrite(motor1_PWM, 0);
    pwmWrite(motor2_PWM, 0);
  }
}

void controlFunction(const ros::TimerEvent&)
{
  motor1->onoff = 1;
  motor2->onoff = 1;

  current_desried_speed_motor1 = tra_motor1.trapezoidal_function(motor1->speed_motor, 5);
  current_desried_speed_motor2 = tra_motor2.trapezoidal_function(motor2->speed_motor, 5);
  motor_control(1, motor1_IN1, 0,  motor1->direction, current_desried_speed_motor1 , motor1->angle_motor, motor1->onoff);
  motor_control(2, motor2_IN1, 0,  motor2->direction, current_desried_speed_motor2 , motor2->angle_motor, motor2->onoff);

  pwmWrite(motor1_PWM, (int) motor1->pwm_value_motor);
  pwmWrite(motor2_PWM, (int) motor2->pwm_value_motor);

}
int main (int argc, char **argv)
{
  wiringPiSetupGpio();

  initialize();

  ros::init(argc, argv, "motor_node");
  ros::NodeHandle nh;

  ros::Timer timer_control = nh.createTimer(ros::Duration(0.01), controlFunction); // 10ms

  motor_theta_dist_sub   = nh.subscribe("motor_theta_dist", 1, motor_theta_dist_callback); // test
  angle_control_done_pub = nh.advertise<std_msgs::String>("angle_control_done",10);
  result_rpm1_pub = nh.advertise<std_msgs::Float64>("result_rpm1",10);
  result_rpm2_pub = nh.advertise<std_msgs::Float64>("result_rpm2",10);

  desired_rpm1_pub = nh.advertise<std_msgs::Float64>("desired_rpm1",10);
  desired_rpm2_pub = nh.advertise<std_msgs::Float64>("desired_rpm2",10);

  pinMode(motor1_IN1, OUTPUT);
  pinMode(motor1_FG1, INPUT);
  pinMode(motor1_FG2, INPUT);

  pinMode(motor2_IN1, OUTPUT);
  pinMode(motor2_FG1, INPUT);
  pinMode(motor2_FG2, INPUT);

  wiringPiISR(motor1_FG1, INT_EDGE_RISING, &motor1_encoder_1);
  wiringPiISR(motor1_FG2, INT_EDGE_RISING, &motor1_encoder_2);
  wiringPiISR(motor2_FG1, INT_EDGE_RISING, &motor2_encoder_1);
  wiringPiISR(motor2_FG2, INT_EDGE_RISING, &motor2_encoder_2);

  pinMode(motor1_PWM, PWM_OUTPUT);
  pinMode(motor2_PWM, PWM_OUTPUT);
  pwmSetMode (PWM_MODE_MS);
  pwmSetRange(512);
  pwmSetClock(2);

  digitalWrite(motor1_IN1,HIGH);
  digitalWrite(motor2_IN1,LOW);


  pwmWrite(motor1_PWM, 0);
  pwmWrite(motor2_PWM, 0);

  while(ros::ok())
  {
    desired_rpm1_msg.data = current_desried_speed_motor1;
    desired_rpm2_msg.data = current_desried_speed_motor2;
    result_rpm1_msg.data = motor1->result_rpm;
    result_rpm2_msg.data = motor2->result_rpm;
    usleep(100);
    result_rpm1_pub.publish(result_rpm1_msg);
    result_rpm2_pub.publish(result_rpm2_msg);
    desired_rpm1_pub.publish(desired_rpm1_msg);
    desired_rpm2_pub.publish(desired_rpm2_msg);

    ros::spinOnce();
  }

  pwmWrite(motor1_PWM, 0);
  pwmWrite(motor2_PWM, 0);

  return 0;
}


