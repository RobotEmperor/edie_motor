#include <ros/ros.h>
#include <stdio.h>
#include <wiringPi.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <robot1/motor.h>
#include <robot1/robot1_info.h>


void initialize()
{
	CW  = true;
	CCW = false;
	ON  = true;
	OFF = false;

	encoder_pulse1_motor1 = 0;
	encoder_pulse2_motor1 = 0;

    encoder_pulse1_motor2 = 0;
	encoder_pulse2_motor2 = 0;

	encoder_pulse_motor1_position = 0;
	encoder_pulse_motor2_position = 0;

	check_position_control1 = false;
    check_position_control2 = false;

	pwm_value_motor_1 = 0;
	pwm_value_motor_2 = 0;

	P_gain =  0.05; // speed controller p gain


	motor_direction_motor1 = true;
	speed_motor1 = 0;
	angle_motor1 = 0;
    result_rpm1 = 0;
	motor_onoff_motor1 = true;

	motor_direction_motor2 = true;
	speed_motor2 = 0;
	angle_motor2 = 0;
    result_rpm2 = 0;
	motor_onoff_motor2 = true;

    position_max_output_common = 0;    

    reference_angle = 0;
    reference_distance = 0;

	angle_control_done_msg.data.clear();
    result_rpm1_msg.data = 0;
    result_rpm2_msg.data = 0;
    
    desired_rpm1_msg.data = 0;
    desired_rpm2_msg.data = 0;

    pre_static_encoder_pulse1 = 0;
    pre_static_encoder_pulse2 = 0;
}
//////////////////////////////////////////////////////////////////////////////
void motor1_encoder_1(void)
{
	encoder_pulse1_motor1 ++;
//	encoder_pulse_motor1_position ++;
}
void motor1_encoder_2(void)
{
	encoder_pulse2_motor1 ++;
//	encoder_pulse_motor2_position ++;
}
void motor2_encoder_1(void)
{
	encoder_pulse1_motor2 ++;
//	encoder_pulse_motor1_position ++;
}
void motor2_encoder_2(void)
{
	encoder_pulse2_motor2 ++;
//	encoder_pulse_motor2_position ++;
}
void motor_callback1(const robot1::robot1_info::ConstPtr& msg)
{
	motor_direction_motor1 = msg->motor_desired_direction;
	speed_motor1 = msg->motor_desired_rpm;
	angle_motor1 = msg->motor_desired_angle;
	motor_onoff_motor1 = msg->motor_onoff;
}
void motor_callback2(const robot1::robot1_info::ConstPtr& msg)
{
	motor_direction_motor2 = msg->motor_desired_direction;
	speed_motor2 = msg->motor_desired_rpm;
	angle_motor2 = msg->motor_desired_angle;
	motor_onoff_motor2 = msg->motor_onoff;
}
//test 
void motor_theta_dist_callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  speed_motor1 = msg->data[0];
  speed_motor2 = msg->data[1];

  encoder_pulse_motor1_position = 0;
  encoder_pulse_motor2_position = 0;

  check_position_control1 = false;
  check_position_control2 = false;
}
void algorithm(double angle, double distance)
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
}

//////////////////////////////////////////////////////////////////////////////
void speed_controller(int* encoder_read1,int* encoder_read2, int desired_speed, double* result_rpm ,double* pwm_value)
{
	double static_encoder_pulse = 0;
	double pre_result_rpm = 0;
	double speed_error = 0;
	double control = 0;

	static_encoder_pulse = (*encoder_read1 + *encoder_read2); // 4ch low pass filter
	*encoder_read1 = 0;
    *encoder_read2 = 0;
	*result_rpm =  (((static_encoder_pulse)*60*20)/(399*4));// digital low pass filter

   

	speed_error = desired_speed  -  *result_rpm ;
	control = ( P_gain * speed_error);

    *pwm_value = (*pwm_value + control);

    if (*pwm_value > 512)
		{
			*pwm_value =512;
		}

     if (*pwm_value < 0)
		{
			*pwm_value = 0;
		}
}
int position_control(int* encoder_read_position, int desired_position, int max_out_put, bool* check)
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
}
/////////////////////////////////////////////////////////////////////////////////////
void motor_control(int id, int motor_line1, int motor_line2, bool direction, int desired_speed_rpm, int angle, bool on_off)
{
	if(on_off == true)
	{
		if(desired_speed_rpm == 0 && angle == 0)
		{
			digitalWrite(motor_line1,LOW);
		//	digitalWrite(motor_line2,LOW);
		}
		else
		{
			if(direction == true)//CW
			{
				digitalWrite(motor_line1,HIGH);
			//	digitalWrite(motor_line2,LOW);
			}
			else if (direction == false)//CCW
			{
				digitalWrite(motor_line1,LOW);
			//	digitalWrite(motor_line2,HIGH);
			}
		}

		switch (id)
		{
		case 1 :
			//if(angle != 0)
		//	{
		//		desired_speed_rpm = position_control(&encoder_pulse_motor1_position, angle,  position_max_output_common, &check_position_control1);
		//	}
		speed_controller(&encoder_pulse1_motor1, &encoder_pulse2_motor1, desired_speed_rpm, &result_rpm1,&pwm_value_motor_1);
			break;
		case 2 :
			//if(angle != 0)
			{
				//desired_speed_rpm = position_control(&encoder_pulse_motor2_position, angle,  position_max_output_common, &check_position_control2);
			}
	   speed_controller(&encoder_pulse1_motor2, &encoder_pulse2_motor2, desired_speed_rpm, &result_rpm2, &pwm_value_motor_2);
			break;
		default :
			break;
		}
	}
	if(on_off == false)
	{
	  pwmWrite(motor1_PWM_1, 0);
	  pwmWrite(motor2_PWM_1, 0);
	}
}

void controlFunction(const ros::TimerEvent&)
{
     motor_control(1, motor1_IN1, 0,  motor_direction_motor1, speed_motor1, angle_motor1, motor_onoff_motor1);
	 motor_control(2, motor2_IN1, 0,  motor_direction_motor2, speed_motor2, angle_motor2, motor_onoff_motor2);


     printf("result_pwm_1 :: %f \n", pwm_value_motor_1);
     printf("result_pwm_2 :: %f \n", pwm_value_motor_2);
     pwmWrite(motor1_PWM_1, (int) pwm_value_motor_1);
     pwmWrite(motor2_PWM_1, (int) pwm_value_motor_2);
}
int main (int argc, char **argv)
{ 
	wiringPiSetupGpio();
	wiringPiISR(motor1_FG1, INT_EDGE_BOTH, &motor1_encoder_1);
	wiringPiISR(motor1_FG2, INT_EDGE_BOTH, &motor1_encoder_2);
    wiringPiISR(motor2_FG1, INT_EDGE_BOTH, &motor2_encoder_1);
	wiringPiISR(motor2_FG2, INT_EDGE_BOTH, &motor2_encoder_2);

	initialize();

	ros::init(argc, argv, "motor_node");
	ros::NodeHandle nh;

    ros::Timer timer_control = nh.createTimer(ros::Duration(0.05), controlFunction); // 10ms

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

	pinMode(motor1_PWM_1, PWM_OUTPUT);
	pinMode(motor2_PWM_1, PWM_OUTPUT);
	pwmSetMode (PWM_MODE_MS);
	pwmSetRange(512); 
	pwmSetClock(2);

	digitalWrite(motor1_IN1,HIGH);
	digitalWrite(motor2_IN1,LOW);


	pwmWrite(motor1_PWM_1, 0);
	pwmWrite(motor2_PWM_1, 0);

	while(ros::ok())
	{
     desired_rpm1_msg.data = speed_motor1;
     desired_rpm2_msg.data = speed_motor2;
     result_rpm1_msg.data = result_rpm1;
     result_rpm2_msg.data = result_rpm2;
	 usleep(100);
     result_rpm1_pub.publish(result_rpm1_msg);
     result_rpm2_pub.publish(result_rpm2_msg);
     desired_rpm1_pub.publish(desired_rpm1_msg);
     desired_rpm2_pub.publish(desired_rpm2_msg);
	 ros::spinOnce();
	}

    pwmWrite(motor1_PWM_1, 0);
    pwmWrite(motor2_PWM_1, 0);

	return 0;
}


