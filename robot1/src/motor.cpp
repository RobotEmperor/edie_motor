#include <ros/ros.h>
#include <stdio.h>
#include <wiringPi.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <robot1/motor.h>
#include <robot1/robot1_info.h>

//////////////////////////////////////////////////////////////////////////////
void myinterrupt_1(void)
{
	encoder_pulse_motor1 ++;
	encoder_pulse_motor1_position ++;
}
void myinterrupt_2(void)
{
	encoder_pulse_motor2 ++;
	encoder_pulse_motor2_position ++;
}
void motor_callback1(const robot1::robot1_info::ConstPtr& msg)
{
	motor_direction_motor1 = msg->motor_desired_direction;
	speed_motor1 = msg->motor_desired_rpm;
	angle_motor1 = msg->motor_desired_angle;
	motor_onoff_motor1 = msg->motor_onoff;

	check_position_control = false;
}
void motor_callback2(const robot1::robot1_info::ConstPtr& msg)
{
	motor_direction_motor2 = msg->motor_desired_direction;
	speed_motor2 = msg->motor_desired_rpm;
	angle_motor2 = msg->motor_desired_angle;
	motor_onoff_motor2 = msg->motor_onoff;

	check_position_control = false;
}
void initialize(int argc, char **argv)
{
	CW  = true;
	CCW = false;
	ON  = true;
	OFF = false;

	encoder_pulse_motor1 = 0;
	encoder_pulse_motor2 = 0;

	encoder_pulse_motor1_position = 0;
	encoder_pulse_motor2_position = 0;

	check_position_control = false;

	pwm_value_motor_1 = 0;
	pwm_value_motor_2 = 0;

	P_gain =  0.5; // speed controller p gain


	motor_direction_motor1 = true;
	speed_motor1 = 0;
	angle_motor1 = 0;
	motor_onoff_motor1 = false;

	motor_direction_motor2 = true;
	speed_motor2 = 0;
	angle_motor2 = 0;
	motor_onoff_motor2 = false;

	ros::init(argc, argv, "motor_node");
	ros::NodeHandle nh;

	motor_reference1_sub = nh.subscribe("motor_reference1", 1, motor_callback1);
	motor_reference2_sub = nh.subscribe("motor_reference2", 1, motor_callback2);
	angle_control_done_pub = nh.advertise<std_msgs::String>("/angle_control_done",10);
	angle_control_done_msg.data.clear();
}
//////////////////////////////////////////////////////////////////////////////
void speed_controller(int* encoder_read, int desired_speed, int* pwm_value)
{
	int static_encoder_pulse = 0;
	int result_rpm  = 0;
	int speed_error = 0;
	int control = 0;

	static_encoder_pulse = *encoder_read;
	*encoder_read = 0;
	result_rpm = (int) (static_encoder_pulse*60*1000)/(374*8);// pulse 11 gear ratio 34

	speed_error = desired_speed  - result_rpm ;

	// printf("speed_error = %d \n", speed_error);
	//printf("pwm_value_motor_2 = %d \n", pwm_value_motor_2);


	control = (int)( P_gain * (float) speed_error);



	if(control > 0)
	{
		if(control > 2)
		{
			control = 2;
		}

		*pwm_value = (*pwm_value + control);

		if (*pwm_value > 255)
		{
			*pwm_value = 255;
		}

	}
	else if(control < 0)
	{

		if(control < - 2)
		{
			control = - 2;
		}

		*pwm_value =  (*pwm_value +  control);

		if (*pwm_value < 0)
		{
			*pwm_value = 0;
		}

	}
}
int position_control(int* encoder_read_position, int* desired_position)
{
	int static_encoder_pulse = 0;
	int result_position  = 0;
	int position_error = 0;
	int control = 0;
	float P_gain_position = 0.5;
	static int speed_output = 0;

	static_encoder_pulse = *encoder_read_position;
	result_position = (int) (static_encoder_pulse*(374/360));// pulse 11 gear ratio 34

	if(check_position_control == true)
	{
		return 0;
	}
	if(*desired_position <= result_position)
	{
		printf("result_position1111 = %d \n ", result_position);
		check_position_control = true;
		static_encoder_pulse = 0;
		speed_output = 0;
		*encoder_read_position = 0;
		*desired_position = 0; // stop after 1 rotation
		angle_control_done_msg.data = "done";
		angle_control_done_pub.publish(angle_control_done_msg);
		return 0; // 0
	}

	position_error = *desired_position  - result_position;


	control = (int)( P_gain_position * (float) position_error);

	if(control > 0)
	{
		if(control > 2)
		{
			control = 2;
		}

		speed_output = (speed_output + control);

		if (speed_output > 20)
		{
			speed_output = 20;
		}

	}
	else if(control < 0)
	{

		if(control < - 2)
		{
			control = - 2;
		}

		speed_output =  (speed_output +  control);

		if (speed_output < 20)
		{
			speed_output = 20;
		}

	}

	return speed_output;
}
/////////////////////////////////////////////////////////////////////////////////////
void motor_control(int id, int motor_line1, int motor_line2, bool direction, int desired_speed_rpm, int* angle, bool on_off)
{

	if(on_off == true)
	{
		if(desired_speed_rpm == 0 && angle == 0)
		{
			digitalWrite(motor_line1,LOW);
			digitalWrite(motor_line2,LOW);
			pwm_value_motor_1 = 0;
			pwm_value_motor_2 = 0;
		}
		else
		{
			if(direction == true)//CW
			{
				digitalWrite(motor_line1,HIGH);
				digitalWrite(motor_line2,LOW);
			}
			else if (direction == false)//CCW
			{
				digitalWrite(motor_line1,LOW);
				digitalWrite(motor_line2,HIGH);
			}
		}

		switch (id)
		{
		case 1 :
			if(angle != 0)
			{
				desired_speed_rpm = position_control(&encoder_pulse_motor1_position, angle);
			}
			speed_controller(&encoder_pulse_motor1, desired_speed_rpm, &pwm_value_motor_1);
			break;
		case 2 :
			if(angle != 0)
			{
				desired_speed_rpm = position_control(&encoder_pulse_motor1_position, angle);
			}
			speed_controller(&encoder_pulse_motor2, desired_speed_rpm, &pwm_value_motor_2);
			break;
		default :
			break;
		}
	}


	if(on_off == false)
	{

		digitalWrite(motor_line1,LOW);
		digitalWrite(motor_line2,LOW);
	}

}
int main (int argc, char **argv)
{ 
	wiringPiSetupGpio();
	wiringPiISR(motor1_FG1, INT_EDGE_RISING, &myinterrupt_1);
	wiringPiISR(motor2_FG1, INT_EDGE_RISING, &myinterrupt_2);

	initialize(argc, argv);

	pinMode(motor1_IN1, OUTPUT);
	pinMode(motor1_IN2, OUTPUT);
	pinMode(motor1_FG1, INPUT);
	pinMode(motor2_IN3, OUTPUT);
	pinMode(motor2_IN4, OUTPUT);
	pinMode(motor2_FG1, INPUT);

	pinMode(motor1_PWM_1, PWM_OUTPUT);
	pinMode(motor2_PWM_1, PWM_OUTPUT);
	pwmSetMode (PWM_MODE_MS);
	pwmSetRange(255); //
	pwmSetClock(2);

	digitalWrite(motor1_IN2,LOW);
	digitalWrite(motor1_IN1,LOW);
	digitalWrite(motor2_IN4,LOW);
	digitalWrite(motor2_IN3,LOW);

	pwmWrite(motor1_PWM_1, 0);
	pwmWrite(motor2_PWM_1, 0);

	while(ros::ok())
	{
		usleep(8000);

		motor_control(1, motor1_IN1, motor1_IN2,  motor_direction_motor1, speed_motor1, &angle_motor1, motor_onoff_motor1);
		motor_control(2, motor2_IN3, motor2_IN4,  motor_direction_motor2, speed_motor2, &angle_motor2, motor_onoff_motor2);

		pwmWrite(motor1_PWM_1, pwm_value_motor_1);
		pwmWrite(motor2_PWM_1, pwm_value_motor_2);

		ros::spinOnce();
	}

	digitalWrite(motor1_IN2,LOW);
	digitalWrite(motor1_IN1,LOW);
	digitalWrite(motor2_IN4,LOW);
	digitalWrite(motor2_IN3,LOW);
	return 0;
}


