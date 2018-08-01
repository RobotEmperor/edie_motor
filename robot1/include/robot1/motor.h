//#define motor2_CCW_1 6
#define motor2_IN3 5
#define motor2_IN4  26
//#define motor2_CW_2  19
#define motor2_PWM_1 13
#define motor2_FG1 12

#define motor1_IN1 27
#define motor1_IN2  23
#define motor1_PWM_1 18

#define motor1_FG1 22

bool CW ;
bool CCW;
bool ON ;
bool OFF;

int encoder_pulse_motor1;
int encoder_pulse_motor2;

int encoder_pulse_motor1_position;
int encoder_pulse_motor2_position;

bool check_position_control;

int pwm_value_motor_1;
int pwm_value_motor_2;

float P_gain =  0.5; //20

bool motor_direction_motor1;
int speed_motor1;
int angle_motor1;
bool motor_onoff_motor1;

bool motor_direction_motor2;
int speed_motor2;
int angle_motor2;
bool motor_onoff_motor2;

void initialize(int argc, char **argv);
int position_control(int* encoder_read_position, int* desired_position);
void speed_controller(int* encoder_read, int desired_speed, int* pwm_value);
void motor_control(int id, int motor_line1, int motor_line2, bool direction, int desired_speed_rpm, int angle, bool on_off);


//ros communication
ros::Publisher  angle_control_done_pub;
ros::Subscriber motor_reference1_sub;
ros::Subscriber motor_reference2_sub;

//message for communication
std_msgs::String angle_control_done_msg;



