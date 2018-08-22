//#define motor2_IN3 5
//#define motor2_IN4  26
//#define motor2_PWM_1 13
//#define motor2_FG1 12

//#define motor1_IN1 27
//#define motor1_IN2  23
//#define motor1_PWM_1 18
//#define motor1_FG1 22

#define motor2_IN1 19

#define motor2_PWM_1 13

#define motor2_FG1 23
#define motor2_FG2 24


#define motor1_IN1 26

#define motor1_PWM_1 12
#define motor1_FG1 22  // 22
#define motor1_FG2 27 //27

bool CW ;
bool CCW;
bool ON ;
bool OFF;

int encoder_pulse1_motor1;
int encoder_pulse2_motor1;

int encoder_pulse1_motor2;
int encoder_pulse2_motor2;

int encoder_pulse_motor1_position;
int encoder_pulse_motor2_position;

double pwm_value_motor_1;
double pwm_value_motor_2;

double P_gain;

bool motor_direction_motor1;
int speed_motor1;
int angle_motor1;
bool check_position_control1;
bool motor_onoff_motor1;
double result_rpm1;
int pre_static_encoder_pulse1;

bool motor_direction_motor2;
int speed_motor2;
int angle_motor2;
bool check_position_control2;
bool motor_onoff_motor2;
double result_rpm2;
int pre_static_encoder_pulse2;

int position_max_output_common;

void initialize();
int position_control(int* encoder_read_position, int desired_position, int max_out_put, bool* check);
void speed_controller(int* encoder_read1,int* encoder_read2, int desired_speed, double* result_rpm ,double* pwm_value);
void motor_control(int id, int motor_line1, int motor_line2, bool direction, int desired_speed_rpm, int angle, bool on_off);

void algorithm(double angle, double distance);
double reference_angle;
double reference_distance;

//timer
void controlFunction(const ros::TimerEvent&);



//ros communication
ros::Publisher  angle_control_done_pub;
ros::Publisher  desired_rpm1_pub;
ros::Publisher  desired_rpm2_pub;

ros::Publisher  result_rpm1_pub;
ros::Publisher  result_rpm2_pub;

ros::Subscriber motor_reference1_sub;
ros::Subscriber motor_reference2_sub;

ros::Subscriber motor_theta_dist_sub;

//message for communication
std_msgs::String angle_control_done_msg;
std_msgs::Float64  desired_rpm1_msg;
std_msgs::Float64  desired_rpm2_msg;

std_msgs::Float64  result_rpm1_msg;
std_msgs::Float64  result_rpm2_msg;



