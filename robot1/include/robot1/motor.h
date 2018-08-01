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

bool CW  = true;
bool CCW = false;
bool ON  = true;
bool OFF = false;



int encoder_pulse_motor1 = 0;
int encoder_pulse_motor2 = 0;

int encoder_pulse_motor1_position = 0;
int encoder_pulse_motor2_position = 0;

bool check_position_control = false;

int pwm_value_motor_1 = 0;
int pwm_value_motor_2 = 0;

float P_gain =  0.5; //20

int position_control(int* encoder_read_position, int* desired_position);
void speed_controller(int* encoder_read, int desired_speed, int* pwm_value);

void motor_control(int id, int motor_line1, int motor_line2, bool direction, int desired_speed_rpm, int angle, bool on_off);


bool motor_direction_motor1 = true;
int speed_motor1 = 0;
int angle_motor1 = 0;
bool motor_onoff_motor1 = false;

bool motor_direction_motor2 = true;
int speed_motor2 = 0;
int angle_motor2 = 0;
bool motor_onoff_motor2 = false;


