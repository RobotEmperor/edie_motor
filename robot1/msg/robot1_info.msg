int32 data1

int8 motor_id
bool  motor_desired_direction 
int32 motor_desired_rpm
int32 motor_desired_angle
bool motor_onoff

# sensor_data 
int32 ir_data_ps
int32 ir_data_als
int32 ir_data_white

float64 acc_data_x
float64 acc_data_y
float64 acc_data_z

float32 sonic_data 
float32 touch_data 


#output_data 
int8 speaker_mode 
int8 dot_matrix_mode

# play sound
uint8 play_sound

#Dlink data
string[] essid
int32[] dbm
