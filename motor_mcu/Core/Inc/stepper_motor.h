
#ifndef INC_STEPPER_MOTOR_H_
#define INC_STEPPER_MOTOR_H_

#include "stm32l4xx_hal.h"

enum STEPPER_DIR{
    DIR_PLUS = 1,
    DIR_MINUS = -1
};

enum STEPPER_STATUS{
    ENABLED,
    DISABLED
};

const float K = 0.0393;      // [mm/imp]
const int Max_Freq_Div  = 5;  // ~200[mm/s]
const int Max_Freq      = 25000;  // ~200[mm/s]
const float Max_Position_X = 175.0;
const float Max_Position_Y =  75.0;

// Motor setup
void motor_start_stop_x( int toggle );
void motor_start_stop_y( int toggle );
int  set_motor_direction_x( int steps_des, int steps_act );
int  set_motor_direction_y( int steps_des, int steps_act );
void make_step_x( void );
void make_step_y( void );

// Speed control
int check_max_speed( int steps_actual, int freq_div );
int steps_to_max_freq_div( int steps_pos );

// Conversion
int convert_position_to_steps( float position );
int convert_velocity_to_freq_div( float velocity );
float convert_steps_to_position( int steps_actual );
float convert_freq_div_to_velocity( int freq_div );

// Optical sensor position control
void ES_STOP();
float adc_get_pos_x( uint32_t adc_output);
float adc_get_pos_y( uint32_t adc_output);
int check_danger_zone( float position, float max_position );

#endif /* INC_STEPPER_MOTOR_H_ */
