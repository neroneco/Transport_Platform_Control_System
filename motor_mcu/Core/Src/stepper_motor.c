
// Implementation of functions connected with step motor control,
// like setting: directions, speed; enabling motor, emergancy functions...
// Also functions connected with optical sensor and position control.





#include "stepper_motor.h"
#include <math.h>
#include <stdlib.h>


extern const float K;      // [mm/imp]
extern const int Max_Freq_Div;  // [-] ~200[mm/s]
extern const int Min_Freq_Div;  // [-] ~200[mm/s]
extern const int Max_Freq;  // [Hz] ~200[mm/s]
extern const float Max_V_X;
extern const float Min_V_X;
extern const float Max_V_Y;
extern const float Min_V_Y;
extern const float Max_Position_X;
extern const float Max_Position_Y;

// --------------------- Motor control -----------------------------------

// EN bit: if high motor inactive
//         if low  motor active
void motor_start_stop_x( int toggle ) {
    if (toggle == ENABLED) {
        HAL_GPIO_WritePin(X_EN_GPIO_Port,  X_EN_Pin ,GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(X_EN_GPIO_Port,  X_EN_Pin ,GPIO_PIN_SET);
    }
}


// Set direction based on desired position
int set_motor_direction( int steps_des, int steps_act ) {
    int direction;
    if (steps_des > steps_act) {
        direction = DIR_PLUS;
    } else {
        direction = DIR_MINUS;
    }
    return direction;
}

// EN bit: if high motor inactive
//         if low  motor active
void motor_start_stop_y( int toggle ) {
    if (toggle == ENABLED) {
        HAL_GPIO_WritePin(Y_EN_GPIO_Port,  Y_EN_Pin ,GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(Y_EN_GPIO_Port,  Y_EN_Pin ,GPIO_PIN_SET);
    }
}




// Make step (CHANGES GLOBAL VARIABLES)
extern int x_freq_div;
extern int x_step_zad;
extern int x_step_akt;
extern int x_dir_akt;
extern int x_hold;
void make_step_x( void ) {

    static int iter;
    iter++;
    iter %= x_freq_div;
    if ( !iter ){
        if ( !x_hold ) {

                HAL_GPIO_TogglePin(X_STEP_GPIO_Port, X_STEP_Pin);

                // one step takes two iterations
                static int step_iter;
                step_iter++;
                step_iter %= 2;
                if ( step_iter ) {
                    switch (x_dir_akt){
                        case DIR_PLUS:
                            x_step_akt++;
                            break;
                        case DIR_MINUS:
                            x_step_akt--;
                            break;
                    }
                }
        }
    }
}


extern int y_freq_div;
extern int y_step_zad;
extern int y_step_akt;
extern int y_dir_akt;
extern int y_hold;
void make_step_y( void ) {

    static int iter;
    iter++;
    iter %= y_freq_div;
    if ( !iter ){
        if ( !y_hold ) {

                HAL_GPIO_TogglePin(X_STEP_GPIO_Port, X_STEP_Pin);

                // one step takes two iterations
                static int step_iter;
                step_iter++;
                step_iter %= 2;
                if ( step_iter ) {
                    switch (y_dir_akt){
                        case DIR_PLUS:
                            y_step_akt++;
                            break;
                        case DIR_MINUS:
                            y_step_akt--;
                            break;
                    }
                }
        }
    }
}

extern float x_v_zad;
extern float x_v_akt;
extern float x_a_zad;
extern float x_a_akt;
extern int   x_dir_zad;
int set_v_x( void ) {
    //int x_dist = abs( x_step_zad - x_step_akt );
    // decelerate first if direction changed
    if ( x_dir_akt != x_dir_zad ) {
        // change direction but first decelerate
        if (x_v_akt > 1.0) {
            x_v_akt = x_v_akt - x_a_akt*0.01;
        } else {
            // change direction;
            switch (x_dir_zad) {
                case (DIR_PLUS):
                    HAL_GPIO_WritePin(X_DIR_GPIO_Port,  X_DIR_Pin ,GPIO_PIN_SET);
                    x_dir_akt = DIR_PLUS;
                    break;
                case (DIR_MINUS):
                    HAL_GPIO_WritePin(X_DIR_GPIO_Port,  X_DIR_Pin ,GPIO_PIN_RESET);
                    x_dir_akt = DIR_MINUS;
                    break;
            }
        }
    // now you can accelerate to desired speed
    } else if ( x_v_akt < x_v_zad ) {
        x_v_akt = x_v_akt + x_a_akt*0.01;
        // lets check what is max speed for
        // actual position based on desired position
        float x_v_max = get_max_speed_for_position_x( x_step_akt, x_step_zad );
        if ( x_v_akt > x_v_max ) {
            x_v_akt = x_v_max;
        }
    // if desired speed changed you should decelerate
    } else if ( x_v_akt > x_v_zad ) {
        x_v_akt = x_v_akt - x_a_akt*0.01;
        // lets check what is max speed for
        // actual position based on desired position
        float x_v_max = get_max_speed_for_position_x( x_step_akt, x_step_zad );
        if ( x_v_akt > x_v_max ) {
            x_v_akt = x_v_max;
        }
    }
    return convert_velocity_to_freq_div( x_v_akt );
}

int set_v_x_ALT( void ) {
    int s_decel = convert_position_to_steps( 0.50*((Max_V_X * Max_V_X) / ((float)x_a_akt)) );
    int dist_remain = abs( x_step_zad - x_step_akt );
    x_dir_zad = set_motor_direction( x_step_zad, x_step_akt );

    // decelerate first if direction changed
    if ( x_dir_akt != x_dir_zad ) {
        // change direction but first decelerate
        if (x_v_akt > 4.0) {
            x_v_akt = x_v_akt - x_a_akt*0.01;
        } else {
            // change direction;
            switch (x_dir_zad) {
                case (DIR_PLUS):
                    HAL_GPIO_WritePin(X_DIR_GPIO_Port,  X_DIR_Pin ,GPIO_PIN_SET);
                    x_dir_akt = DIR_PLUS;
                    break;
                case (DIR_MINUS):
                    HAL_GPIO_WritePin(X_DIR_GPIO_Port,  X_DIR_Pin ,GPIO_PIN_RESET);
                    x_dir_akt = DIR_MINUS;
                    break;
            }
        }
    } else {
        float v_lim;
        if ( dist_remain <= s_decel ) {
            v_lim = sqrtf( ((float)dist_remain) / ((float)s_decel) ) * Max_V_X;
        } else if ( dist_remain > s_decel ) {
            v_lim = Max_V_X;
        }
        if ( x_v_akt >= v_lim ) {
            x_v_akt -= x_a_akt*0.01;
        } else if ( x_v_akt < v_lim ) {
            if ( x_v_akt >= x_v_zad ) {
                x_v_akt -= x_a_akt*0.01;
            } else if ( x_v_akt < x_v_zad ) {
                x_v_akt += x_a_akt*0.01;
            }
        }
    }

    // limits
    if (x_v_akt < 0.0){
        x_v_akt = 0.0;
    } else if (x_v_akt > 200.0) {
        x_v_akt = 200.0;
    }
    return convert_velocity_to_freq_div( x_v_akt );
}


extern float y_v_zad;
extern float y_v_akt;
extern float y_a_zad;
extern float y_a_akt;
extern int   y_dir_zad;
int set_v_y( void ) {
    // decelerate first if direction changed
    if ( y_dir_akt != y_dir_zad ) {
        // change direction but first decelerate
        if (y_v_akt > 5.0) {
            y_v_akt = y_v_akt - y_a_akt*0.01;
        } else {
            // change direction;
            switch (y_dir_zad) {
                case (DIR_PLUS):
                    HAL_GPIO_WritePin(Y_DIR_GPIO_Port,  Y_DIR_Pin ,GPIO_PIN_SET);
                    y_dir_akt = DIR_PLUS;
                    break;
                case (DIR_MINUS):
                    HAL_GPIO_WritePin(Y_DIR_GPIO_Port,  Y_DIR_Pin ,GPIO_PIN_RESET);
                    y_dir_akt = DIR_MINUS;
                    break;
            }
        }
    // now you can accelerate to desired speed
    } else if ( y_v_akt < y_v_zad ) {
        y_v_akt = y_v_akt + y_a_akt*0.01;
        // lets check what is max speed for
        // actual position based on desired position
        float y_v_max = get_max_speed_for_position_x( y_step_akt, y_step_zad);
        if ( y_v_akt > y_v_max ) {
            y_v_akt = y_v_max;
        }
    // if desired speed changed you should decelerate
    } else if ( y_v_akt > y_v_zad ) {
        y_v_akt = y_v_akt - y_a_akt*0.01;
        // lets check what is max speed for
        // actual position based on desired position
        float y_v_max = get_max_speed_for_position_x( y_step_akt, y_step_zad );
        if ( y_v_akt > y_v_max ) {
            y_v_akt = y_v_max;
        }
    }
    return convert_velocity_to_freq_div( y_v_akt );
}

// -----------------------------------------------------------------------





// --------------------- Speed control -----------------------------------
// TODO: change so that the acceleration/deceleration can be influenced
float get_max_speed_for_position_x( int steps_actual, int steps_desired ) {
    int dif = abs( steps_actual - steps_desired );
    if ( dif > 2546 ) {
        return Max_V_X;
    }
    return ((200.0/2546.0)*((float)dif));
}

float get_max_speed_for_position_y( int steps_actual, int steps_desired ) {
    int dif = abs( steps_actual - steps_desired );
    if ( dif > 2546 ) {
        return Max_V_Y;
    }
    return ((195.0/2546.0)*((float)dif) + 4.0);
}

int check_max_speed_x( int steps_actual, int freq_div, int dir) {
    int max_freq_div = steps_to_max_freq_div_x(steps_actual, dir);
    if ( freq_div > max_freq_div ) {
        return freq_div;
    } else {
        return max_freq_div;
    }
}


// for 4278 steps I want to have speed of ~   1[mm/s] -> freq_div = 1000
// for 2546 steps I want to have speed of ~ 200[mm/s] -> freq_div =    5
int steps_to_max_freq_div_x( int steps_pos, int dir ) {
    // if pos in range of <-100[mm], 100[mm]> ~~~ <-2546[steps], 2546[steps]>
    if ( (steps_pos > -2546) && (steps_pos < 2546) ){
        return Max_Freq_Div;
    } else if ( (steps_pos > 0) && (steps_pos < 4278) && (dir == DIR_MINUS) ) {
        return Max_Freq_Div;
    } else if ( (steps_pos < 0) && (steps_pos > -4278) && (dir == DIR_PLUS) ) {
        return Max_Freq_Div;
    } else {
        // 4278[steps] ~ 168[mm]
        return abs(((int)( (995.0 / 1732.0) * (float)(abs(steps_pos) - 2546))) + Max_Freq_Div);
    }
}


int check_max_speed_y( int steps_actual, int freq_div, int dir) {
    int max_freq_div = steps_to_max_freq_div_y(steps_actual, dir);
    if ( freq_div > max_freq_div ) {
        return freq_div;
    } else {
        return max_freq_div;
    }
}


// for 1910 steps I want to have speed of ~   1[mm/s] -> freq_div = 1000
// for 0 steps I want to have speed of ~ 200[mm/s] -> freq_div =    5
int steps_to_max_freq_div_y( int steps_pos, int dir ) {

    if ( (steps_pos > 0) && (steps_pos < 1910) && (dir == DIR_MINUS) ) {
        return Max_Freq_Div;
    } else if ( (steps_pos < 0) && (steps_pos > -1910) && (dir == DIR_PLUS) ) {
        return Max_Freq_Div;
    } else {
        // 1910[steps] ~ 75[mm]
        return abs(((int)( (995.0 / 1910.0) * (float)(abs(steps_pos)))) + Max_Freq_Div);
    }
}

// -----------------------------------------------------------------------





// --------------------- Conversion --------------------------------------

// converts position in (float)[mm]   to position in steps (int)[steps]
int convert_position_to_steps( float position ) {
    return (int)(position / K);
}


// converts velocity in (float)[mm/s] to velocity in steps (int)[steps/s]
int convert_velocity_to_freq_div( float velocity ) {
    if ( (velocity > 5.0) || (velocity < 200.0) ) {
        float freq = (velocity / K);
        return (Max_Freq / freq);
    } else if (velocity > 200.0) {
        return Max_Freq_Div;
    } else {
        return Min_Freq_Div;
    }
}


// converts position in steps (int)[steps]   to position in (float)[mm]
float convert_steps_to_position( int steps_actual ) {
    return (K * ((float)steps_actual));
}


// converts velocity in steps (int)[steps/s] to velocity in (float)[mm/s]
float convert_freq_div_to_velocity( int freq_div ) {
    if (freq_div > 4) {
        float freq = (float)(Max_Freq / freq_div);
        return (K * freq);
    }
    return 200.0;
}

// -----------------------------------------------------------------------





// ---------------- Optical sensor position control ----------------------

extern TIM_HandleTypeDef htim6;
extern int x_en;
extern int y_en;
void ES_STOP() {
    HAL_TIM_Base_Stop_IT(&htim6);
    HAL_GPIO_WritePin(X_EN_GPIO_Port, X_EN_Pin ,GPIO_PIN_SET);
    HAL_GPIO_WritePin(Y_EN_GPIO_Port, Y_EN_Pin ,GPIO_PIN_SET);
    while(1){
        x_en = DISABLED;
        y_en = DISABLED;
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        HAL_Delay(1000);
    };
}


// converts ADC output value in [bits] to position value in [mm]
//    For X axis:
float adc_get_pos_x( uint32_t adc_output) {
    // convert [bits] to [volts]: (3.3[V]) / (4096[bits])* ADC_VALUE; (ADC is 12-bit)
    static float volt_dist;
    volt_dist = 0.0008056640625f * (float)adc_output;

    // Emergancy check
    if ( (volt_dist > 2.07) || (volt_dist < 0.54) ) {
        ES_STOP();
    }

    // simple digital IIR LP filter (recursive average): f_cutoff ~ 4[Hz]
    static float alpha = 0.10;
    static float volt_filtered;
    volt_filtered = ((1.0-alpha) * volt_filtered) + (alpha * volt_dist);
    // convert [volts] to [mm]; (exponential approximation)
    static float a = -315.9;
    static float b = -0.8679;
    static float c =  352.7 ;
    float pos_approx = a * pow( volt_filtered, b ) + c;


    //return volt_dist;
    return pos_approx;
}


//    For Y axis:
float adc_get_pos_y( uint32_t adc_output) {
    // convert [bits] to [volts]: (3.3[V]) / (4096[bits])* ADC_VALUE; (ADC is 12-bit)
    static float volt_dist;
    volt_dist = 0.0008056640625f * (float)adc_output;

    // Emergency check
    if ( (volt_dist > 2.07) || (volt_dist < 0.89) ) {
        ES_STOP();
    }

    // simple digital IIR LP filter (recursive average): f_cutoff ~ 4[Hz]
    static float alpha = 0.10;
    static float volt_filtered;
    volt_filtered = ((1.0-alpha) * volt_filtered) + (alpha * volt_dist);

    // convert [volts] to [mm]; (exponential approximation)
    static float a = -260.9;
    static float b = -1.094;
    static float c =  202.6;
    float pos_approx = a * pow( volt_filtered, b ) + c;

    //return volt_filtered;
    return pos_approx;
}


// checks if actual position is allowed
int check_danger_zone( float position, float max_position ) {
    if ( (position > max_position) || (position < -max_position) )
        return 1;
    else
        return 0;
}

// -----------------------------------------------------------------------




