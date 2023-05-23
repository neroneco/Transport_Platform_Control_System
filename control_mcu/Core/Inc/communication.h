#ifndef INC_communication_H_
#define INC_communication_H_

#include "stm32f7xx_hal.h"
#include <stdio.h>
#include <string.h>

//-------------------------------------------------
//      communication structures/enums/defines
//-------------------------------------------------

typedef struct {
    uint8_t sampling_rate;
    uint8_t scale_accel;
    uint8_t scale_gyro;
    uint8_t dlpf_accel;
    uint8_t dlpf_gyro;
} mpu_status;

typedef struct {
    uint8_t sampling_rate;
    uint8_t filter_type;
} imu_status;

typedef struct {
    float cart_x_mass;
    float cart_y_mass;
    int	  steps;
    float max_x_position;
    float max_y_position;
    float max_speed;
    float max_accel;
} carts_status;

typedef struct {
    mpu_status      mpu9250;
    mpu_status      mpu6886;
    imu_status      imu;
    carts_status    carts;
} system_status_struct;

typedef struct {
    system_status_struct sytem_status;
    float carts_pos_adc_x[100];
    float carts_pos_x[100];
    float carts_pos_adc_y[100];
    float carts_pos_y[100];
    float carts_vel_x[100];
    float carts_vel_y[100];
    float carts_acc_x[100];
    float carts_acc_y[100];
#if 1
    float mpu9250_acce_x[100];
    float mpu9250_acce_y[100];
    float mpu9250_acce_z[100];
    float mpu9250_gyro_x[100];
    float mpu9250_gyro_y[100];
    float mpu9250_gyro_z[100];
    float mpu9250_pitch[100];
    float mpu9250_roll[100];

    float mpu6886_acce_x[100];
    float mpu6886_acce_y[100];
    float mpu6886_acce_z[100];
    float mpu6886_gyro_x[100];
    float mpu6886_gyro_y[100];
    float mpu6886_gyro_z[100];
    float mpu6886_pitch[100];
    float mpu6886_roll[100];

#endif
    float pitch_no_filter[100];
    float roll_no_filter[100];
    float pitch_complementary[100];
    float roll_complementary[100];
    float pitch_alfa_beta[100];
    float roll_alfa_beta[100];
    float pitch_kalman[100];
    float roll_kalman[100];
    float pitch[100];
    float roll[100];
} data_packet_struct;

enum PACKET {
    PACKET_ONE,
    PACKET_TWO,
    PACKET_NONE,
};


//-------------------------------------------------
//         I2C communication functions
//-------------------------------------------------

void I2C_ReadBytes( I2C_HandleTypeDef* h_i2c, uint8_t dev_addr, uint8_t reg_addr, uint8_t* buf_out, uint8_t byte_num );
void I2C_WriteBytes(I2C_HandleTypeDef* h_i2c, uint8_t dev_addr, uint8_t* buf_out,  uint8_t byte_num );

uint8_t I2C_ReadReg( I2C_HandleTypeDef* h_i2c, uint8_t dev_addr, uint8_t reg);
void    I2C_WriteReg(I2C_HandleTypeDef* h_i2c, uint8_t dev_addr, uint8_t reg, uint8_t value);

#endif /* INC_communication_H_ */
