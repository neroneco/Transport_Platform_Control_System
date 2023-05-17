/*
 * MPU9250.h
 *
 *  Created on: Feb 14, 2023
 *      Author: xyz
 */

#ifndef INC_accelerometers_H_
#define INC_accelerometers_H_

#include "stm32f7xx_hal.h"


//-------------------------------------------------
// definition of  MPU9250/MPU6886 register address
//-------------------------------------------------
#define SMPLRT_DIV      0x19    //Sample Rate Divider. Typical values:0x07(125Hz) 1KHz internal sample rate
#define CONFIG          0x1A    //Low Pass Filter.Typical values:0x06(5Hz)
#define GYRO_CONFIG     0x1B    //Gyro Full Scale Select. Typical values:0x10(1000dps)
#define ACCEL_CONFIG_1  0x1C    //Accel Full Scale Select. Typical values:0x01(2g)
#define ACCEL_CONFIG_2  0x1D

#define ACCEL_XOUT_H    0x3B
#define ACCEL_XOUT_L    0x3C
#define ACCEL_YOUT_H    0x3D
#define ACCEL_YOUT_L    0x3E
#define ACCEL_ZOUT_H    0x3F
#define ACCEL_ZOUT_L    0x40

#define TEMP_OUT_H      0x41
#define TEMP_OUT_L      0x42

#define GYRO_XOUT_H     0x43
#define GYRO_XOUT_L     0x44
#define GYRO_YOUT_H     0x45
#define GYRO_YOUT_L     0x46
#define GYRO_ZOUT_H     0x47
#define GYRO_ZOUT_L     0x48


#define MAG_XOUT_L      0x03
#define MAG_XOUT_H      0x04
#define MAG_YOUT_L      0x05
#define MAG_YOUT_H      0x06
#define MAG_ZOUT_L      0x07
#define MAG_ZOUT_H      0x08


#define	PWR_MGMT_1      0x6B    //Power Management. Typical values:0x00(run mode)
#define	WHO_AM_I        0x75    //identity of the device


#define	GYRO_ADDRESS   0xD0	  //Gyro and Accel device address
#define MAG_ADDRESS    0x18   //compass device address
#define ACCEL_ADDRESS  0xD0

#define ADDRESS_AD0_LOW     0xD0 //address pin low (GND), default for InvenSense evaluation board
#define ADDRESS_AD0_HIGH    0xD1 //address pin high (VCC)
#define DEFAULT_ADDRESS     GYRO_ADDRESS
#define WHO_AM_I_VAL        0x73 //identity of MPU9250 is 0x71. identity of MPU9255 is 0x73.



//-------------------------------------------------
//     accelerometers data structers / enumes
//-------------------------------------------------

enum MPU_SAMPLING_RATE {
    RATE_1kHZ,
    RATE_4kHZ,
    RATE_8kHZ,
    RATE_32kHZ
};

enum MPU_ACCEL_SCALE {
    SCALE_2g,
    SCALE_4g,
    SCALE_8g,
    SCALE_16g
};

enum MPU_GYRO_SCALE {
    SCALE_250dps,
    SCALE_500dps,
    SCALE_1000dps,
    SCALE_2000dps
};

enum MPU_DLPF {
    MPU_DLPF_NONE,
    MPU_DLPF_5HZ,
    MPU_DLPF_5_1HZ,
    MPU_DLPF_10HZ,
    MPU_DLPF_10_2HZ,
    MPU_DLPF_20HZ,
    MPU_DLPF_21_2HZ,
    MPU_DLPF_41HZ,
    MPU_DLPF_44_8HZ,
    MPU_DLPF_92HZ,
    MPU_DLPF_99HZ,
    MPU_DLPF_176HZ,
    MPU_DLPF_184HZ,
    MPU_DLPF_218_1HZ,
    MPU_DLPF_250HZ,
    MPU_DLPF_420HZ,
    MPU_DLPF_460HZ,
    MPU_DLPF_1046HZ,
    MPU_DLPF_1130HZ,
    MPU_DLPF_3281HZ,
    MPU_DLPF_3600HZ,
    MPU_DLPF_8173HZ,
    MPU_DLPF_8800HZ
};

enum IMU_SAMPLING_RATE {
    RATE_100HZ,
    RATE_200HZ,
    RATE_250HZ,
    RATE_400HZ,
    RATE_500HZ,
};

enum STEP_MOTOR {
    STEP_16,
    STEP_8,
    STEP_4,
    STEP_2,
    STEP_FULL
};



typedef struct {
    float x;
    float y;
    float z;
} axis_data_struct;

typedef struct {
    float pitch;
    float roll;
} angles_data_struct;

typedef struct {
    axis_data_struct 	acc ;
    axis_data_struct 	gyro;
    angles_data_struct  angle;
} mpu_data_struct;

void mpu9250_Init( I2C_HandleTypeDef* h_i2c );
void mpu6886_Init( I2C_HandleTypeDef* h_i2c );

void read_accel(   I2C_HandleTypeDef* h_i2c, mpu_data_struct* mpu );
void read_gyro(    I2C_HandleTypeDef* h_i2c, mpu_data_struct* mpu );
void read_sensors_mpu9250( I2C_HandleTypeDef* h_i2c, mpu_data_struct* mpu );
void read_sensors_mpu6886( I2C_HandleTypeDef* h_i2c, mpu_data_struct* mpu );

#endif /* INC_accelerometers_H_ */
