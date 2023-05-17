#include "accelerometers.h"


//-------------------------------------------------
//    accelerometers init and data acquisition
//-------------------------------------------------

void mpu9250_Init( I2C_HandleTypeDef* h_i2c )
{
//    I2C_WriteReg( h_i2c, ACCEL_ADDRESS, PWR_MGMT_1      , 0x00 );
//    I2C_WriteReg( h_i2c, ACCEL_ADDRESS, SMPLRT_DIV      , 0x00 );
//    I2C_WriteReg( h_i2c, ACCEL_ADDRESS, CONFIG          , 0x02 );
//    I2C_WriteReg( h_i2c, ACCEL_ADDRESS, GYRO_CONFIG     , 0x00 );
//    I2C_WriteReg( h_i2c, ACCEL_ADDRESS, ACCEL_CONFIG_1  , 0x00 );
//    I2C_WriteReg( h_i2c, ACCEL_ADDRESS, ACCEL_CONFIG_2  , 0x02 );
    I2C_WriteReg( h_i2c, ACCEL_ADDRESS, PWR_MGMT_1      , 0x00 );
    I2C_WriteReg( h_i2c, ACCEL_ADDRESS, SMPLRT_DIV      , 0x00 );
    I2C_WriteReg( h_i2c, ACCEL_ADDRESS, CONFIG          , 0x00 );
    I2C_WriteReg( h_i2c, ACCEL_ADDRESS, GYRO_CONFIG     , 0x01 );
    I2C_WriteReg( h_i2c, ACCEL_ADDRESS, ACCEL_CONFIG_1  , 0x00 );
    I2C_WriteReg( h_i2c, ACCEL_ADDRESS, ACCEL_CONFIG_2  , 0x01 );
    HAL_Delay(10);
}

void mpu6886_Init( I2C_HandleTypeDef* h_i2c )
{
//    I2C_WriteReg( h_i2c, ACCEL_ADDRESS, PWR_MGMT_1      , 0x00 );
//    I2C_WriteReg( h_i2c, ACCEL_ADDRESS, SMPLRT_DIV      , 0x00 );
//    I2C_WriteReg( h_i2c, ACCEL_ADDRESS, CONFIG          , 0x02 );
//    I2C_WriteReg( h_i2c, ACCEL_ADDRESS, GYRO_CONFIG     , 0x00 );
//    I2C_WriteReg( h_i2c, ACCEL_ADDRESS, ACCEL_CONFIG_1  , 0x00 );
//    I2C_WriteReg( h_i2c, ACCEL_ADDRESS, ACCEL_CONFIG_2  , 0x02 );
    I2C_WriteReg( h_i2c, ACCEL_ADDRESS, PWR_MGMT_1      , 0x00 );
    I2C_WriteReg( h_i2c, ACCEL_ADDRESS, SMPLRT_DIV      , 0x00 );
    I2C_WriteReg( h_i2c, ACCEL_ADDRESS, CONFIG          , 0x00 );
    I2C_WriteReg( h_i2c, ACCEL_ADDRESS, GYRO_CONFIG     , 0x01 );
    I2C_WriteReg( h_i2c, ACCEL_ADDRESS, ACCEL_CONFIG_1  , 0x00 );
    I2C_WriteReg( h_i2c, ACCEL_ADDRESS, ACCEL_CONFIG_2  , 0x01 );
    HAL_Delay(10);
}

void read_accel( I2C_HandleTypeDef* h_i2c, mpu_data_struct* mpu )
{
    static uint8_t buf[16] = {0};

    I2C_ReadBytes( h_i2c, ACCEL_ADDRESS, ACCEL_XOUT_H, buf, 6);

    // TODO change to bite shift buf>>14 instead of division by 16384
    mpu->acc.x = ((float)((int16_t)((buf[0]<<8) | buf[1]))) / (16384);
    mpu->acc.y = ((float)((int16_t)((buf[2]<<8) | buf[3]))) / (16384);
    mpu->acc.z = ((float)((int16_t)((buf[4]<<8) | buf[5]))) / (16384);
}

void read_gyro( I2C_HandleTypeDef* h_i2c, mpu_data_struct* mpu )
{
    static uint8_t buf[16] = {0};

    I2C_ReadBytes( h_i2c, ACCEL_ADDRESS, GYRO_XOUT_H, buf, 6);

    mpu->gyro.x = ((float)((int16_t)((buf[0]<<8) | buf[1]))) / (131.0f);
    mpu->gyro.y = ((float)((int16_t)((buf[2]<<8) | buf[3]))) / (131.0f);
    mpu->gyro.z = ((float)((int16_t)((buf[4]<<8) | buf[5]))) / (131.0f);
}

void read_sensors_mpu9250( I2C_HandleTypeDef* h_i2c, mpu_data_struct* mpu )
{
    static uint8_t buf[16] = {0};

    I2C_ReadBytes( h_i2c, ACCEL_ADDRESS, ACCEL_XOUT_H, buf, 14);

    // TODO change to bite shift buf>>14 instead of division by 16384
    mpu->acc.x = ((float)((int16_t)((buf[0]<<8) | buf[1]))) / (16384);
    mpu->acc.y = ((float)((int16_t)((buf[2]<<8) | buf[3]))) / (16384);
    mpu->acc.z = ((float)((int16_t)((buf[4]<<8) | buf[5]))) / (16384);

    // TODO might be  useful but for now it is not
    //tempr = (buf[6]<<8) | buf[7];

    mpu->gyro.x = ((float)((int16_t)((buf[8]<<8)  | buf[9] ))) / (131.0f);
    mpu->gyro.y = ((float)((int16_t)((buf[10]<<8) | buf[11]))) / (131.0f);
    mpu->gyro.z = ((float)((int16_t)((buf[12]<<8) | buf[13]))) / (131.0f);
}

void read_sensors_mpu6886( I2C_HandleTypeDef* h_i2c, mpu_data_struct* mpu )
{
    static uint8_t buf[16] = {0};

    I2C_ReadBytes( h_i2c, ACCEL_ADDRESS, ACCEL_XOUT_H, buf, 14);

    // TODO change to bite shift buf>>14 instead of division by 16384
    mpu->acc.z = ((float)((int16_t)((buf[0]<<8) | buf[1]))) / (16384);
    mpu->acc.x = ((float)((int16_t)((buf[2]<<8) | buf[3]))) / (16384);
    mpu->acc.y = ((float)((int16_t)((buf[4]<<8) | buf[5]))) / (16384);

    // TODO might be  useful but for now it is not
    //tempr = (buf[6]<<8) | buf[7];

    mpu->gyro.z = ((float)((int16_t)((buf[8]<<8)  | buf[9] ))) / (131.0f);
    mpu->gyro.x = ((float)((int16_t)((buf[10]<<8) | buf[11]))) / (131.0f);
    mpu->gyro.y = ((float)((int16_t)((buf[12]<<8) | buf[13]))) / (131.0f);
}
