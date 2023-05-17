
#include "control.h"

extern volatile data_packet_struct Data_Packet[2];
extern volatile int Previous;

void controler( int regulator_type, 
                int filter_type )
{

    static carts_data_struct carts;
    static mpu_data_struct mpu9250, mpu6886;
    static imu_data_struct imu;

    carts_get_data( &carts );
    imu_get_pitch_roll( filter_type, &mpu9250, &mpu6886, &imu );

    regulate( regulator_type, &imu, &carts );

    send_control_signal( &carts );

    store_data( &carts ,&mpu9250, &mpu6886, &imu );

}

void carts_get_data( carts_data_struct* carts_data )
{
    spi_read( carts_data );
}


void send_control_signal( carts_data_struct* carts_data )
{
    spi_send( carts_data );
}

void store_data( carts_data_struct* carts_data,
                 mpu_data_struct*   mpu9250, 
                 mpu_data_struct*   mpu6886, 
                 imu_data_struct*   imu )
{
    static int iter = 0;
    static int current = 0;

    Data_Packet[current].carts_pos_x[iter]          = carts_data->pos.x;
    Data_Packet[current].carts_pos_y[iter]          = carts_data->pos.y;
    Data_Packet[current].carts_vel_x[iter]          = carts_data->vel.x;
    Data_Packet[current].carts_vel_y[iter]          = carts_data->vel.y;
    Data_Packet[current].carts_acc_x[iter]          = carts_data->acc.x;
    Data_Packet[current].carts_acc_y[iter]          = carts_data->acc.y;
#if 0
    Data_Packet[current].mpu9250_acce_x[iter]       = mpu9250->acc.x;
    Data_Packet[current].mpu9250_acce_y[iter]       = mpu9250->acc.y;
    Data_Packet[current].mpu9250_acce_y[iter]       = mpu9250->acc.z;
    Data_Packet[current].mpu9250_gyro_x[iter]       = mpu9250->gyro.x;
    Data_Packet[current].mpu9250_gyro_y[iter]       = mpu9250->gyro.y;
    Data_Packet[current].mpu9250_gyro_y[iter]       = mpu9250->gyro.z;

    Data_Packet[current].mpu6886_acce_x[iter]       = mpu6886->acc.x;
    Data_Packet[current].mpu6886_acce_y[iter]       = mpu6886->acc.y;
    Data_Packet[current].mpu6886_acce_y[iter]       = mpu6886->acc.z;
    Data_Packet[current].mpu6886_gyro_x[iter]       = mpu6886->gyro.x;
    Data_Packet[current].mpu6886_gyro_y[iter]       = mpu6886->gyro.y;
    Data_Packet[current].mpu6886_gyro_y[iter]       = mpu6886->gyro.z;
#endif
    Data_Packet[current].pitch_no_filter[iter]      = imu->no_filter.pitch;
    Data_Packet[current].roll_no_filter[iter]       = imu->no_filter.roll;
    Data_Packet[current].pitch_complementary[iter]  = imu->complementary.pitch;
    Data_Packet[current].roll_complementary[iter]   = imu->complementary.roll;
    Data_Packet[current].pitch_alfa_beta[iter]      = imu->alfa_beta.pitch;
    Data_Packet[current].roll_alfa_beta[iter]       = imu->alfa_beta.roll;
    Data_Packet[current].pitch_kalman[iter]         = imu->kalman.pitch;
    Data_Packet[current].roll_kalman[iter]          = imu->kalman.roll;
    Data_Packet[current].pitch[iter]                = imu->angles.pitch;
    Data_Packet[current].roll[iter]                 = imu->angles.roll;

    iter++;
    iter %= 200;

    if (iter == 0) {
        Previous = current;
        current++;
        current %= 2;
    }
}
