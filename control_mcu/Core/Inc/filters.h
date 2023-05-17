#ifndef INC_filters_H_
#define INC_filters_H_

#include "accelerometers.h"
#include <math.h>


//-------------------------------------------------
//          filters data structers/enumes
//-------------------------------------------------

enum FILTER_TYPE {
    NONE,
    COMPLEMENTARY,
    alfa_beta,
    KALMAN,
    ALL
};

//typedef struct {
//    float pitch;
//    float roll;
//} angles_data_struct;

typedef struct {
	angles_data_struct mean;
    angles_data_struct no_filter;
    angles_data_struct complementary;
    angles_data_struct alfa_beta;
    angles_data_struct kalman;
    angles_data_struct angles;
} imu_data_struct;

//-------------------------------------------------
//               filters functions
//-------------------------------------------------

void filter_none( mpu_data_struct* mpu );

void filter_complementary( mpu_data_struct* mpu9250, 
                           mpu_data_struct* mpu6886, 
                           imu_data_struct* imu );

void filter_alfa_beta( mpu_data_struct* mpu9250,
                       mpu_data_struct* mpu6886, 
                       imu_data_struct* imu );

void filter_kalman( mpu_data_struct* mpu9250, 
                    mpu_data_struct* mpu6886, 
                    imu_data_struct* imu );

void filter_mean(	mpu_data_struct* mpu9250,
        			mpu_data_struct* mpu6886,
					imu_data_struct* imu      );

void imu_init(void);

void imu_get_pitch_roll( int filter_type, 
                         mpu_data_struct* mpu9250, 
                         mpu_data_struct* mpu6886, 
                         imu_data_struct* imu );

#endif /* INC_filters_H_ */
