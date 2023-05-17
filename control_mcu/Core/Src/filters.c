
#include "filters.h"
#include <math.h>
#include <stdlib.h>

extern I2C_HandleTypeDef* h_i2c_1;
extern I2C_HandleTypeDef* h_i2c_2;

void filter_none( mpu_data_struct* mpu )
{
    float ax = mpu->acc.x ;
    float ay = mpu->acc.y ;
    float az = mpu->acc.z ;

    mpu->angle.pitch = atan2(   az , sqrt((ay*ay) + (ax*ax)) ) * 57.3 ;
    mpu->angle.roll  = atan2( (-ay), sqrt((ax*ax) + (az*az)) ) * 57.3 ;
}

void filter_complementary( mpu_data_struct* mpu9250, 
                           mpu_data_struct* mpu6886, 
                           imu_data_struct* imu )  
{
    static float gx ;
    static float gy ;
    static float gz ;

    static float T = 0.005 ;

    gx = mpu9250->gyro.x ;
    gy = mpu9250->gyro.y ;
    gz = mpu9250->gyro.z ;

    angles_data_struct acc  ;
    angles_data_struct gyro ;
    static angles_data_struct complementary = {0} ;

    acc.pitch = imu->mean.pitch ;
    acc.roll  = imu->mean.roll  ;

    gyro.pitch = gy * T + complementary.pitch ;
    gyro.roll  = gz * T + complementary.roll  ;

    complementary.pitch = 0.025 * acc.pitch + 0.975 * gyro.pitch ;
    complementary.roll  = 0.025 * acc.roll  + 0.975 * gyro.roll  ;

    imu->complementary.pitch = complementary.pitch ;
    imu->complementary.roll  = complementary.roll  ;
}




typedef struct {
    angles_data_struct pos ;
    angles_data_struct vel ;
} pos_vel_struct;

void filter_alfa_beta(   mpu_data_struct* mpu9250, 
                                mpu_data_struct* mpu6886, 
                                imu_data_struct* imu ) 
{

    static float alpha = 0.02 ;
    static float beta  = 1.0 ;

    static float T = 0.005 ;

    static pos_vel_struct measure ;
    static pos_vel_struct predict ;
    static pos_vel_struct smooth  ;

//    static float ax ;
//    static float ay ;
//    static float az ;
//
//    ax = mpu9250->acc.x ;
//    ay = mpu9250->acc.y ;
//    az = mpu9250->acc.z ;
//
//    measure.pos.pitch = atan2(   az , sqrt((ay*ay) + (ax*ax)) ) * 57.3 ;
//    measure.pos.roll  = atan2( (-ay), sqrt((ax*ax) + (az*az)) ) * 57.3 ;

    measure.pos.pitch = imu->mean.pitch ;
    measure.pos.roll  = imu->mean.roll  ;

    measure.vel.pitch = mpu9250->gyro.y ;
    measure.vel.roll  = mpu9250->gyro.z ;

    // Prediction
    predict.pos.pitch = smooth.pos.pitch + ( T * smooth.vel.pitch ) ;
    predict.pos.roll  = smooth.pos.roll  + ( T * smooth.vel.roll  ) ;
    predict.vel.pitch = smooth.vel.pitch ;
    predict.vel.roll  = smooth.vel.roll  ;

    // Correction
    smooth.pos.pitch = predict.pos.pitch + alpha  * ( measure.pos.pitch - predict.pos.pitch ) ;
    smooth.pos.roll  = predict.pos.roll  + alpha  * ( measure.pos.roll  - predict.pos.roll  ) ;
    smooth.vel.pitch = predict.vel.pitch + beta   * ( measure.vel.pitch - predict.vel.pitch ) ;
    smooth.vel.roll  = predict.vel.roll  + beta   * ( measure.vel.roll  - predict.vel.roll  ) ;

    imu->alfa_beta.pitch = smooth.pos.pitch ;
    imu->alfa_beta.roll  = smooth.pos.roll  ;

}

typedef struct {
    angles_data_struct pos      ;
    angles_data_struct drift    ;
} state_vector_struct;

typedef struct {
    float P_pred[4]  ;
    float AP[4]      ;
    float P[4]       ;
    float Q[4]       ;
    float R          ;
    float e          ;
    float S          ;
    float K[2]       ;
} kalman_struct;

void filter_kalman( mpu_data_struct* mpu9250, 
                    mpu_data_struct* mpu6886, 
                    imu_data_struct* imu ) 
{
    static pos_vel_struct   	measure  ;
    static state_vector_struct  predict  ;
    static state_vector_struct  estimate ;


    // A = [ 1 -T ;   B = [ T 
    //       0  1 ]         0 ]
    static kalman_struct kalman_pitch = {   .Q = {0.03,0,0,0.1},
                                            .R = 50              };
    static kalman_struct kalman_roll  = {   .Q = {0.03,0,0,0.1},
                                            .R = 50              };

    static float T = 0.005 ;

//    static float ax ;
//    static float ay ;
//    static float az ;
//
//    ax = mpu9250->acc.x ;
//    ay = mpu9250->acc.y ;
//    az = mpu9250->acc.z ;

//    measure.pos.pitch = atan2(   az , sqrt((ay*ay) + (ax*ax)) ) * 57.3 ;
//    measure.pos.roll  = atan2( (-ay), sqrt((ax*ax) + (az*az)) ) * 57.3 ;

    measure.pos.pitch = imu->mean.pitch ;
    measure.pos.roll  = imu->mean.roll  ;

    measure.vel.pitch = mpu9250->gyro.y ;
    measure.vel.roll  = mpu9250->gyro.z ;

    // Pitch
    {
        predict.pos.pitch   = estimate.pos.pitch    - ( T * estimate.drift.pitch ) + ( T * measure.vel.pitch ) ;
        predict.drift.pitch = estimate.drift.pitch ;

        kalman_pitch.AP[0] = ( kalman_pitch.P[0] * 1 ) + ( kalman_pitch.P[2] * (-T) ) ;
        kalman_pitch.AP[1] = ( kalman_pitch.P[1] * 1 ) + ( kalman_pitch.P[3] * (-T) ) ;
        kalman_pitch.AP[2] = ( kalman_pitch.P[2] * 1 ) ;
        kalman_pitch.AP[3] = ( kalman_pitch.P[3] * 1 ) ;

        kalman_pitch.P_pred[0] = kalman_pitch.AP[0] + ( kalman_pitch.AP[1] * (-T) ) + kalman_pitch.Q[0] ;
        kalman_pitch.P_pred[1] = kalman_pitch.AP[1]                                 + kalman_pitch.Q[1] ;
        kalman_pitch.P_pred[2] = kalman_pitch.AP[2] + ( kalman_pitch.AP[3] * (-T) ) + kalman_pitch.Q[2] ;
        kalman_pitch.P_pred[3] = kalman_pitch.AP[3]                                 + kalman_pitch.Q[3] ;

        kalman_pitch.e = measure.pos.pitch - predict.pos.pitch ;

        kalman_pitch.S = kalman_pitch.P_pred[0] + kalman_pitch.R ;

        kalman_pitch.K[0] = kalman_pitch.P_pred[0] * (1/kalman_pitch.S) ;
        kalman_pitch.K[1] = kalman_pitch.P_pred[2] * (1/kalman_pitch.S) ;

        // Estimation
        estimate.pos.pitch      = predict.pos.pitch   + kalman_pitch.K[0] * kalman_pitch.e ;
        estimate.drift.pitch    = predict.drift.pitch + kalman_pitch.K[1] * kalman_pitch.e ;

        kalman_pitch.P[0] = (1 - kalman_pitch.K[0]) * kalman_pitch.P_pred[0] ;
        kalman_pitch.P[1] = (1 - kalman_pitch.K[0]) * kalman_pitch.P_pred[1] ;
        kalman_pitch.P[2] = (( - kalman_pitch.K[1]) * kalman_pitch.P_pred[0]) + kalman_pitch.P_pred[2] ;
        kalman_pitch.P[3] = (( - kalman_pitch.K[1]) * kalman_pitch.P_pred[1]) + kalman_pitch.P_pred[3] ;

        imu->kalman.pitch = estimate.pos.pitch ;
    }

    // Roll
    {
        predict.pos.roll    = estimate.pos.roll     - ( T * estimate.drift.roll )  + ( T * measure.vel.roll ) ;
        predict.drift.roll  = estimate.drift.roll ;

        kalman_roll.AP[0] = ( kalman_roll.P[0] * 1 ) + ( kalman_roll.P[2] * (-T) ) ;
        kalman_roll.AP[1] = ( kalman_roll.P[1] * 1 ) + ( kalman_roll.P[3] * (-T) ) ;
        kalman_roll.AP[2] = ( kalman_roll.P[2] * 1 ) ;
        kalman_roll.AP[3] = ( kalman_roll.P[3] * 1 ) ;

        kalman_roll.P_pred[0] = kalman_roll.AP[0] + ( kalman_roll.AP[1] * (-T) ) + kalman_roll.Q[0] ;
        kalman_roll.P_pred[1] = kalman_roll.AP[1]                                 + kalman_roll.Q[1] ;
        kalman_roll.P_pred[2] = kalman_roll.AP[2] + ( kalman_roll.AP[3] * (-T) ) + kalman_roll.Q[2] ;
        kalman_roll.P_pred[3] = kalman_roll.AP[3]                                 + kalman_roll.Q[3] ;

        kalman_roll.e = measure.pos.roll - predict.pos.roll ;

        kalman_roll.S = kalman_roll.P_pred[0] + kalman_roll.R ;

        kalman_roll.K[0] = kalman_roll.P_pred[0] * (1/kalman_roll.S) ;
        kalman_roll.K[1] = kalman_roll.P_pred[2] * (1/kalman_roll.S) ;

        // Estimation
        estimate.pos.roll      = predict.pos.roll   + kalman_roll.K[0] * kalman_roll.e ;
        estimate.drift.roll    = predict.drift.roll + kalman_roll.K[1] * kalman_roll.e ;

        kalman_roll.P[0] = (1 - kalman_roll.K[0]) * kalman_roll.P_pred[0] ;
        kalman_roll.P[1] = (1 - kalman_roll.K[0]) * kalman_roll.P_pred[1] ;
        kalman_roll.P[2] = (( - kalman_roll.K[1]) * kalman_roll.P_pred[0]) + kalman_roll.P_pred[2] ;
        kalman_roll.P[3] = (( - kalman_roll.K[1]) * kalman_roll.P_pred[1]) + kalman_roll.P_pred[3] ;

        imu->kalman.roll = estimate.pos.roll ;
    }
}

void filter_mean(	mpu_data_struct* mpu9250,
        			mpu_data_struct* mpu6886,
					imu_data_struct* imu      )
{

	imu->mean.pitch = 0.5 * ( (mpu9250->angle.pitch - 0.6478020) + (mpu6886->angle.pitch + 0.2368675) );
	imu->mean.roll  = 0.5 * ( (mpu9250->angle.roll  + 0.55) + (mpu6886->angle.roll  + 6.77   ) );


//	static float gx9250, gx6886 ;
//	static float gy9250, gy6886 ;
//	static float gz9250, gz6886 ;
//
//	ax9250 = mpu9250->acc.x  ;
//	ay9250 = mpu9250->acc.y  ;
//	az9250 = mpu9250->acc.z  ;
//	ax6886 = mpu6886->acc.x  + 0.0341372;
//	ay6886 = mpu6886->acc.y  - 0.0250024;
//	az6886 = mpu6886->acc.z  + 0.0649273;
//	gx9250 = mpu9250->gyro.x - 1.174190266666665;
//	gy9250 = mpu9250->gyro.y + 0.746404666666667;
//	gz9250 = mpu9250->gyro.z - 0.957398666666667;
//	gx6886 = mpu6886->gyro.x - 5.712968800000009;
//	gy6886 = mpu6886->gyro.y + 17.113791066666668;
//	gz6886 = mpu6886->gyro.z - 13.199390666666675;
//
//	mean->acc.x = (weight_acc * ax9250) + ((1-weight_acc) * ax6886) ;
//	mean->acc.y = (weight_acc * ay9250) + ((1-weight_acc) * ay6886) ;
//	mean->acc.z = (weight_acc * az9250) + ((1-weight_acc) * az6886) ;
//
//	mean->gyro.x = (weight_gyro * gx9250) + ((1-weight_gyro) * gx6886) ;
//	mean->gyro.y = (weight_gyro * gy9250) + ((1-weight_gyro) * gy6886) ;
//	mean->gyro.z = (weight_gyro * gz9250) + ((1-weight_gyro) * gz6886) ;
}

void imu_init()
{
    mpu9250_Init( h_i2c_1 );
    mpu6886_Init( h_i2c_2 );
}

void imu_get_pitch_roll( int filter_type, 
                         mpu_data_struct* mpu9250, 
                         mpu_data_struct* mpu6886, 
                         imu_data_struct* imu )
{
    read_sensors( h_i2c_1, mpu9250 );
    read_sensors( h_i2c_2, mpu6886 );

    switch( filter_type )
    {
        case COMPLEMENTARY :
        {
            filter_complementary( mpu9250, mpu6886, imu );
            imu->angles.pitch = imu->complementary.pitch;
            imu->angles.roll  = imu->complementary.roll;
            break;
        }
        case alfa_beta :
        {
            filter_alfa_beta( mpu9250, mpu6886, imu );
            imu->angles.pitch = imu->alfa_beta.pitch;
            imu->angles.roll  = imu->alfa_beta.roll;
            break;
        }
        case KALMAN :
        {
            filter_kalman( mpu9250, mpu6886, imu );
            imu->angles.pitch = imu->kalman.pitch;
            imu->angles.roll  = imu->kalman.roll;
            break;
        }
        case ALL :
        {
            filter_none( mpu9250 );
            filter_complementary( mpu9250, mpu6886, imu );
            filter_alfa_beta( mpu9250, mpu6886, imu );
            filter_kalman( mpu9250, mpu6886, imu );
            imu->angles.pitch = imu->no_filter.pitch;
            imu->angles.roll  = imu->no_filter.roll;
            break;
        }
        case NONE :
        default :
        {
            filter_none( mpu9250 );
            imu->angles.pitch = imu->no_filter.pitch;
            imu->angles.roll  = imu->no_filter.roll;
            break;
        }
    }
}
