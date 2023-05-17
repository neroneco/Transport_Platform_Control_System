#ifndef INC_control_H_
#define INC_control_H_

#include "communication.h"
#include "accelerometers.h"
#include "filters.h"
//-------------------------------------------------
//             control data structers
//-------------------------------------------------

typedef struct {
    axis_data_struct pos;
    axis_data_struct vel;
    axis_data_struct acc;
} carts_data_struct;

//-------------------------------------------------
//             control functions
//-------------------------------------------------

void controler( int regulator_type, 
                int filter_type );

void carts_get_data( carts_data_struct* carts_data );

void send_control_signal( carts_data_struct* carts_data );

void store_data( carts_data_struct* carts_data,
                 mpu_data_struct*   mpu9250, 
                 mpu_data_struct*   mpu6886, 
                 imu_data_struct*   imu );

#endif /* INC_control_H_ */
