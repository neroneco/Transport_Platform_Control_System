/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tcp_server.h"
#include "pid.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim6;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M7 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/* USER CODE BEGIN 1 */

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart4;

extern int				      Data_Request;
extern int                    RX_Status;
extern int  				  Previous;
extern uint8_t                Buffer_RX[10];
extern data_packet_struct     Data_Packet[2];

extern motor_status_struct OUT_motor_status;
extern motor_status_struct  IN_motor_status;

extern config_packet_struct config_packet;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    HAL_GPIO_TogglePin(STATUS_REG_GPIO_Port, STATUS_REG_Pin);

	static mpu_data_struct mpu9250;
	static mpu_data_struct mpu6886;
	//static mpu_data_struct mean;
	static imu_data_struct imu;

	read_sensors_mpu9250( &hi2c1, &mpu9250 );
	read_sensors_mpu6886( &hi2c2, &mpu6886 );

	filter_none( &mpu9250 );
	filter_none( &mpu6886 );

	filter_mean(			&mpu9250, &mpu6886, &imu );

	filter_complementary( 	&mpu9250, &mpu6886, &imu  );
	filter_alfa_beta( 		&mpu9250, &mpu6886, &imu  );
	filter_kalman( 			&mpu9250, &mpu6886, &imu  );

    OUT_motor_status.pos[0] = config_packet.x_position;
    OUT_motor_status.vel[0] = config_packet.x_velocity;
    OUT_motor_status.acc[0] = 0.0;
    OUT_motor_status.en[0]  = config_packet.x_en;
    OUT_motor_status.pos[1] = config_packet.y_position;
    OUT_motor_status.vel[1] = config_packet.y_velocity;
    OUT_motor_status.acc[1] = 0.0;
    OUT_motor_status.en[1]  = config_packet.y_en;



    static float mov_avg[50];
    static float   e[4];
    static float   e_cart;
    static float D_e;

    // mean
    float avg = 0.0;
    for ( int i = 0; i < 49; i++ ) {
        mov_avg[49-i] = mov_avg[48-i];
        avg += 0.02*mov_avg[49-i];
    }
    mov_avg[0] = imu.kalman.pitch;
    avg += 0.02*mov_avg[0];
    e[0] = avg;


    static float dt = 0.01;

    static float x_f;
    static float x_v_f;
    static float x_a_f;
    static float P_f;
    static float D_f;

    static float x_sr;
    static float x_v_sr;
    static float x_a_sr;
    static float P_sr;
    static float I_sr;

    static float x;
    static float x_v;
    static float x_a;

    static float raw;

    static int motor_iter;
    motor_iter++;
    motor_iter %= 2;
    static int mode;
    mode = config_packet.mode;
    if (!motor_iter){

        if ( mode == MANUAL ) { // mode = manual

        } else if ( mode == AUTO ) { // mode = regulate

            // derivative
            D_e= (1.0/(6.0*dt))*( e[0] + 3.0*e[1] - 3.0*e[2] - e[3] );
            e[3] = e[2];
            e[2] = e[1];
            e[1] = e[0];
//            if ( (D_e < 2.0) && (D_e > -2.0) ) {
//                D_e = 0.0;
//            }

            //                        P     D
            //x_f   = PD( -e[0], -D_e, 0.0, 20.0, dt );

            //e_cart = IN_motor_status.adc_pos[0];
            //x_v_sr = PI_cart( -e_cart, 0.11, 0.0020, dt );
            x_v_f = PD_platform( -e[0], -D_e, 1.0, 2.3, dt ); //D = 2.5
            //x_a_f = PD( -e[0], -D_e, 20.0, 0.0, dt );
            x_v = x_v_f + x_v_sr;
            raw = x_v;
            if (x_v > 0) {
                x = 160.0;
            } else {
                x = -160.0;
            }
            //x = x_f;
            x_v = fabs(x_v);
            //x_a = fabs(x_a_f);

            // limits
            // x
            if ( x > 160.0 ) {
                x =  160.0;
            } else if ( x < -160.0 ) {
                x = -160.0;
            }
            // v
            if ( x_v > 200.0 ) {
                x_v =  200.0;
            } else if ( x_v < 1.0 ) {
                x_v = 0.0;
            }
            // a
            if ( x_a > 600.0 ) {
                x_a =  600.0;
            } else if ( x_a < 0.0 ) {
                x_a = 0.0;
            }

            int   x_en = ENABLED;

            OUT_motor_status.pos[0] = x;
            OUT_motor_status.vel[0] = x_v;
            OUT_motor_status.acc[0] = x_a;
            OUT_motor_status.en[0]  = x_en;
        }

        HAL_GPIO_TogglePin(ST_UART_GPIO_Port, ST_UART_Pin);
        HAL_UART_Transmit(&huart4, (uint8_t*)&OUT_motor_status, sizeof(motor_status_struct), 1);
        HAL_UART_Receive( &huart4, (uint8_t*)&IN_motor_status , sizeof(motor_status_struct), 1);
        HAL_GPIO_TogglePin(ST_UART_GPIO_Port, ST_UART_Pin);
    }

    static int iter 	= 0 ;
    static int current 	= 0 ;

    Data_Packet[current].carts_pos_adc_x[iter]      = IN_motor_status.adc_pos[0];
    Data_Packet[current].carts_pos_x[iter]          = IN_motor_status.pos[0];
    Data_Packet[current].carts_pos_adc_y[iter]      = IN_motor_status.adc_pos[1];
    Data_Packet[current].carts_pos_y[iter]          = IN_motor_status.pos[1];
    Data_Packet[current].carts_vel_x[iter]          = IN_motor_status.vel[0];
    Data_Packet[current].carts_vel_y[iter]          = IN_motor_status.vel[1];
    Data_Packet[current].carts_acc_x[iter]          = IN_motor_status.acc[0];
    Data_Packet[current].carts_acc_y[iter]          = IN_motor_status.acc[1];

    Data_Packet[current].mpu9250_acce_x[iter]       = mpu9250.acc.x;
    Data_Packet[current].mpu9250_acce_y[iter]       = mpu9250.acc.y;
    Data_Packet[current].mpu9250_acce_z[iter]       = mpu9250.acc.z;
    Data_Packet[current].mpu9250_gyro_x[iter]       = mpu9250.gyro.x;
    Data_Packet[current].mpu9250_gyro_y[iter]       = mpu9250.gyro.y;
    Data_Packet[current].mpu9250_gyro_z[iter]       = mpu9250.gyro.z;
    Data_Packet[current].mpu9250_pitch[iter]       	= mpu9250.angle.pitch;
    Data_Packet[current].mpu9250_roll[iter]       	= mpu9250.angle.roll;

    Data_Packet[current].mpu6886_acce_x[iter]       = mpu6886.acc.x;
    Data_Packet[current].mpu6886_acce_y[iter]       = mpu6886.acc.y;
    Data_Packet[current].mpu6886_acce_z[iter]       = mpu6886.acc.z;
    Data_Packet[current].mpu6886_gyro_x[iter]       = mpu6886.gyro.x;
    Data_Packet[current].mpu6886_gyro_y[iter]       = mpu6886.gyro.y;
    Data_Packet[current].mpu6886_gyro_z[iter]       = mpu6886.gyro.z;
    Data_Packet[current].mpu6886_pitch[iter]        = mpu6886.angle.pitch;
    Data_Packet[current].mpu6886_roll[iter]         = mpu6886.angle.roll;


    Data_Packet[current].pitch_no_filter[iter]      = e[0];
    Data_Packet[current].roll_no_filter[iter]       = x;
    Data_Packet[current].pitch_complementary[iter]  = -D_e;
    Data_Packet[current].roll_complementary[iter]   = raw;
    Data_Packet[current].pitch_alfa_beta[iter]      = 0.0;
    Data_Packet[current].roll_alfa_beta[iter]       = x_a;
    Data_Packet[current].pitch_kalman[iter]         = imu.kalman.pitch;
    Data_Packet[current].roll_kalman[iter]          = imu.kalman.roll;
    Data_Packet[current].pitch[iter]                = (float)iter;
    Data_Packet[current].roll[iter]                 = (float)iter;

    iter++;
    iter %= 100;

    if (iter == 0) {
        Previous = current;
        current++;
        current %= 2;
    }
    HAL_GPIO_TogglePin(STATUS_REG_GPIO_Port, STATUS_REG_Pin);
}
/* USER CODE END 1 */
