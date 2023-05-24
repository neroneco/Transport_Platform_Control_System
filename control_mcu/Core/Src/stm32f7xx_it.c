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

    static float mov_avg[10];
    mov_avg[9] = mov_avg[8];
    mov_avg[8] = mov_avg[7];
    mov_avg[7] = mov_avg[6];
    mov_avg[6] = mov_avg[5];
    mov_avg[5] = mov_avg[4];
    mov_avg[4] = mov_avg[3];
    mov_avg[3] = mov_avg[2];
    mov_avg[2] = mov_avg[1];
    mov_avg[1] = mov_avg[0];
    mov_avg[0] = imu.kalman.pitch;
    float control_pitch = (mov_avg[0] + mov_avg[1] + mov_avg[2] + mov_avg[3] + mov_avg[4] + mov_avg[5] + mov_avg[6] + mov_avg[7] + mov_avg[8] + mov_avg[9] ) / 10.0;
    static int motor_iter;
    motor_iter++;
    motor_iter %= 10;
    static int mode;
    mode = config_packet.mode;
    if (!motor_iter){

        if ( mode == MANUAL ) { // mode = manual

        } else if ( mode == AUTO ) { // mode = regulate
//            static float P_pos = 10000.0;
//            float x_zad = P_pos * ( 0 + control_pitch );
//            if (control_pitch > 0.2) {
//                x_zad = -160.0;
//            } else if (control_pitch < -0.2) {
//                x_zad =  160.0;
//            }

            static float P_vel  = 3.0;
            static float P_acel = 5.0;
            static float contr[4];
            contr[3] = contr[2];
            contr[2] = contr[1];
            contr[1] = contr[0];
            contr[0] = control_pitch;
            float D_contr= (1.0/(6.0*0.05))*(contr[0] + 3.0*contr[1] - 3.0*contr[2] - contr[3]);

            float x_v_zad = P_vel  * fabs(D_contr);
            float x_a_zad = P_acel * fabs(D_contr);
            float x_zad;
            if (D_contr > 0.0){
                x_zad =  -160.0;
            } else if (D_contr <= -0.0) {
                x_zad =  160.0;
            } else {
                x_zad = 0.0;
                x_v_zad = 0.0;
            }
            if (x_v_zad > 200.0) {
                x_v_zad =  200.0;
            } else if (x_v_zad < 10.0) {
                x_v_zad = 0.0;
            }
            if (x_a_zad > 600.0) {
                x_a_zad =  600.0;
            } else if (x_a_zad < 50.0) {
                x_a_zad = 50.0;
            }
            int   x_en = ENABLED;

            OUT_motor_status.pos[0] = x_zad;
            OUT_motor_status.vel[0] = x_v_zad;
            OUT_motor_status.acc[0] = x_a_zad;
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


    Data_Packet[current].pitch_no_filter[iter]      = imu.mean.pitch;
    Data_Packet[current].roll_no_filter[iter]       = imu.mean.roll;
    Data_Packet[current].pitch_complementary[iter]  = imu.complementary.pitch;
    Data_Packet[current].roll_complementary[iter]   = imu.complementary.roll;
    Data_Packet[current].pitch_alfa_beta[iter]      = imu.alfa_beta.pitch;
    Data_Packet[current].roll_alfa_beta[iter]       = imu.alfa_beta.roll;
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
