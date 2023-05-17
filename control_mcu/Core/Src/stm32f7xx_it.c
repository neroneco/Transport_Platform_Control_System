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

	//uint8_t RX_Buffer[20] = {'a','b','c','d','e','f','g','h','i','j'};
//	motor_status.pos = 10.0;
//	motor_status.vel = 10.0;
//	motor_status.acc = 10.0;
	//HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)RX_Buffer, (uint8_t*)&motor_status, 12, 3);
	//static uint8_t i = 1;
	uint8_t result = 0;
	static uint8_t out = 1;

	OUT_motor_status.pos[0] = 1.1;
	OUT_motor_status.vel[0] = 10.1;
	OUT_motor_status.acc[0] = 100.1;

	HAL_GPIO_TogglePin(ST_UART_GPIO_Port, ST_UART_Pin);
	HAL_UART_Transmit(&huart4, (uint8_t*)&OUT_motor_status, sizeof(motor_status_struct), 1);
	HAL_UART_Receive( &huart4, (uint8_t*)&IN_motor_status , sizeof(motor_status_struct), 1);
	HAL_GPIO_TogglePin(ST_UART_GPIO_Port, ST_UART_Pin);
	//HAL_UART_Receive( &huart4, (uint8_t*)&result, sizeof(uint8_t), 1);
	//HAL_GPIO_WritePin(ST_UART_GPIO_Port, ST_UART_Pin,GPIO_PIN_RESET);

	out++;
	out %= 21;
	//HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&out, (uint8_t*)&result, sizeof(uint8_t), 2);

	//HAL_SPI_Receive( &hspi1, (uint8_t*)RX_Buffer, sizeof(motor_status_struct), 2);

    static int iter 	= 0 ;
    static int current 	= 0 ;

    Data_Packet[current].carts_pos_x[iter]          = IN_motor_status.pos[0];
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
