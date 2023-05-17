/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
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
#include "stm32l4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdlib.h>
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
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart4;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
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
  * @brief This function handles Prefetch fault, memory access fault.
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
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC channel1 and channel2 underrun error interrupts.
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
extern ADC_HandleTypeDef hadc1;

volatile int adc_conv_complate = 0;

extern uint32_t adc_out[2];
extern float  volt_dist[2];
extern float  pos[2];
extern float  pos_new[2];
extern float  pos_prev[2];

enum DIR{
    DIR_PLUS = 1,
    DIR_MINUS = -1
};

enum STEPPER_STATUS{
    ENABLED,
    DISABLED
};

static float K = 0.0393;

static int x_dir = DIR_PLUS;
static int y_dir = DIR_PLUS;
static int x_en = DISABLED;
static int y_en = DISABLED;

static int x_step = 0;
static int y_step = 0;
static int x_des_pos = 0;
static int y_des_pos = 0;

static int x_speed = 10000;
static int y_speed = 10000;

static void ES_STOP(){
    HAL_TIM_Base_Stop_IT(&htim6);
    HAL_GPIO_WritePin(X_EN_GPIO_Port, X_EN_Pin ,GPIO_PIN_SET);
    HAL_GPIO_WritePin(Y_EN_GPIO_Port, Y_EN_Pin ,GPIO_PIN_SET);
    while(1){
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        HAL_Delay(1000);
    };
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static int iter = 0;

    static int x_v = 0;
    static int y_v = 0;

    x_v %= x_speed;
    y_v %= y_speed;

    // Step
    if (x_step != x_des_pos){
        if (x_en == ENABLED){
            if (!x_v){
                HAL_GPIO_TogglePin(X_STEP_GPIO_Port, X_STEP_Pin);
                switch (x_dir)
                {
                case DIR_PLUS:
                    x_step++;
                    break;
                case DIR_MINUS:
                    x_step--;
                    break;
                }
            }
        }
    }
    if (y_step != y_des_pos){
        if (y_en == ENABLED){
            if (!y_v){
                HAL_GPIO_TogglePin(Y_STEP_GPIO_Port, Y_STEP_Pin);
                switch (y_dir)
                {
                case DIR_PLUS:
                    y_step++;
                    break;
                case DIR_MINUS:
                    y_step--;
                    break;
                }
            }
        }
    }

    x_v++;
    y_v++;


    iter %= 2000;
    if (!iter) {
        HAL_GPIO_TogglePin(st_UART_GPIO_Port, st_UART_Pin);
        HAL_TIM_Base_Stop_IT(&htim6);

        volt_dist[0] = 0.0008056640625f * (float)adc_out[0];
        volt_dist[1] = 0.0008056640625f * (float)adc_out[1];
        static float ax = -319.6;
        static float bx = -0.7314;
        static float cx =  374.9;
        static float ay = -230.5;
        static float by = -0.8702;
        static float cy =  230.7;

        if (volt_dist[0]>2.2){
            ES_STOP();
            //volt_dist[0] = 2.2;
        }
        else if (volt_dist[0]<0.45){
            ES_STOP();
            //volt_dist[0] = 0.45;
        }

        if (volt_dist[1]>1.8){
            ES_STOP();
            //volt_dist[1] = 1.8;
        }
        else if (volt_dist[1]<0.65){
            ES_STOP();
            //volt_dist[1] = 0.65;
        }

        pos_new[0] = ax*pow(volt_dist[0],bx) + cx;
        pos_new[1] = ay*pow(volt_dist[1],by) + cy;

        static float alpha = 0.15;
        pos[0] = (1.0-alpha)*pos_prev[0] + alpha*pos_new[0];
        pos[1] = (1.0-alpha)*pos_prev[1] + alpha*pos_new[1];
        pos_prev[0] = pos[0];
        pos_prev[1] = pos[1];

        if ( fabs(x_step*K - pos_prev[0])> 15){
            x_step = (int)(pos_prev[0]/K);
        }

        if ( fabs(y_step*K - pos_prev[1])> 15){
            y_step = (int)(pos_prev[1]/K);
        }

        if ( (pos_prev[0]>170) || (pos_prev[0]<-170) || (pos_prev[1]>70) || (pos_prev[1]<-70)){
            ES_STOP();
            //volt_dist[0] = 2.2;
        }

        HAL_ADC_Start_DMA(&hadc1, adc_out, 2);
        HAL_TIM_Base_Start_IT(&htim6);
        HAL_GPIO_TogglePin(st_UART_GPIO_Port, st_UART_Pin);
    }
    iter++;
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    HAL_GPIO_TogglePin(st_ADC_GPIO_Port, st_ADC_Pin);
    HAL_ADC_Stop_DMA(&hadc1);

    HAL_GPIO_TogglePin(st_ADC_GPIO_Port, st_ADC_Pin);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    HAL_GPIO_WritePin(st_ADC_GPIO_Port, st_ADC_Pin, GPIO_PIN_SET);
}



extern motor_status_struct OUT_motor_status;
extern motor_status_struct IN_motor_status;


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    x_des_pos = (int)((IN_motor_status.pos[0])/K);
    y_des_pos = (int)((IN_motor_status.pos[1])/K);
    x_speed   = (int)(25000/(IN_motor_status.vel[0]/K));
    y_speed   = (int)(25000/(IN_motor_status.vel[1]/K));

    if(x_speed > 0){
        x_dir = DIR_PLUS;
    } else {
        x_dir = DIR_MINUS;
    }
    if(y_speed > 0){
        y_dir = DIR_PLUS;
    } else {
        y_dir = DIR_MINUS;
    }

    x_speed = abs(x_speed);
    y_speed = abs(y_speed);


    // Direction
    if (x_dir == DIR_PLUS)
        HAL_GPIO_WritePin(X_DIR_GPIO_Port,  X_DIR_Pin ,GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(X_DIR_GPIO_Port,  X_DIR_Pin ,GPIO_PIN_RESET);

    if (y_dir == DIR_PLUS)
        HAL_GPIO_WritePin(Y_DIR_GPIO_Port,  Y_DIR_Pin ,GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(Y_DIR_GPIO_Port,  Y_DIR_Pin ,GPIO_PIN_RESET);

    x_en = IN_motor_status.en[0];
    y_en = IN_motor_status.en[1];
    // Enable
    if (x_en == ENABLED)
        HAL_GPIO_WritePin(X_EN_GPIO_Port,  X_EN_Pin ,GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(X_EN_GPIO_Port,  X_EN_Pin ,GPIO_PIN_SET);
    if (y_en == ENABLED)
        HAL_GPIO_WritePin(Y_EN_GPIO_Port,  Y_EN_Pin ,GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(Y_EN_GPIO_Port,  Y_EN_Pin ,GPIO_PIN_SET);


    static int i = 0;
    OUT_motor_status.pos[0] = ((float)x_step*K);
    OUT_motor_status.vel[0] = (float)((K*25000/x_speed)*x_dir);
    OUT_motor_status.acc[0] = 50.0*sin(2.0*M_PI*0.001*i + 2.0*M_PI/3.0);
    OUT_motor_status.pos[1] = ((float)y_step*K);
    OUT_motor_status.vel[1] = (float)((K*25000/y_speed)*x_dir);
    OUT_motor_status.acc[1] = 20.0*sin(2.0*M_PI*0.001*i + 2.0*M_PI/3.0);
    i++;
    HAL_UART_Transmit_IT(&huart4, (uint8_t*)&OUT_motor_status, sizeof(motor_status_struct));
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	HAL_UART_Receive_IT(&huart4, (uint8_t*)&IN_motor_status, sizeof(motor_status_struct));
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	if (HAL_UART_Init(&huart4) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_UART_Receive_IT(&huart4, (uint8_t*)&IN_motor_status, sizeof(motor_status_struct));
}

void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *huart){
	HAL_UART_Transmit_IT(&huart4, (uint8_t*)&OUT_motor_status, sizeof(motor_status_struct));

}
void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart){
	HAL_UART_Receive_IT(&huart4, (uint8_t*)&IN_motor_status, sizeof(motor_status_struct));
}



/* USER CODE END 1 */
