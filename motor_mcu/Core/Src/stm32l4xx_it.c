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

#include "stepper_motor.h"
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

// ADC DMA optical sensor data
extern uint32_t adc_out[2];

// Global variables extern
//    X axis:
extern int x_en;
extern int x_freq_div;
extern int x_step_zad;
extern int x_step_akt;
extern float x_v_zad;
extern float x_v_akt;
extern float x_a_zad;
extern float x_a_akt;
extern int x_dir_zad;
extern int x_dir_akt;
extern int x_hold;

//    Y axis:
extern int y_en;
extern int y_freq_div;
extern int y_step_zad;
extern int y_step_akt;
extern float y_v_zad;
extern float y_v_akt;
extern float y_a_zad;
extern float y_a_akt;
extern int y_dir_zad;
extern int y_dir_akt;
extern int y_hold;

extern float K;      // [mm/imp]
extern int Max_Freq_Div;  // ~200[mm/s]
extern int Max_Freq;  // ~200[mm/s]
extern float Max_Position_X;
extern float Max_Position_Y;
extern float Max_V_X;
extern float Max_V_Y;

extern float K;      // [mm/imp]
extern int Max_Freq_Div;  // [-] ~200[mm/s]
extern int Min_Freq_Div;  // [-] ~200[mm/s]
extern int Max_Freq;  // [Hz] ~200[mm/s]
extern float Max_V_X;
extern float Min_V_X;
extern float Max_V_Y;
extern float Min_V_Y;
extern float Max_Position_X;
extern float Max_Position_Y;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    if (x_en == ENABLED) {
        make_step_x();
#if 0
            make_step_y();
#endif
    }

    // 100 [Hz] speed control
    static int iter;
    iter++;
    iter %= 500;
    if (!iter) {
        HAL_GPIO_TogglePin(st_UART_GPIO_Port, st_UART_Pin);
        HAL_TIM_Base_Stop_IT(&htim6);

        // 100 [Hz] check/set actual velocity
        {
            x_freq_div = set_v_x();
            if ( x_freq_div == Min_Freq_Div ) {
                x_hold = 1;
            } else {
                x_hold = 0;
            }
#if 0
            y_freq_div = set_v_y();
#endif
        }

        // 25[Hz] ADC optical sensor read
        // get position from optical sensor and convert them to steps position
        static int adc_iter;
        adc_iter++;
        adc_iter %= 4;
        if (!adc_iter) {

            static float adc_pos_x;
            static float adc_pos_y;
            adc_pos_x = adc_get_pos_x(adc_out[0]);
            adc_pos_y = adc_get_pos_y(adc_out[1]);

            // safety code: turn off motors and signal error status if adc_pos in danger zone
            if (check_danger_zone(adc_pos_x, Max_Position_X)) {
                ES_STOP();
            }
            if (check_danger_zone(adc_pos_y, Max_Position_Y)) {
                ES_STOP();
            }

            static int adc_steps_x;
            static int adc_steps_y;
            adc_steps_x = convert_position_to_steps(adc_pos_x);
            adc_steps_y = convert_position_to_steps(adc_pos_y);

            // check difference in measured position and expected position from steps of stepper motor
            // and swap them if the difference is larger then ~14[mm] (~14[mm] = ~356 steps)
            static int dif_steps_x;
            static int dif_steps_y;
            dif_steps_x = abs(adc_steps_x - x_step_akt);
            dif_steps_y = abs(adc_steps_y - y_step_akt);
            if ( dif_steps_x > 356 ) {
                x_step_akt = adc_steps_x;
            }
            if ( dif_steps_y > 356 ) {
                y_step_akt = adc_steps_y;
            }
        }

        HAL_ADC_Start_DMA(&hadc1, adc_out, 2);
        HAL_TIM_Base_Start_IT(&htim6);
        HAL_GPIO_TogglePin(st_UART_GPIO_Port, st_UART_Pin);
    }
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
    static int i = 0;
    i++;
    OUT_motor_status.pos[0] = convert_steps_to_position( x_step_akt );
    OUT_motor_status.vel[0] = x_v_akt;
    OUT_motor_status.acc[0] = 50.0*sin(2.0*M_PI*0.001*i + 2.0*M_PI/3.0);
    OUT_motor_status.pos[1] = convert_steps_to_position( y_step_akt );
    OUT_motor_status.vel[1] = y_v_akt;
    OUT_motor_status.acc[1] = 20.0*sin(2.0*M_PI*0.001*i + 2.0*M_PI/3.0);
    OUT_motor_status.en[0]  = x_en;
    OUT_motor_status.en[1]  = y_en;
    // convert position in (float)[mm]   to position in steps (int)[steps]
    // convert velocity in (float)[mm/s] to velocity in steps (int)[steps/s]
    x_step_zad = convert_position_to_steps( IN_motor_status.pos[0] );
    x_v_zad    = IN_motor_status.vel[0];
    x_en       = IN_motor_status.en[0];

    y_step_zad = convert_position_to_steps( IN_motor_status.pos[1] );
    y_v_zad    = IN_motor_status.vel[1];
    y_en       = IN_motor_status.en[1];

    // Set direction based on desired position
    x_dir_zad = set_motor_direction_x( x_step_zad, x_step_akt );
    y_dir_zad = set_motor_direction_y( y_step_zad, y_step_akt );

    // EN bit: if high motor inactive
    //         if low  motor active
    motor_start_stop_x( x_en );
    motor_start_stop_y( y_en );

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
