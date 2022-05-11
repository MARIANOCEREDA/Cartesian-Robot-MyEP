/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "string.h"
#include "stdio.h"
#include "tim.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
void Execute_emergency_stop();
void Generate_trig_pulses();
void Move_motor1(uint16_t);
void Move_motor2(uint8_t);
//void Open_gripper();
//void Close_gripper();
void SPI_com();
void Clasify_object();
void Analyze_command();
void Print_color();
uint16_t Calculate_steps(float);
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */


/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

volatile enum st {measure_dist,move_m1,stop_m1,stop_m2,move_m2,com_spi,open_grip,close_grip,clasify_obj,off,emergency_stop,homing};
enum st state;
enum st prev_state;
uint8_t start;
uint8_t v_off;
uint8_t homing_on;
uint32_t ARR_max;

//Variables para manejo de motores
int32_t steps_mot1,prev_steps_mot1,new_steps_mot1;
uint8_t mot2_dir;
int32_t pwm_mot1;
int32_t pwm_mot2;
volatile uint32_t ARR_aux; //VALOR DE ARR POR DEFECTO
volatile uint32_t pulse;

//Uso fines de carrera
uint8_t fdc;

//Enable de motores
uint8_t en_m1,en_m2;

//Variables para uso de encoder
int16_t encoder_count,prev_encoder_count;
int16_t measured_steps;

//Variables para el uso de Input capture - sensor de distancia
 uint32_t IC_Val1;
 uint32_t IC_Val2;
 float difference;
 uint8_t Is_First_Captured;  // is the first value captured ?
 float posx,prev_posx,measured_pos;

 //Variables para uso de servomotor
#define MAX_VALUE_Gripper 249
#define MIN_VALUE_Gripper 0

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define emerg_stop_Pin GPIO_PIN_0
#define emerg_stop_GPIO_Port GPIOA
#define emerg_stop_EXTI_IRQn EXTI0_IRQn
#define trig_Pin GPIO_PIN_1
#define trig_GPIO_Port GPIOA
#define ss_spi_Pin GPIO_PIN_0
#define ss_spi_GPIO_Port GPIOB
#define fdc_1_Pin GPIO_PIN_13
#define fdc_1_GPIO_Port GPIOB
#define fdc_1_EXTI_IRQn EXTI15_10_IRQn
#define fdc_2_Pin GPIO_PIN_14
#define fdc_2_GPIO_Port GPIOB
#define fdc_2_EXTI_IRQn EXTI15_10_IRQn
#define en_m1_Pin GPIO_PIN_15
#define en_m1_GPIO_Port GPIOB
#define en_m2_Pin GPIO_PIN_8
#define en_m2_GPIO_Port GPIOA
#define dir_mot2_Pin GPIO_PIN_9
#define dir_mot2_GPIO_Port GPIOA
#define dir_mot1_Pin GPIO_PIN_12
#define dir_mot1_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
