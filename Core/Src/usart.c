/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
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

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "spi.h"
#include "main.h"
/* USER CODE END 0 */

UART_HandleTypeDef huart2;

/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();
  
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  if(huart->Instance == USART2){
	 uint8_t dato = uart_data[0];
	 if(indice_uart>= SIZE_COM) indice_uart = 0;
	 switch(dato){
	 case ':':
		 indice_uart = 0;
		 break;
	 case '\r':
	 case ';': //Carácter de fin de trama.
		 command[indice_uart]=0;
		 Analyze_command();
		 break;
	 case 8:
		 if (indice_uart) indice_uart--;
		 break;
	 default:
		 command[indice_uart++]=dato;
		 break;
	 }
	 HAL_UART_Receive_IT(&huart2,uart_data, 1);
	}
}

void Analyze_command(){
	float aux = 0;
	uint16_t vel_rpm = 0;
	if(strstr(command,"on")){
		printf("Start code \r\n");
		printf("#--------------------------------# \r\n");
		start = 1;
	}else if(strstr(command,"off")){
		printf("Finish code \r\n");
		printf("#--------------------------------# \r\n");
		v_off = 0;
	}else if(strstr(command,"color")){
		printf("#--------------------------------# \r\n");
		Print_color();
		printf("Hora: %d \r\n",hour);
		printf("Minuto: %d \r\n",minute);
		printf("Segundo: %d \r\n",second);
		printf("#--------------------------------# \r\n");
	}else if(strstr(command,"restart")){
		start = 1;
		en_m1 = 0;
		en_m2 = 0;
		state = homing;
	}else{
		switch(command[0]){
		case 'v':
		case 'V':
			printf("in \r\n");
			if(command[1] != 0){ // si es distinto de 0
				aux = atof(&command[1]);		// Convierte cadena decimal en entero
				if(aux >=1 && aux <=3){ //aux es la velocidad en ms del pulso máximo
					ARR_max = round((500000/(1000/aux))-1); //Cambiamos máxima que puede tener el pwm
					printf("Periodo maximo: %d \r\n",ARR_max);
					printf("#--------------------------------# \r\n");
				}else{
					printf("Debe ingresar una velocidad valida, en el rango [1,5]ms \r\n");
					printf("#--------------------------------# \r\n");
				}
				vel_rpm = round(60/(aux/1000*200));
				printf("Velocidad maxima: %d rpm \r\n",vel_rpm);
				printf("#--------------------------------# \r\n");
				}
				break;
		default:
			break;
		}
	}
}


//Redefinimos la primitiva para usar printf
int __io_putchar(int ch){
	HAL_UART_Transmit(&huart2,(uint8_t*)&ch, 1,100);
	return ch;
}


/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
