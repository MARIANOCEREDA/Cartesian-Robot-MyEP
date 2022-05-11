/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

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
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//Inicialización de variables
	start = 2;
	v_off = 2;
	homing_on = 0;
	state = measure_dist; //estado inicial

	//Variables spi
	indice_spi = 0;

	//Variables UART
	indice_uart = 0;

	//Variables para motores
	steps_mot1 = 0 ,prev_steps_mot1 = 0,new_steps_mot1 = 0;
	mot2_dir = 0;
	pwm_mot1 = 0;
	pwm_mot2 = 0;
	ARR_aux = 700; //VALOR DE ARR POR DEFECTO
	pulse = 499;
	ARR_max = 600;

	//Uso fines de carrera
	fdc = 1;

	//Enable de motores
	en_m1 = 0;
	en_m1 = 0;

	float aux_measure = 0;

	 //Variables para uso de encoder
	encoder_count = 0;
	prev_encoder_count = 0;
	measured_steps = 0;

	//Variables para el uso de Input capture - sensor de distancia
	IC_Val1 = 0;
	IC_Val2 = 0;
	difference = 0.0;
	Is_First_Captured = 0;
	posx = 0.0;
	prev_posx = 1.0;
	measured_pos = 0.0;

	//Para manejo de servomotor
	uint8_t dcycle = 0;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  //START INPUT CAPTURE PARA USO DE SENSOR DISTANCIA
  HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_4);

  //VARIABLE PARA CONTROL DE MOTORES
  uint16_t steps = 0;

  //START PWM PARA SERVO
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
  __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_4,0);

  //ACTIVAR PIN SS - COM SPI - EN HIGH DESACTIVA EL ESCLAVO
  HAL_GPIO_WritePin(ss_spi_GPIO_Port,ss_spi_Pin,SET);
  HAL_SPI_Receive_IT(&hspi1,&rec_data_spi[0],1);

  //RECEPCION DE UART USANDO INTERRPUCIÓN
   HAL_UART_Receive_IT(&huart2,uart_data, 1);

   //INICIALIZAMOS EL ENCODER MODE
   HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);

   //Deshabilitamos interrupciones externas de fines de carrera para comenzar y limpiamos flags.
   HAL_NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
   HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("#--------------------------------#\r\n");
  printf( "Comandos:\r\n"
		  ":on; -> Start: Comienza el programa\r\n"
		  ":off; -> End: Finaliza el programa\r\n"
		  ":color; -> Color: Devuelve el color del objeto detectado\r\n"
		  ":vX.X; -> X.X = [1,3]ms Configura duracion del pulso mas corto (maxima velocidad) del motor \r\n"
  	  	  ":restart; -> Reinicio del programa despues de haber activado parada de emeregencia \r\n");
  printf("#--------------------------------# \r\n");

  //DEFINICION DE ALGUNAS VARIABLES DE INICIALIZACION

  while (1)
  {
	switch (start){
		case 1:
			switch(state){
			case measure_dist:
				Generate_trig_pulses();
				Generate_trig_pulses();
				HAL_Delay(300);
				if(measured_pos == aux_measure){
					if(measured_pos < 26 && measured_pos >9){
						printf("Posicion de objeto detectado: %4.2lf cm \r\n",measured_pos);
						printf("#--------------------------------# \r\n");
						mot2_dir = 0;
						state = move_m1;
					}
				}
				aux_measure = measured_pos;
				break;
			case move_m1: // Inicializa el pwm que mueve al motor 1. La inicialización está dentro de move_motor1
				//SPI_com();
				steps = Calculate_steps(posx); //distancia en x*10/hilo del tornillo * cantidad de pasos para una vuelta.
				Move_motor1(steps);
				state = stop_m1;
				break;
			case stop_m1: // Detiene al motor 1 cuando se cumpla la condición
				if (measured_steps >= steps_mot1){
					__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1,0);
					HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_1);
					printf("ms: %d \r\n",measured_steps);
					prev_posx = posx;
					prev_encoder_count = encoder_count;
					prev_steps_mot1 = steps;
					pwm_mot1 = 0;
					ARR_aux = 0;
					measured_steps = 0;
					if (homing_on == 0){
						state = move_m2;
					}else{
						start = 2;
						homing_on = 0;
						printf("Hasta luego ! \r\n");
						printf("#--------------------------------# \r\n");
						printf("Ingrese :on; para volver a comenzar \r\n");
						state = measure_dist;
					}
				}
				break;

			case move_m2: // Inicializa el pwm que mueve al motor 2. La inicialización está dentro de move_motor2
				Move_motor2(mot2_dir);
				HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
				state = stop_m2;
				break;

			case stop_m2: // Detendrá el motor 2 cuando se cumpla la condición
				if (fdc == 0){
					pwm_mot2=0;
					HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
					__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,0);
					HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_2);
					fdc = 1;
					if (mot2_dir == 0){
						state = close_grip;
						HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4); // Inicializamos pwm para servo
					}else{
						state = open_grip;
						HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4); // Inicializamos pwm para servo
					}
				}
				break;

			case close_grip: // Cierra la pinza, modificando el dcycle del pwm
				dcycle++;
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,dcycle);
				HAL_Delay(5);
				if(dcycle == MAX_VALUE_Gripper){
					HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_4);
					printf("Pinza cerrada \r\n");
					state = com_spi;
				}
				break;

			case com_spi: // comunicación spi con arduino
				SPI_com();
				SPI_com();
				Print_color();
				state = clasify_obj;
				break;

			case clasify_obj: // le asigna una posición al objeto segun el color, dentro de la función clasify_obj
				Clasify_object();
				mot2_dir = 1;
				state = move_m1;
				break;

			case open_grip: // Abre la pinza, modificando el dcycle
				dcycle--;
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,dcycle);
				 HAL_Delay(5);
				if (dcycle == MIN_VALUE_Gripper){
					HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_4);
					printf("Pinza abierta \r\n");
					state = measure_dist;
					aux_measure = 0;
					if (v_off == 0){
						state = off;
					}
				}
				break;

			case off: // "Apaga" el programa. En realidad lo deja en la posición de homing para futuros usos, ya que no poseemos fines de carrera.
				homing_on = 1;
				posx = 8;
				v_off = 2;
				state = move_m1;
				break;

			case emergency_stop: // Detiene el programa en cualquier lugar donde se encuentre
				start = 0;
				Execute_emergency_stop();
				printf("Para reiniciar el programa, ingrese :restart; \r\n");
				break;

			case homing: // Se llama a este estado
				mot2_dir = 1;
				posx = 8;
				state = move_m1;
				break;

			default:
				break;
			}
			break;
		case 3:
			start = 1;
			break;
		default:
			break;
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Execute_emergency_stop(){
	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_1); //Apagamos los PWM de motores
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,0);
	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,0);
	HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_4); //Apagamos PWM de servo
	pwm_mot1 = 0;
	pwm_mot2 = 0;
	en_m1 = 1; //Deshabilitamos los motores
	en_m2 = 1;
}

void Generate_trig_pulses(){
	//Genera el pulso necesario para el sensor de distancia
	HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, RESET);
	HAL_Delay(0.04);
	HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, SET);
	HAL_Delay(0.01);
	HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, RESET);
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC4);
}

uint16_t Calculate_steps(float pos){
	//Convierte valores de distancia en cm, a pasos para que realice el motor
	uint16_t steps = 0;
	steps = (pos*10/8)*200 - (9*10/8)*200; // restamos la distancia de 9cm, que es la distancia minima entre sensor y eje.
	return steps;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	/* Detecta los flancos generados por el sensor de distancia. Cuando se genera el pulso, se genera un flanco y
	Is_first_captured valdrá 1. Luego, cuando el pulso vuelve hacia el sensor, se detecta otro flanco y medimos la diferencia
	de tiempo en que fueron capturados ambos flancos.Eso será proporcional a la distancia medida*/

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)  // Si el canal del timer es el 4
	{
		if (Is_First_Captured==0) // Si el primer valor no ha sido capturado
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4); // lee el primer valor
			Is_First_Captured = 1;  //Setea a 1 la variable
			// Cambiamos la polaridad a "falling edge"
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured==1)   //Si el primer valor ya fue capturado
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);  //Leemos el segundo valor
			__HAL_TIM_SET_COUNTER(htim, 0);  //Reseteamos el contador

			if (IC_Val2 > IC_Val1){
				difference = IC_Val2-IC_Val1;
			}else if (IC_Val1 > IC_Val2){
				difference = (0xffff - IC_Val1) + IC_Val2;
			}

			posx = difference * 0.034/2;
			posx = roundf(posx*10)/10;
			measured_pos = posx;
			Is_First_Captured = 0;


			//Seteamos polaridad en flanco de subida
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC4);
		}
	}
}

void Move_motor1(uint16_t steps){
	//Motor 1 -> Se mueve la cantidad de pasos aclarados y luego corrige
	steps_mot1 = steps;
	if(steps_mot1 > prev_steps_mot1){
		HAL_GPIO_WritePin(dir_mot1_GPIO_Port,dir_mot1_Pin,SET);
		steps_mot1 = steps_mot1 - prev_steps_mot1;
	}else if (steps_mot1 < prev_steps_mot1){
		HAL_GPIO_WritePin(dir_mot1_GPIO_Port, dir_mot1_Pin,RESET);
		steps_mot1 = prev_steps_mot1 - steps_mot1;
	}

	printf("Pasos a realizar: %ld \r\n",steps_mot1);
	if(HAL_GPIO_ReadPin(dir_mot1_GPIO_Port, dir_mot1_Pin) == 1){
		printf("Direccion: Adelante \r\n");
		printf("#--------------------------------# \r\n");
	}else{
		printf("Direccion: Atras \r\n");
		printf("#--------------------------------# \r\n");
	}

	//Inicializamos los pulsos con PWM
	if(steps_mot1 != 0){
		ARR_aux = ARR_max+150;
		__HAL_TIM_SetAutoreload(&htim4,ARR_aux);
		pulse = ((ARR_aux + 1) * 50) / 100-1;
		HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_1);
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1,pulse);
	}
}

void Move_motor2(uint8_t dir){
	//Motor 2 -> se mueve en la direccion de dir (0 o 1) hasta que se active algun fin de carrera.
	ARR_aux = ARR_max;
	__HAL_TIM_SetAutoreload(&htim4,ARR_aux);
	pulse = ((ARR_aux + 1) * 50) / 100-1; //Duty cycle en 50%
	HAL_Delay(200);
	HAL_GPIO_WritePin(dir_mot2_GPIO_Port,dir_mot2_Pin,dir);
	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,pulse);
	//HAL_Delay(1000);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn); // habilitamos interrupciones para que detengan al motor cuando se active el fdc
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) //Rutina de interrupción por fin de carrera.
{
	//Se activan cuando lo hace el fin de carrera correspondiente, que hace que se detenga el motor 2
  if(GPIO_Pin == fdc_2_Pin){
	  if(HAL_GPIO_ReadPin(fdc_2_GPIO_Port,fdc_2_Pin) == 1){
		  printf("Fin de carrera 2 detectado - stop motor 2\r\n");
		  fdc = 0;
	  }
  }else if(GPIO_Pin == fdc_1_Pin){
	  if(HAL_GPIO_ReadPin(fdc_1_GPIO_Port,fdc_1_Pin) == 1){
		  printf("Fin de carrera 1 detectado - stop motor 2 \r\n");
		  fdc = 0;
	  }
  }
  if(GPIO_Pin == emerg_stop_Pin){
	  if(HAL_GPIO_ReadPin(emerg_stop_GPIO_Port,emerg_stop_Pin) == 1){
		  state = emergency_stop;
	  }
  }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	//Cada vez que se genera un pulso por pwm, se ingresa a esta funcion que cuenta dichos pulsos.
	if (htim -> Instance == htim4.Instance){
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
			pwm_mot1++;  //Cada vez que se genera un pulso por pwm (un paso del motor) suma uno en esta variable
			encoder_count =  TIM3->CNT;
			measured_steps = fabs(encoder_count*2.5 - prev_encoder_count*2.5);
			//Rampa de acelereación
			if(pwm_mot1<=steps_mot1){
				ARR_aux =ARR_aux - (2*ARR_aux)/(4*(steps_mot1-measured_steps)+1);
				if(ARR_aux < ARR_max){
					ARR_aux = ARR_max;
				}
			}
		}else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
			pwm_mot2++;
		}
	}
	__HAL_TIM_SetAutoreload(&htim4,ARR_aux); //Se carga el auto reload register para cambiar la nueva frecuencia del pwm
	pulse = ((ARR_aux + 1) * 50) / 100-1; // cambiamos pulse, para que el dc siempre sea del 50%
}


void Clasify_object(){
	//Asigna la posicion en x a la cual debe ser llevada el objeto , segun el color del mismo.
	switch(color){
	case 1: //Naranja y Rojo se los clasifica como iguales
	case 2:
		posx = 9;
		break;
	case 3:
	case 4: // verde y amarillo se los clasifica como iguales
		posx = 17;
		break;
	case 5:
	case 6:
	case 7:	//Azul,cyan y magenta los clasificamos como lo mismo
		posx = 23;
		break;
	default:
		posx = 9;
		break;
	}
}

void Print_color(){
	switch(color){
	case 1:
		printf("Color detectado: Rojo \r\n");
		break;
	case 2:
		printf("Color detectado: Naranja \r\n");
		break;
	case 3:
		printf("Color detectado: Amarillo \r\n");
		break;
	case 4:
		printf("Color detectado: Verde \r\n");
		break;
	case 5:
		printf("Color detectado: Cyan \r\n");
		break;
	case 6:
		printf("Color detectado: Azul \r\n");
		break;
	case 7:
		printf("Color detectado: Magenta \r\n");
		break;
	default:
		break;
	}
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
