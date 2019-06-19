/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32f4xx_it.h"
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
extern int Velo_TimeUp;
extern int Velo_TimeFull;
extern int Velo_Up;
extern int Velo_DutyCycle;

extern int Rota_TimeUp;
extern int Rota_TimeFull;
extern int Rota_Up;
extern int Rota_DutyCycle;

extern int Limpa_TimeUp;
extern int Limpa_TimeFull;
extern int Limpa_Up;
extern int Limpa_DutyCycle;
extern int Limpa;

extern int START_TimeUp;
extern int START_TimeFull;
extern int START_Up;
extern int START_DutyCycle;
extern int START;

/*variaveis de velocidade e de rotacao*/
int velocidade = 0;
int rotacao = 0;
int velocidade_H = 0;
int velocidade_L = 0;
int rotacao_H = 0;
int rotacao_L = 0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
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
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */
	if(HAL_GPIO_ReadPin(GPIOC, START_Pin) == 1){
		//calculo dutycycle
		START_DutyCycle = START_TimeUp/START_TimeFull;
		START_TimeUp = 0;
		START_TimeFull = 1;
		START_Up = 1;
	}else{
		START_Up = 0;
	}
	if(START_DutyCycle > 35){
		START = 1;
	}else{
		START = 0;
	}
  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */
	if(HAL_GPIO_ReadPin(GPIOC, Motores_de_limpeza_Pin) == 1){
		//calculo dutycycle
		Limpa_DutyCycle = Limpa_TimeUp/Limpa_TimeFull;
		Limpa_TimeUp = 0;
		Limpa_TimeFull = 1;
		Limpa_Up = 1;
	}else{
		Limpa_Up = 0;
	}
	if(Limpa_DutyCycle > 35){
		Limpa = 1;
	}else{
		Limpa = 0;
	}
  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

	/*incrementos de tempos*/
	Velo_TimeFull++;
	Rota_TimeFull++;
	Limpa_TimeFull++;
	START_TimeUp++;
	if(Velo_Up)Velo_TimeUp++;
	if(Rota_Up)Rota_TimeUp++;
	if(Limpa_Up)Limpa_TimeUp++;
	if(START_Up)START_TimeUp++;

	/*FLUXO DO PROGRAMA*/
	if(START){
		HAL_USART_Transmit(husart1,0x82,0,0);//comando contorl do Roomba
		if(START){
			HAL_USART_Transmit(husart1,0x8A,0,0);//opcode do comando motors comand
			if(Limpa){
				HAL_USART_Transmit(husart1,0x07,0,0);//liga os motores de limpesa
			}else{
				HAL_USART_Transmit(husart1,0x00,0,0);//desliga os motores de limpesa
			}
			/*conversao dutycycle para binario */
			velocidade = ((Velo_DutyCycle-30)/20)*1000 - 500;
			rotacao = ((Rota_DutyCycle-30)/20)*4000 - 2000;
			velocidade_L = velocidade % 256;
			rotacao_L = rotacao % 256;
			velocidade_H = velocidade % 4096;
			rotacao_H = rotacao % 4096;
			if(velocidade < 0){
				velocidade_H = velocidade_H + 240;
			}
			if(rotacao < 0){
				rotacao_H = rotacao_H + 240;
			}
			HAL_USART_Transmit(husart1,137,0,0); //comando drive do Roomba
			/*envia a velocidade linear*/
			HAL_USART_Transmit(husart1,velocidade_H,0,0);
			HAL_USART_Transmit(husart1,velocidade_L,0,0);
			/*envia a rotação*/
			HAL_USART_Transmit(husart1,rotacao_H,0,0);
			HAL_USART_Transmit(husart1,rotacao_L,0,0);
		}else{
			HAL_USART_Transmit(husart1,0x85,0,0);//comando power do Roomba
		}
	}

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

	if( HAL_GPIO_ReadPin(GPIOC, Velocidade_Linear_Pin) == 1 ){
		//calculo dutycycle
		Velo_DutyCycle = Velo_TimeUp/Velo_TimeFull;
		Velo_TimeUp = 0;
		Velo_TimeFull = 1;
		Velo_Up = 1;
	}else{
		Velo_Up = 0;
	}
	if(HAL_GPIO_ReadPin(GPIOC, Rota__o_Pin) == 1){
		//calculo dutycycle
		Rota_DutyCycle = Rota_TimeUp/Rota_TimeFull;
		Rota_TimeUp = 0;
		Rota_TimeFull = 1;
		Rota_Up = 1;
	}else{
		Rota_Up = 0;
	}

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
