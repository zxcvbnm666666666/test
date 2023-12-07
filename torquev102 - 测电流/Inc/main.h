/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define TOR_SEL1_Pin GPIO_PIN_4
#define TOR_SEL1_GPIO_Port GPIOC
#define TOR_SEL0_Pin GPIO_PIN_5
#define TOR_SEL0_GPIO_Port GPIOC
#define KLED_DIN_Pin GPIO_PIN_12
#define KLED_DIN_GPIO_Port GPIOB
#define KLED_SCK_Pin GPIO_PIN_13
#define KLED_SCK_GPIO_Port GPIOB
#define KLED_CS_Pin GPIO_PIN_14
#define KLED_CS_GPIO_Port GPIOB
#define KLED_EN_Pin GPIO_PIN_15
#define KLED_EN_GPIO_Port GPIOB
#define SLED0_Pin GPIO_PIN_8
#define SLED0_GPIO_Port GPIOC
#define SLED1_Pin GPIO_PIN_9
#define SLED1_GPIO_Port GPIOC
#define H6PWR_EN_Pin GPIO_PIN_15
#define H6PWR_EN_GPIO_Port GPIOA
#define ENCODER_PWR_Pin GPIO_PIN_10
#define ENCODER_PWR_GPIO_Port GPIOC
#define TORQUE_PWR_Pin GPIO_PIN_11
#define TORQUE_PWR_GPIO_Port GPIOC
#define TIMY_CH1_Pin GPIO_PIN_4
#define TIMY_CH1_GPIO_Port GPIOB
#define TIMY_CH1_EXTI_IRQn EXTI4_IRQn
#define TIMY_CH2_Pin GPIO_PIN_5
#define TIMY_CH2_GPIO_Port GPIOB
#define TIMX_CH1_Pin GPIO_PIN_8
#define TIMX_CH1_GPIO_Port GPIOB
#define TIMX_CH1_EXTI_IRQn EXTI9_5_IRQn
#define TIMX_CH2_Pin GPIO_PIN_9
#define TIMX_CH2_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
struct TorqueSensor
{
	uint16_t PulsCount;  			//计数脉冲
	uint16_t TorqueValue;			//扭矩电压
	uint16_t Verfer;					//
};
extern struct TorqueSensor Sensor[8];

extern uint16_t FrequencyEncoder;  //单位Hz
extern uint16_t FrequencyTorque;		//单位Hz

extern unsigned char RsPoint_USART3;	//数据计数
extern unsigned char RsBuf_USART3[64];  				//接收缓冲区
extern unsigned char Received_USART3;      //接收标志

extern uint16_t	 IndexEncoder;
extern uint16_t  IndexTorque;

#define LED0_ON				HAL_GPIO_WritePin(SLED0_GPIO_Port, SLED0_Pin, GPIO_PIN_RESET);
#define LED0_OFF			HAL_GPIO_WritePin(SLED0_GPIO_Port, SLED0_Pin, GPIO_PIN_SET);
#define LED0_Flash		HAL_GPIO_TogglePin(SLED0_GPIO_Port, SLED0_Pin);

#define LED1_ON				HAL_GPIO_WritePin(SLED1_GPIO_Port, SLED1_Pin, GPIO_PIN_RESET);
#define LED1_OFF			HAL_GPIO_WritePin(SLED1_GPIO_Port, SLED1_Pin, GPIO_PIN_SET);
#define LED1_Flash		HAL_GPIO_TogglePin(SLED1_GPIO_Port, SLED1_Pin);

#define ENCODER_PWR_ON				HAL_GPIO_WritePin(ENCODER_PWR_GPIO_Port, ENCODER_PWR_Pin, GPIO_PIN_SET);
#define ENCODER_PWR_OFF			  HAL_GPIO_WritePin(ENCODER_PWR_GPIO_Port, ENCODER_PWR_Pin, GPIO_PIN_RESET);

#define TORQUE_PWR_ON				  HAL_GPIO_WritePin(TORQUE_PWR_GPIO_Port, TORQUE_PWR_Pin, GPIO_PIN_SET);
#define TORQUE_PWR_OFF			  HAL_GPIO_WritePin(TORQUE_PWR_GPIO_Port, TORQUE_PWR_Pin, GPIO_PIN_RESET);

#define H6PWR_EN_ON				  HAL_GPIO_WritePin(H6PWR_EN_GPIO_Port, H6PWR_EN_Pin, GPIO_PIN_SET);
#define H6PWR_EN_OFF			  HAL_GPIO_WritePin(H6PWR_EN_GPIO_Port, H6PWR_EN_Pin, GPIO_PIN_RESET);

#define TOR_SEL1_0				HAL_GPIO_WritePin(TOR_SEL1_GPIO_Port, TOR_SEL1_Pin, GPIO_PIN_RESET);
#define TOR_SEL1_1				HAL_GPIO_WritePin(TOR_SEL1_GPIO_Port, TOR_SEL1_Pin, GPIO_PIN_SET);
#define TOR_SEL0_0				HAL_GPIO_WritePin(TOR_SEL0_GPIO_Port, TOR_SEL0_Pin, GPIO_PIN_RESET);
#define TOR_SEL0_1				HAL_GPIO_WritePin(TOR_SEL0_GPIO_Port, TOR_SEL0_Pin, GPIO_PIN_SET);

#define TOR_SEL_0					TOR_SEL1_0;TOR_SEL0_0;
#define TOR_SEL_1					TOR_SEL1_0;TOR_SEL0_1;
#define TOR_SEL_2					TOR_SEL1_1;TOR_SEL0_0;
#define TOR_SEL_3					TOR_SEL1_1;TOR_SEL0_1;

#define ENCODER7_B		HAL_GPIO_ReadPin(TIMX_CH2_GPIO_Port,TIMX_CH2_Pin)
#define ENCODER8_B		HAL_GPIO_ReadPin(TIMY_CH2_GPIO_Port,TIMY_CH2_Pin)

void USART3_Putc(unsigned char c);
void USART3_Send(unsigned char * SendMsg,unsigned char Num);
void writeFlash(void);
void readFlash(void);
/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
