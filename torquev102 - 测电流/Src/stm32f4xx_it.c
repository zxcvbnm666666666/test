/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
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
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* USER CODE BEGIN 0 */
#include "crc.h" 
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;
extern ADC_HandleTypeDef hadc1;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern UART_HandleTypeDef huart3;
//uint16_t VrefValue=0;
/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
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
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
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
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
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
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
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
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
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
	static uint16_t count; 
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */
	if (++count>=1000){
		LED0_Flash;
		count=0;
	}
  /* USER CODE END SysTick_IRQn 1 */
}
void delay(uint16_t num)
{
  uint16_t i,j;
  for(i=0;i<num;i++)
    for(j=0;j<0x800;j++);
}
/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line4 interrupt.
*/
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
* @brief This function handles EXTI line[9:5] interrupts.
*/
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
* @brief This function handles USART3 global interrupt.
*/
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
	unsigned char temp;
	if(USART3 == huart3.Instance)
	{     
		if(RESET != __HAL_UART_GET_FLAG(&huart3,UART_FLAG_PE))
		{
				temp=USART3->DR;	
				__HAL_UART_CLEAR_PEFLAG(&huart3); 
		}
		if(RESET != __HAL_UART_GET_FLAG(&huart3,UART_FLAG_FE))
		{
				temp=USART3->DR;
				__HAL_UART_CLEAR_FEFLAG(&huart3); 
		}

		if(RESET != __HAL_UART_GET_FLAG(&huart3,UART_FLAG_ORE))
		{
				__HAL_UART_CLEAR_OREFLAG(&huart3); 
				temp=USART3->DR;
		}
		
		if(RESET != __HAL_UART_GET_FLAG(&huart3,UART_FLAG_RXNE)){
				temp=USART3->DR;
				RsBuf_USART3[RsPoint_USART3] = temp; 
				RsPoint_USART3=(RsPoint_USART3+1)%64;
		}
		if(RESET != __HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE)){
			__HAL_UART_CLEAR_IDLEFLAG(&huart3);
			Received_USART3=1; 
		}
	}
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
* @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
*/
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
	unsigned char SendMsg[23]={0x55,0xAA,0x01};
	uint16_t CrcCode;
  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */
	
	//TIM6用于控制编码器数据采样及上传
	Sensor[0].PulsCount = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim1));//获取定时器的值
	__HAL_TIM_SET_COUNTER(&htim1,0);
	Sensor[1].PulsCount = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim2));//获取定时器的值
	__HAL_TIM_SET_COUNTER(&htim2,0);
	Sensor[2].PulsCount = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim3));//获取定时器的值
	__HAL_TIM_SET_COUNTER(&htim3,0);
	Sensor[3].PulsCount = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim4));//获取定时器的值
	__HAL_TIM_SET_COUNTER(&htim4,0);
	Sensor[4].PulsCount = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim5));//获取定时器的值
	__HAL_TIM_SET_COUNTER(&htim5,0);
	Sensor[5].PulsCount = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim8));//获取定时器的值
	__HAL_TIM_SET_COUNTER(&htim8,0);
	
	if (Received_USART3==0){
		//发送编码器数据， 55 AA 0x01 2byte*8 CRC16，共23byte
		SendMsg[3]=Sensor[0].PulsCount>>8;
		SendMsg[4]=Sensor[0].PulsCount;
		SendMsg[5]=Sensor[1].PulsCount>>8;
		SendMsg[6]=Sensor[1].PulsCount;
		SendMsg[7]=Sensor[2].PulsCount>>8;
		SendMsg[8]=Sensor[2].PulsCount;
		SendMsg[9]=Sensor[3].PulsCount>>8;
		SendMsg[10]=Sensor[3].PulsCount;
		SendMsg[11]=Sensor[4].PulsCount>>8;
		SendMsg[12]=Sensor[4].PulsCount;
		SendMsg[13]=Sensor[5].PulsCount>>8;
		SendMsg[14]=Sensor[5].PulsCount;
		SendMsg[15]=Sensor[6].PulsCount>>8;
		SendMsg[16]=Sensor[6].PulsCount;
		SendMsg[17]=Sensor[7].PulsCount>>8;
		SendMsg[18]=Sensor[7].PulsCount;
		CrcCode=Get_Crc16(&SendMsg[2],17);
		SendMsg[19]=CrcCode>>8;
		SendMsg[20]=CrcCode;
		SendMsg[21]=IndexEncoder>>8;
		SendMsg[22]=IndexEncoder;
		//USART3_Send(SendMsg,23); 
		IndexEncoder++;
	}
	
	Sensor[6].PulsCount=0;
	Sensor[7].PulsCount=0;
  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
* @brief This function handles TIM7 global interrupt.
*/
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */
	unsigned char SendMsg[23]={0x55,0xAA,0x02};
	uint16_t CrcCode;
	
	uint8_t i;
  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

	//TIM7用于控制扭矩采样及上传
	//采样ch1、ch5
	TOR_SEL_0;
	delay(50);
	HAL_ADC_Start(&hadc1);
	while(HAL_ADC_PollForConversion(&hadc1,0xffff)!=HAL_OK);//等待ADC转换完成
	Sensor[0].TorqueValue=HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,0xffff);//等待ADC转换完成
	Sensor[4].TorqueValue=HAL_ADC_GetValue(&hadc1);
//		//取参考电压
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,0xffff);//等待ADC转换完成
	Sensor[4].Verfer=HAL_ADC_GetValue(&hadc1);
	Sensor[0].Verfer = Sensor[4].Verfer;
	HAL_ADC_Stop(&hadc1);
	
	//采样ch2、ch6
	TOR_SEL_1;
	delay(50);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,0xffff);//等待ADC转换完成
	Sensor[1].TorqueValue=HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,0xffff);//等待ADC转换完成
	Sensor[5].TorqueValue=HAL_ADC_GetValue(&hadc1);
////		//取参考电压
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,0xffff);//等待ADC转换完成
	Sensor[5].Verfer=HAL_ADC_GetValue(&hadc1);
	Sensor[1].Verfer = Sensor[5].Verfer;
	HAL_ADC_Stop(&hadc1);
//	
//	//采样ch3、ch7
	TOR_SEL_2;
	delay(50);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,0xffff);//等待ADC转换完成
	Sensor[2].TorqueValue=HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,0xffff);//等待ADC转换完成
	Sensor[6].TorqueValue=HAL_ADC_GetValue(&hadc1);
////		//取参考电压
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,0xffff);//等待ADC转换完成
	Sensor[6].Verfer=HAL_ADC_GetValue(&hadc1);
	Sensor[2].Verfer = Sensor[6].Verfer;
	HAL_ADC_Stop(&hadc1);
//	
//	//采样ch4、ch8
	TOR_SEL_3;
	delay(50);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,0xffff);//等待ADC转换完成
	Sensor[3].TorqueValue=HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,0xffff);//等待ADC转换完成
	Sensor[7].TorqueValue=HAL_ADC_GetValue(&hadc1);
//	
//	//取参考电压
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,0xffff);//等待ADC转换完成
	Sensor[7].Verfer=HAL_ADC_GetValue(&hadc1);
	Sensor[3].Verfer = Sensor[7].Verfer;
	HAL_ADC_Stop(&hadc1);
	
	//电压处理
	for (i=0;i<8;i++){
		Sensor[i].TorqueValue=(unsigned int)((float)Sensor[i].TorqueValue/Sensor[i].Verfer*2500);
	}
	
	
	if (Received_USART3==0){
		//发送扭矩数据， 55 AA 0x02 2byte*8 CRC16，共21byte
		SendMsg[3]=Sensor[0].TorqueValue>>8;
		SendMsg[4]=Sensor[0].TorqueValue;
		SendMsg[5]=Sensor[1].TorqueValue>>8;
		SendMsg[6]=Sensor[1].TorqueValue;
		SendMsg[7]=Sensor[2].TorqueValue>>8;
		SendMsg[8]=Sensor[2].TorqueValue;
		SendMsg[9]=Sensor[3].TorqueValue>>8;
		SendMsg[10]=Sensor[3].TorqueValue;
		SendMsg[11]=Sensor[4].TorqueValue>>8;
		SendMsg[12]=Sensor[4].TorqueValue;
		SendMsg[13]=Sensor[5].TorqueValue>>8;
		SendMsg[14]=Sensor[5].TorqueValue;
		SendMsg[15]=Sensor[6].TorqueValue>>8;
		SendMsg[16]=Sensor[6].TorqueValue;
		SendMsg[17]=Sensor[7].TorqueValue>>8;
		SendMsg[18]=Sensor[7].TorqueValue;
		CrcCode=Get_Crc16(&SendMsg[2],17);
		SendMsg[19]=CrcCode>>8;
		SendMsg[20]=CrcCode;
		SendMsg[21]=IndexTorque>>8;
		SendMsg[22]=IndexTorque;
		USART3_Send(SendMsg,23);
		IndexTorque++;
	}
  /* USER CODE END TIM7_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
