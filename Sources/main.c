/**
	******************************************************************************
	* @file    UART/UART_TwoBoards_ComIT/Src/main.c 
	* @author  MCD Application Team
	* @brief   This sample code shows how to use UART HAL API to transmit
	*          and receive a data buffer with a communication process based on
	*          IT transfer. 
	*          The communication is done using 2 Boards.
	******************************************************************************
	* @attention
	*
	* <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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
#include "main.h"
#include "log.h"
#include "cmsis_os.h"
#include "stm32f7xx_it.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/** @addtogroup STM32F7xx_HAL_Examples
	* @{
	*/

/** @addtogroup UART_TwoBoards_ComIT
	* @{
	*/ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define I2C_TIMING              0x40912732 //0x00303D5D; 0x00A0689A
#define STM32F7_ADDR 			0xD0
#define UPDATE_INTERVAL 		15 //refresh rate: 1/0.015ms = 66Hz
#define TASK_INTERVAL			5000
/* Private variables ---------------------------------------------------------*/
__IO uint32_t UserButtonStatus = 0;  /* set to 1 after User Button interrupt  */


osThreadId led_ring_ThreadId;
osThreadId i2c_slave_ThreadId;

uint8_t aTxBuffer[] = "Hello Olli";
uint8_t aRxBuffer[RXBUFFERSIZE];
I2C_HandleTypeDef I2C1_Handle;
I2C_HandleTypeDef I2C4_Handle;
CY8CMBR3116_Result result;

osTimerId  xTimerUpdate;
osThreadId MainHandler;
osThreadId xCircularRingHandler;
osThreadId xHeartBeatHandler;
osThreadId xAllColorsHandler;
osThreadId xColorWheelHandler;
osThreadId xPatternMoveHandler;
osThreadId xFullEmptyHandler;
osThreadId xAlternateColorsHandler;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Error_Handler(void);
static void CPU_CACHE_Enable(void);
void I2C1_Init(void);
void CPU_I2C4_Init(void);

static void led_control_Thread(void const * argument);
static void CircularRing_Task(void const * argument);
static void HeartBeat_Task(void const * argument);
static void AllColors_Task(void const * argument);
static void ColorWheel_Task(void const * argument);
static void PatternMove_Task(void const * argument);
static void FullEmpty_Task(void const * argument);
static void AlternateColors_Task(void const * argument);

/* Private functions ---------------------------------------------------------*/
/**
	* @brief  Main program
	* @param  None
	* @retval None
	*/
int main(void)
{
	/* Enable the CPU Cache */
	CPU_CACHE_Enable();
	HAL_Init();

	/* Configure the system clock to 216 MHz */
	SystemClock_Config();
	/* Configure LED1 */
	BSP_LED_Init(LED1);

	/* Configure logs */
	log_init();
	UART3_Init();

	/* Configure LED RING */
	ws281x_init();

	/* Configure I2C bus */
	I2C1_Init();
	CPU_I2C4_Init();

	/* Configure EXTI Line0 (connected to PB0 pin) in interrupt mode */
	MBR3_HOST_INT_Config();

	/* Configure MBR3 */
	result = ConfigureMBR3(&I2C1_Handle);
	if(result != CY8CMBR3116_Result_OK)
	{
		DEBUG_ERROR("Configure MBR3");
	}
	/* Create threads */
	osThreadDef(led_control, led_control_Thread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);

	osThreadDef(HeartBeat, HeartBeat_Task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
	osThreadDef(CircularRing, CircularRing_Task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
	osThreadDef(AllColors, AllColors_Task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
	osThreadDef(ColorWheel, ColorWheel_Task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
	osThreadDef(PatternMove, PatternMove_Task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
	osThreadDef(FullEmpty, FullEmpty_Task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
	osThreadDef(AlternateColors, AlternateColors_Task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);

	xHeartBeatHandler = osThreadCreate(osThread(HeartBeat), NULL);
	xCircularRingHandler = osThreadCreate(osThread(CircularRing), NULL);
	xAllColorsHandler = osThreadCreate(osThread(AllColors), NULL);
	xColorWheelHandler = osThreadCreate(osThread(ColorWheel), NULL);
	xPatternMoveHandler = osThreadCreate(osThread(PatternMove), NULL);
	xFullEmptyHandler = osThreadCreate(osThread(FullEmpty), NULL);
	xAlternateColorsHandler = osThreadCreate(osThread(AlternateColors), NULL);
	MainHandler = osThreadCreate(osThread(led_control), NULL);

	osThreadSuspend(xHeartBeatHandler);
	osThreadSuspend(xCircularRingHandler);
	osThreadSuspend(xAllColorsHandler);
	osThreadSuspend(xColorWheelHandler);
	osThreadSuspend(xPatternMoveHandler);
	osThreadSuspend(xFullEmptyHandler);
	osThreadSuspend(xAlternateColorsHandler);

	/* Send hello message through uart */
	DEBUG_PRINT("Starting.......!");

	/* Start scheduler */
	osKernelStart();

	/* We should never
	 get here as control is now taken by the scheduler */
	for(;;);

}

static void led_control_Thread(void const * argument) {
	(void) argument;
	uint32_t PreviousWakeTime = osKernelSysTick();

	for(;;)
	{

		osThreadResume(xHeartBeatHandler);
		osDelay(15000);
		osThreadSuspend(xHeartBeatHandler);

		osThreadResume(xCircularRingHandler);
		osDelay(TASK_INTERVAL);
		osThreadSuspend(xCircularRingHandler);

		osThreadResume(xAllColorsHandler);
		osDelay(10000);
		osThreadSuspend(xAllColorsHandler);

		osThreadResume(xColorWheelHandler);
		osDelay(TASK_INTERVAL);
		osThreadSuspend(xColorWheelHandler);

		osThreadResume(xPatternMoveHandler);
		osDelay(TASK_INTERVAL);
		osThreadSuspend(xPatternMoveHandler);

		osThreadResume(xFullEmptyHandler);
		osDelay(TASK_INTERVAL);
		osThreadSuspend(xFullEmptyHandler);

		osThreadResume(xAlternateColorsHandler);
		osDelay(TASK_INTERVAL);
		osThreadSuspend(xAlternateColorsHandler);
	}
}

static void CircularRing_Task(void const * argument) {
	stripEffect_CircularRing(50, 0, 0, 20);
}

static void HeartBeat_Task(void const * argument) {
	stripEffect_HeartBeat(150, 250, 0, 0);
}

static void AllColors_Task(void const * argument) {
	stripEffect_AllColors(10);
}

static void ColorWheel_Task(void const * argument) {
	stripEffect_ColorWheel(50);
}

static void PatternMove_Task(void const * argument) {
	stripEffect_PatternMove(50, 2, 10, 10, 10);
}

static void FullEmpty_Task(void const * argument) {
	stripEffect_FullEmpty(50, 20, 20, 20);
}

static void AlternateColors_Task(void const * argument) {
	stripEffect_AlternateColors(1000, 10, 50, 0, 0, 0, 0, 50);
}

/**
	* @brief  System Clock Configuration
	*         The system Clock is configured as follow : 
	*            System Clock source            = PLL (HSE)
	*            SYSCLK(Hz)                     = 216000000
	*            HCLK(Hz)                       = 216000000
	*            AHB Prescaler                  = 1
	*            APB1 Prescaler                 = 4
	*            APB2 Prescaler                 = 2
	*            HSE Frequency(Hz)              = 25000000
	*            PLL_M                          = 25
	*            PLL_N                          = 432
	*            PLL_P                          = 2
	*            PLL_Q                          = 9
	*            VDD(V)                         = 3.3
	*            Main regulator output voltage  = Scale1 mode
	*            Flash Latency(WS)              = 7
	* @param  None
	* @retval None
	*/
void SystemClock_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;
	HAL_StatusTypeDef ret = HAL_OK;

	/* Enable HSE Oscillator and activate PLL with HSE as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 432;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 9;
	
	ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
	if(ret != HAL_OK)
	{
		while(1) { ; }
	}
	
	/* Activate the OverDrive to reach the 216 MHz Frequency */  
	ret = HAL_PWREx_EnableOverDrive();
	if(ret != HAL_OK)
	{
		while(1) { ; }
	}

	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2; 

	ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);
	if(ret != HAL_OK)
	{
		while(1) { ; }
	}  
}
/**
  * @brief  This function configure I2C1 bus.
  * @param  None
  * @retval None
  */
void I2C1_Init(void)
{
    /*##Configure the I2C peripheral ######################################*/
    I2C1_Handle.Instance              = I2Cx_MASTER;
    I2C1_Handle.Init.Timing           = I2C_TIMING;
    I2C1_Handle.Init.OwnAddress1      = 0;
    I2C1_Handle.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    I2C1_Handle.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    I2C1_Handle.Init.OwnAddress2      = 0;
    I2C1_Handle.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    I2C1_Handle.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

    if(HAL_I2C_Init(&I2C1_Handle) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }

    /* Enable the Analog I2C Filter */
    HAL_I2CEx_ConfigAnalogFilter(&I2C1_Handle, I2C_ANALOGFILTER_ENABLE);
}

void CPU_I2C4_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	RCC_PeriphCLKInitTypeDef  RCC_PeriphCLKInitStruct;

	  /*##-1- Configure the I2C clock source. The clock is derived from the SYSCLK #*/
	RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2Cx_CPU;
	RCC_PeriphCLKInitStruct.I2c1ClockSelection = RCC_I2Cx_CPU_CLKSOURCE_SYSCLK;
	HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

	/*##-2- Enable peripherals and GPIO Clocks #################################*/
	/* Enable GPIO TX/RX clock */
	I2Cx_CPU_SCL_GPIO_CLK_ENABLE();
	I2Cx_CPU_SDA_GPIO_CLK_ENABLE();

	  /* Enable I2Cx clock */
	I2Cx_CPU_CLK_ENABLE();
	/*##-3- Configure peripheral GPIO ##########################################*/
	  /** I2C1 GPIO configuration
	    PD12 ------> I2C4_SCL
	    PD13 ------> I2C4_SDA
	*/
	/* I2C SCL GPIO pin configuration  */
	GPIO_InitStruct.Pin       = I2Cx_CPU_SCL_PIN;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull      = GPIO_PULLUP;
	GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate = I2Cx_CPU_SCL_SDA_AF;
	HAL_GPIO_Init(I2Cx_CPU_SCL_GPIO_PORT, &GPIO_InitStruct);
	  
	/* I2C SDA GPIO pin configuration  */
	GPIO_InitStruct.Pin       = I2Cx_CPU_SDA_PIN;
	GPIO_InitStruct.Alternate = I2Cx_CPU_SCL_SDA_AF;
	HAL_GPIO_Init(I2Cx_CPU_SDA_GPIO_PORT, &GPIO_InitStruct);
	
	  /* NVIC for I2Cx */
	HAL_NVIC_SetPriority(I2Cx_CPU_ER_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(I2Cx_CPU_ER_IRQn);
	HAL_NVIC_SetPriority(I2Cx_CPU_EV_IRQn, 0, 2);
	HAL_NVIC_EnableIRQ(I2Cx_CPU_EV_IRQn);

	/*##Configure the I2C peripheral ######################################*/
    I2C4_Handle.Instance              = I2Cx_CPU;
    I2C4_Handle.Init.Timing           = I2C_TIMING;
    I2C4_Handle.Init.OwnAddress1      = STM32F7_ADDR;
    I2C4_Handle.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    I2C4_Handle.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    I2C4_Handle.Init.OwnAddress2      = 0;
    I2C4_Handle.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    I2C4_Handle.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

    if(HAL_I2C_Init(&I2C4_Handle) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }
}
/**
  * @brief  This function handles External line 0 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void)
{
  	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0))
  	{
    	//Read button status
    	result = ReadandDisplaySensorStatus(&I2C1_Handle);
    	if(result != CY8CMBR3116_Result_OK)
		{
			Error_Handler();
		}
  	}
  	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
}
/**
	* @brief EXTI line detection callbacks
	* @param GPIO_Pin: Specifies the pins connected EXTI line
	* @retval None
	*/
// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
// {
// 	if(GPIO_Pin == KEY_BUTTON_PIN)
// 	{  
// 		UserButtonStatus = 1;
// 	}
// }


/**
  * @brief  Tx Transfer completed callback.
  * @param  I2cHandle: I2C handle. 
  * @note   This example shows a simple way to report end of DMA Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
  /* Toggle LED1: Transfer in transmission process is correct */
  BSP_LED_Toggle(LED1);
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
  /* Toggle LED1: Transfer in transmission process is correct */
  BSP_LED_Toggle(LED1);
}

/**
  * @brief  Rx Transfer completed callback.
  * @param  I2cHandle: I2C handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
  /* Toggle LED1: Transfer in reception process is correct */
  BSP_LED_Toggle(LED1);
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
  /* Toggle LED1: Transfer in reception process is correct */
  BSP_LED_Toggle(LED1);
}

/**
  * @brief  I2C error callbacks.
  * @param  I2cHandle: I2C handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle)
{
  /** Error_Handler() function is called when error occurs.
    * 1- When Slave don't acknowledge it's address, Master restarts communication.
    * 2- When Master don't acknowledge the last data transferred, Slave don't care in this example.
    */
  if (HAL_I2C_GetError(I2cHandle) != HAL_I2C_ERROR_AF)
  {
    Error_Handler();
  }
}

/**
	* @brief  This function is executed in case of error occurrence.
	* @param  None
	* @retval None
	*/
static void Error_Handler(void)
{
	/* Turn LED1 on */
	BSP_LED_On(LED1);
	while(1)
	{
		/* Error if LED1 is slowly blinking (1 sec. period) */
		BSP_LED_Toggle(LED1); 
		HAL_Delay(1000); 
	}  
}

/**
	* @brief  CPU L1-Cache enable.
	* @param  None
	* @retval None
	*/
static void CPU_CACHE_Enable(void)
{
	/* Enable I-Cache */
	SCB_EnableICache();

	/* Enable D-Cache */
	SCB_EnableDCache();
}

#ifdef  USE_FULL_ASSERT
/**
	* @brief  Reports the name of the source file and the source line number
	*         where the assert_param error has occurred.
	* @param  file: pointer to the source file name
	* @param  line: assert_param error line source number
	* @retval None
	*/
void assert_failed(uint8_t* file, uint32_t line)
{ 
	/* User can add his own implementation to report the file name and line number,
		 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif

/**
	* @}
	*/

/**
	* @}
	*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
