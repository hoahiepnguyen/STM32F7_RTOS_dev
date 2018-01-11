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
#define I2C_TIMING              0x40912732
/* Definition I2C ADDRESS */
#define STM32F7_ADDRESS         0xD0
#define CYPRESS_ADDR            0x37
#define STA321MP_ADDR           0x40
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint32_t UserButtonStatus = 0;  /* set to 1 after User Button interrupt  */
__IO ITStatus UartReady = RESET;
osThreadId led_ring_ThreadId;
osThreadId i2c_master_ThreadId;
osThreadId i2c_slave_ThreadId;
osThreadId uart_receive_ThreadId;
osThreadId uart_transmit_ThreadId;

uint8_t aTxBuffer[] = "hello dark";
uint8_t aRxBuffer[RXBUFFERSIZE];
I2C_HandleTypeDef I2c1Handle;
I2C_HandleTypeDef I2c2Handle;
UART_HandleTypeDef Uart6Handle;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Error_Handler(void);
static void CPU_CACHE_Enable(void);
static void led_ring_Thread(void const *argument);
static void i2c_master_Thread(void const *argument);
static void i2c_slave_Thread(void const *argument);
static void uart_receive_Thread(void const * argument);
static void uart_transmit_Thread(void const * argument);
/* Private functions ---------------------------------------------------------*/

void UART6_Init(void)
{
    static DMA_HandleTypeDef hdma_tx;
    static DMA_HandleTypeDef hdma_rx;

    GPIO_InitTypeDef    GPIO_InitStruct;

    /** UART6 GPIO configuration
        PC6 ------> UART6_TX
        PC7 ------> UART6_RX
    */
    /* Enable GPIO TX/RX clock */
    USART6_TX_GPIO_CLK_ENABLE();
    USART6_RX_GPIO_CLK_ENABLE();

    /* Enable USARTx clock */
    USART6_CLK_ENABLE();

    /* UART TX GPIO pin configuration  */
    GPIO_InitStruct.Pin       = USART6_TX_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = USART6_TX_AF;

    HAL_GPIO_Init(USART6_TX_GPIO_PORT, &GPIO_InitStruct);

    /* UART RX GPIO pin configuration  */
    GPIO_InitStruct.Pin = USART6_RX_PIN;
    GPIO_InitStruct.Alternate = USART6_RX_AF;

    HAL_GPIO_Init(USART6_RX_GPIO_PORT, &GPIO_InitStruct);


    /*##-3- Configure the DMA ##################################################*/
    /* Configure the DMA handler for Transmission process */
    hdma_tx.Instance                 = USART6_TX_DMA_STREAM;
    hdma_tx.Init.Channel             = USART6_TX_DMA_CHANNEL;
    hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_tx.Init.Mode                = DMA_NORMAL;
    hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;

    HAL_DMA_Init(&hdma_tx);

    /* Associate the initialized DMA handle to the UART handle */
    __HAL_LINKDMA(&Uart6Handle, hdmatx, hdma_tx);

    /* Configure the DMA handler for reception process */
    hdma_rx.Instance                 = USART6_RX_DMA_STREAM;
    hdma_rx.Init.Channel             = USART6_RX_DMA_CHANNEL;
    hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_rx.Init.Mode                = DMA_NORMAL;
    hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;

    HAL_DMA_Init(&hdma_rx);

    /* Associate the initialized DMA handle to the the UART handle */
    __HAL_LINKDMA(&Uart6Handle, hdmarx, hdma_rx);
      
    /*##-4- Configure the NVIC for DMA #########################################*/
    /* NVIC configuration for DMA transfer complete interrupt (USART6_TX) */
    HAL_NVIC_SetPriority(USART6_DMA_TX_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(USART6_DMA_TX_IRQn);
    
    /* NVIC configuration for DMA transfer complete interrupt (USART6_RX) */
    HAL_NVIC_SetPriority(USART6_DMA_RX_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART6_DMA_RX_IRQn);

    /* NVIC for USART */
    HAL_NVIC_SetPriority(USART6_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(USART6_IRQn);

    Uart6Handle.Instance        = USART6;
    Uart6Handle.Init.BaudRate   = 115200;
    Uart6Handle.Init.WordLength = UART_WORDLENGTH_8B;
    Uart6Handle.Init.StopBits   = UART_STOPBITS_1;
    Uart6Handle.Init.Parity     = UART_PARITY_NONE;
    Uart6Handle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    Uart6Handle.Init.Mode       = UART_MODE_TX_RX;
    Uart6Handle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if(HAL_UART_Init(&Uart6Handle) != HAL_OK)
    {
        Error_Handler();
    }

}

void i2c1_slave_init(void)
{
    /*##Configure the I2C peripheral ######################################*/
    I2c1Handle.Instance              = I2Cx_SLAVE;
    I2c1Handle.Init.Timing           = I2C_TIMING;
    I2c1Handle.Init.AddressingMode   = I2C_ADDRESSINGMODE_10BIT;
    I2c1Handle.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    I2c1Handle.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    I2c1Handle.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    I2c1Handle.Init.OwnAddress1      = STM32F7_ADDRESS;
    I2c1Handle.Init.OwnAddress2      = 0xFF;

    if(HAL_I2C_Init(&I2c1Handle) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }

    /* Enable the Analog I2C Filter */
    HAL_I2CEx_ConfigAnalogFilter(&I2c1Handle, I2C_ANALOGFILTER_ENABLE);
}

void i2c2_master_init(void)
{
    GPIO_InitTypeDef    GPIO_InitStruct;
    RCC_PeriphCLKInitTypeDef    RCC_PeriphCLKInitStruct;

    /*##-1- Configure the I2C clock source. The clock is derived from the SYSCLK #*/
    RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2Cx_MASTER;
    RCC_PeriphCLKInitStruct.I2c1ClockSelection = RCC_I2Cx_MASTER_CLKSOURCE_SYSCLK;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

    /*##-2- Enable peripherals and GPIO Clocks #################################*/
    /* Enable GPIO TX/RX clock */
    I2Cx_MASTER_SDA_GPIO_CLK_ENABLE();
    I2Cx_MASTER_SCL_GPIO_CLK_ENABLE();

    /* Enable I2Cx clock */
    I2Cx_MASTER_CLK_ENABLE();

    /*##-3- Configure peripheral GPIO ##########################################*/
    /** I2C2 GPIO configuration
        PB10 ------> I2C2_SCL
        PB11 ------> I2C2_SDA
    */
    /* I2C SCL GPIO pin configuration  */
    GPIO_InitStruct.Pin         = I2Cx_MASTER_SCL_PIN;
    GPIO_InitStruct.Mode        = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull        = GPIO_PULLUP;
    GPIO_InitStruct.Speed       = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate   = I2Cx_MASTER_SCL_SDA_AF;
    HAL_GPIO_Init(I2Cx_MASTER_SCL_GPIO_PORT, &GPIO_InitStruct);

    /* I2C SDA GPIO pin configuration  */
    GPIO_InitStruct.Pin       = I2Cx_MASTER_SDA_PIN;
    GPIO_InitStruct.Alternate = I2Cx_MASTER_SCL_SDA_AF;
    HAL_GPIO_Init(I2Cx_MASTER_SDA_GPIO_PORT, &GPIO_InitStruct);

    /* NVIC for I2Cx */
    HAL_NVIC_SetPriority(I2Cx_MASTER_ER_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(I2Cx_MASTER_ER_IRQn);
    HAL_NVIC_SetPriority(I2Cx_MASTER_EV_IRQn, 0, 2);
    HAL_NVIC_EnableIRQ(I2Cx_MASTER_EV_IRQn);

    /* Configure the I2C peripheral */
    I2c2Handle.Instance              = I2Cx_MASTER;
    I2c2Handle.Init.Timing           = I2C_TIMING;
    I2c2Handle.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    I2c2Handle.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    I2c2Handle.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    I2c2Handle.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    I2c2Handle.Init.OwnAddress1      = CYPRESS_ADDR;
    I2c2Handle.Init.OwnAddress2      = STA321MP_ADDR;
    if(HAL_I2C_Init(&I2c2Handle) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }

    /* Enable the Analog I2C Filter */
    HAL_I2CEx_ConfigAnalogFilter(&I2c2Handle, I2C_ANALOGFILTER_ENABLE);
}

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
	/* Configure USER Button */
//	BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

	/* Configure logs */
	log_init();
	/* Congigure USART6 */
	UART6_Init();

	/* Configure I2C bus */
//	i2c1_slave_init();
//	i2c2_master_init();

	/* Threads definition */
//	osThreadDef(i2c_master, i2c_master_Thread, osPriorityRealtime, 0, configMINIMAL_STACK_SIZE);
//	osThreadDef(i2c_slave, i2c_slave_Thread, osPriorityRealtime, 0, configMINIMAL_STACK_SIZE);
//	osThreadDef(led_ring, led_ring_Thread, osPriorityHigh, 0, configMINIMAL_STACK_SIZE);
	osThreadDef(uart_transmit, uart_transmit_Thread, osPriorityHigh, 0, configMINIMAL_STACK_SIZE);
	osThreadDef(uart_receive, uart_receive_Thread, osPriorityHigh, 0, configMINIMAL_STACK_SIZE);

	/* Start threads */

//	i2c_master_ThreadId = osThreadCreate(osThread(i2c_master), NULL);
//	i2c_slave_ThreadId = osThreadCreate(osThread(i2c_slave), NULL);
//	led_ring_ThreadId = osThreadCreate(osThread(led_ring), NULL);
//	uart_transmit_ThreadId = osThreadCreate(osThread(uart_transmit), NULL);
	uart_receive_ThreadId = osThreadCreate(osThread(uart_receive), NULL);
	printf("Init Done \r\n");
	/* Start scheduler */
	osKernelStart();

	/* We should never
	 get here as control is now taken by the scheduler */
	for(;;);
}

/**
    * @brief  Toggle LED1 thread
    * @param  Thread not used
    * @retval None
    */
static void i2c_master_Thread(void const *argument)
{
    (void) argument;
    uint32_t PreviousWakeTime = osKernelSysTick();

    aTxBuffer[0] = 0x00; //address reg
    aTxBuffer[1] = 0x01; //data

    for(;;)
    {
        BSP_LED_On(LED1);
        // /* Wait for USER button press before starting the Communication */
        // while(BSP_PB_GetState(BUTTON_KEY) != 1)
        // {

        // }

        // /* wait for USER button release before starting the Communication */
        // while(BSP_PB_GetState(BUTTON_KEY) != 0)
        // {
        // }

        /*## Start the transmission process #####################################*/  
        /* While the I2C in reception process, user can transmit data through 
         "aTxBuffer" buffer */
        while(HAL_I2C_Master_Transmit_IT(&I2c2Handle, (uint16_t)CYPRESS_ADDR, (uint8_t*)aTxBuffer, 2)!= HAL_OK)
        {
        /* Error_Handler() function is called when Timeout error occurs.
           When Acknowledge failure occurs (Slave don't acknowledge it's address)
           Master restarts communication */
            if (HAL_I2C_GetError(&I2c2Handle) != HAL_I2C_ERROR_AF)
            {
                Error_Handler();
            }
        }
        BSP_LED_Off(LED1);

        /*## Wait for the end of the transfer ###################################*/  
        /*  Before starting a new communication transfer, you need to check the current   
          state of the peripheral; if it’s busy you need to wait for the end of current
          transfer before starting a new one.
          For simplicity reasons, this example is just waiting till the end of the 
          transfer, but application may perform other tasks while transfer operation
          is ongoing. */  
        while (HAL_I2C_GetState(&I2c2Handle) != HAL_I2C_STATE_READY)
        {
        }
        // /* Wait for USER Button press before starting the Communication */
        // while (BSP_PB_GetState(BUTTON_KEY) != 1)
        // {
        // }

        // /* Wait for USER Button release before starting the Communication */
        // while (BSP_PB_GetState(BUTTON_KEY) != 0)
        // {
        // }

        /*## Put I2C peripheral in reception process ###########################*/  
        if (HAL_I2C_Master_Receive_IT(&I2c2Handle, (uint16_t)CYPRESS_ADDR, (uint8_t *)aRxBuffer, 2) == HAL_OK)
        {
            BSP_LED_On(LED1);
            printf("I2C_MASTER: aRxBuffer[0] = %x\r\n", aRxBuffer[0]);
            printf("I2C_MASTER: aRxBuffer[1] = %x\r\n", aRxBuffer[1]);
        }
        else
        {
            osDelayUntil(&PreviousWakeTime, 100);
        }

    }

}

/**
    * @brief  Toggle LED1 thread
    * @param  Thread not used
    * @retval None
    */
static void i2c_slave_Thread(void const *argument)
{
    (void) argument;
    uint32_t PreviousWakeTime = osKernelSysTick();

    for(;;)
    {
        if(HAL_I2C_Slave_Receive_DMA(&I2c1Handle, (uint8_t *)aRxBuffer, 2) == HAL_OK)
        {
            BSP_LED_On(LED1);
            printf("%x\r\n", aRxBuffer[0]);
            printf("%x\r\n", aRxBuffer[1]);
        }
        else
        {
            osDelayUntil(&PreviousWakeTime, 100);
        }
    }
}

static void led_ring_Thread(void const *argument)
{
	(void) argument;
	uint32_t PreviousWakeTime = osKernelSysTick();

	for(;;)
	{
		osDelayUntil(&PreviousWakeTime, 1000);
		printf("led ring\r\n");
	}
}

static void uart_receive_Thread(void const * argument)
{
	(void) argument;
	uint32_t PreviousWakeTime = osKernelSysTick();
	int i;

	for(;;)
	{
        /* Clear buffer before receiving */
        for(i=0; i<4; i++)
        {
          aRxBuffer[i] = 0;
        }
		/* Reset transmission flag */
    	UartReady = RESET;

  		if(HAL_UART_Receive_IT(&Uart6Handle, (uint8_t *)aRxBuffer, 4) != HAL_OK)
  		{
			Error_Handler();
        }
		while (UartReady != SET)
		{
     	}
     	if(aRxBuffer[0] == 66)
     	{
			printf("receive 'B' string\r\n");
     	}
     	if(aRxBuffer[1] == 76)
     	{
			printf("receive 'L' string\r\n");
     	}
     	if(aRxBuffer[2] == 48)
     	{
			printf("receive '0' string\r\n");
     	}
     	if(aRxBuffer[3] == 49)
     	{
			printf("receive '1' string\r\n");

     	}
     	printf("\r\n");
		osDelayUntil(&PreviousWakeTime, 100);
	}
}

static void uart_transmit_Thread(void const * argument)
{
	(void) argument;
	uint32_t PreviousWakeTime = osKernelSysTick();

    /* Reset transmission flag */
    UartReady = RESET;

	if(HAL_UART_Transmit_IT(&Uart6Handle, (uint8_t *)aTxBuffer, TXBUFFERSIZE) != HAL_OK)
	{
		Error_Handler();
	}

	while (UartReady != SET)
	{
    }
	for(;;)
	{
		osDelayUntil(&PreviousWakeTime, 1000);
	}
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
	* @brief EXTI line detection callbacks
	* @param GPIO_Pin: Specifies the pins connected EXTI line
	* @retval None
	*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == KEY_BUTTON_PIN)
	{  
		UserButtonStatus = 1;
	}
}

/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   This example shows a simple way to report end of IT Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
  UartReady = SET;

  
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  	/* Set transmission flag: transfer complete */
  	UartReady = SET;
//	printf("run here!!!\r\n");
  
}

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
    Error_Handler();
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
