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
#include "stripEffects.h"
#include "i2c.h"
#include "cy8cmbr3.h"
#include "uart.h"
#include "cmsis_os.h"
#include "stm32f7xx_it.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "DOA.h"
#include "pdm_filter.h"
#include "DSP.h"
#include "arm_math.h"
#include "gpio.h"
/** @addtogroup STM32F7xx_HAL_Examples
	* @{
	*/

/** @addtogroup UART_TwoBoards_ComIT
	* @{
	*/ 
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define I2CX_TIMING             0x00A0689A //0x40912732 //0x00303D5D; 0x00A0689A
#define STM32F7_ADDR 			0xD0
#define UPDATE_INTERVAL 		15 //refresh rate: 1/0.015ms = 66Hz
#define TASK_INTERVAL			5000
/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef UART3_Handle;
extern __IO ITStatus Uart3Tx_Ready;
extern __IO ITStatus Uart3Rx_Ready;

LPTIM_HandleTypeDef             LptimHandle;

CY8CMBR3116_Result result;

extern Mic_Array_Data Buffer1,Buffer2,Buffer3;
extern __IO int16_t SPI1_stNipple,I2S1_stNipple, I2S2_stNipple,SPI4_stNipple;
extern __IO   uint8_t I2S1_stPosShft,I2S2_stPosShft,SPI4_stPosShft;
extern USBD_AUDIO_ItfTypeDef  USBD_AUDIO_fops;
extern __IO uint8_t  swtBufUSBOut;
extern __IO uint8_t flgRacing;
extern __IO GPIO_PinState stMIC56;
extern __IO GPIO_PinState stMIC56Old;

extern uint16_t WaveRecord_flgIni;
extern uint32_t EnergySound,EnergyError;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c4;

USBH_HandleTypeDef hUSBHost;
USBD_HandleTypeDef hUSBDDevice;
AUDIO_ApplicationTypeDef appli_state = APPLICATION_IDLE;//APPLICATION_IDLE
SPI_HandleTypeDef hspi4;
GPIO_InitTypeDef GPIO_INS;
Mic_Array_Coef_f FacMic;
Audio_Out OutData;    

int16_t bufBeam1[PAR_N];
int16_t bufBeam2[PAR_N];

float CrssCorVal78,CrssCorVal14,CrssCorVal25,CrssCorVal63;

__IO uint16_t  WaveRec_idxSens4,WaveRec_idxSens3;
__IO uint16_t  WaveRec_idxSens1,WaveRec_idxSens2;
__IO uint16_t  WaveRec_idxSens5,WaveRec_idxSens6;
__IO uint8_t   flgDlyUpd; 
__IO uint8_t   flg10ms;

uint8_t buffer_switch = BUF3_PLAY; /* record data to buffer1 */
uint8_t Command_index=1;
uint8_t swtCase1Mic56;
uint8_t Direction;

extern __IO uint16_t idxFrmUSB;

/* Buffer used for transmission */
uint8_t aTxBuffer[] = "Hello Olli";
/* Buffer used for reception */
uint8_t aRxBuffer[RXBUFFERSIZE];
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void CPU_CACHE_Enable(void);
static void USB_Audio_Config(void);

void MX_I2C1_Init(void);
void PWMInit(void);

void I2C1_Init(void);
//void CPU_I2C4_Init(void);

/* Private functions ---------------------------------------------------------*/
/*--------------INLINE FUNCTION-----------------------------------------------*/
inline static void BF_Update(void)
{       
	  /* Hafl buffer is filled in by I2S data stream in */
	  if((flgDlyUpd==0))
	  {
			

			uint8_t stDirBeam, stOption;

			stDirBeam = 0;
			stOption =0;

			flgDlyUpd = 1; 
/*-------------------------------------------------------------------------------------------------------------
			  
	Sequence  Record Data                     Processing Data                 Player Data
			  
	1-------  Buffer1                         Buffer2                         Buffer3
			  
	2-------  Buffer3                         Buffer1                         Buffer2		  
			  
	3-------  Buffer2                         Buffer3                         Buffer1 
 ---------------------------------------------------------------------------------------------------------------*/
			/* Processing Data */
			switch (buffer_switch)  //buffer_switch
			{             
				case BUF1_PLAY:                    
					/* Sound Source Localization */
					Direction = DOACalc(&Buffer3);
					 
					if (HAL_LPTIM_PWM_Start(&LptimHandle, 359, Direction*60-1) != HAL_OK)
					  {
						 ////Error_Handler();
					  }
					
					if (stOption==0)
					{
						/* Summing in Buffer3 */
						BeamFormingSD(&Buffer3,stDirBeam,(int16_t *)bufBeam1);

						for (uint16_t i=0; i< PAR_N; i++)
						{
							 Buffer3.bufMIC1[i] = bufBeam1[i];
						}
					}
					else
					{
						OutData.bufMIC1 = bufBeam1;
						OutData.bufMIC2 = bufBeam2;
						/* Beam on forward and backward of sound source */
						BeamFormingSDCom(&Buffer3,stDirBeam,OutData);
						for (uint16_t i=0; i< PAR_N; i++)
						{
							 Buffer3.bufMIC1[i] = bufBeam1[i];
							 Buffer3.bufMIC2[i] = bufBeam2[i];
						}
					}               
					break;
				case BUF2_PLAY:
					/* Sound Source Localization */
					Direction = DOACalc(&Buffer1);
					if (HAL_LPTIM_PWM_Start(&LptimHandle, 359, Direction*60-1) != HAL_OK)
					{
						////Error_Handler();
					}
					if (stOption==0)
					{
						/* Summing in Buffer3 */
						BeamFormingSD(&Buffer1,stDirBeam,(int16_t *)bufBeam1);
						for (uint16_t i=0; i< PAR_N; i++)
						{
							 Buffer1.bufMIC1[i] = bufBeam1[i];
						}
					}
					else
					{
						OutData.bufMIC1 = bufBeam1;
						OutData.bufMIC2 = bufBeam2;
						/* Beam on forward and backward of sound source */
						BeamFormingSDCom(&Buffer1,stDirBeam,OutData);
						for (uint16_t i=0; i< PAR_N; i++)
						{
							 Buffer1.bufMIC1[i] = bufBeam1[i];
							 Buffer1.bufMIC2[i] = bufBeam2[i];
						}
					 }
				break;
							
			   case BUF3_PLAY:
				
					/* Sound Source Localization */
					Direction = DOACalc(&Buffer2);
					if (HAL_LPTIM_PWM_Start(&LptimHandle, 359, Direction*60-1) != HAL_OK)
					{
						 ////Error_Handler();
					}
					if (stOption==0)
					{
						/* Summing in Buffer3 */
						BeamFormingSD(&Buffer2,stDirBeam,(int16_t *)bufBeam1);
						for (uint16_t i=0; i< PAR_N; i++)
						{
							 Buffer2.bufMIC1[i] = bufBeam1[i];
						}                        
					}
					else
					{
						OutData.bufMIC1 = bufBeam1;
						OutData.bufMIC2 = bufBeam2;
						/* Beam on forward and backward of sound source */
						BeamFormingSDCom(&Buffer2,stDirBeam,OutData);
						for (uint16_t i=0; i< PAR_N; i++)
						{
							 Buffer2.bufMIC1[i] = bufBeam1[i];
							 Buffer2.bufMIC2[i] = bufBeam2[i];
						}
					 }

				break;
							
				default:
					break;
			   
		}
			
		AudioPlayerUpd();
	  }

}

inline static void Audio_Play_Out(void)
{

/*-------------------------------------------------------------------------------------------------------------
			  
	Sequence  Record Data                     Processing Data                 Player Data
			  
	1-------  Buffer1                         Buffer2                          Buffer3
			  
	2-------  Buffer3                         Buffer1                           Buffer2		  
			  
	3-------  Buffer2                         Buffer3                           Buffer1 
 ---------------------------------------------------------------------------------------------------------------*/
	flgRacing=0;

#if USB_STREAMING
	AudioUSBSend(idxFrmUSB);
#endif

	++idxFrmUSB;
	/* if player is finished for curent buffer                                  */ 
	if (idxFrmUSB == PAR_N/(AUDIO_SAMPLING_FREQUENCY/1000))
	{

	   RESET_IDX
	   idxFrmUSB=0;
	  switch (buffer_switch)
	  {
		  case BUF1_PLAY:
			  /* set flag for switch buffer */		  
			  buffer_switch = BUF3_PLAY;
			  break;
		   case BUF2_PLAY:
			  /* set flag for switch buffer */
			  buffer_switch = BUF1_PLAY;        
			  break;
		   case BUF3_PLAY:
			  /* set flag for switch buffer */		  
			  buffer_switch = BUF2_PLAY;
			  break;
		   default:
			  break;
	  }
	}			 
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
	BSP_AUDIO_OUT_ClockConfig(AUDIO_FREQ, NULL);

	/* Pin configuration for audio */
	Codec_GPIO_Init();
	GPIO_INT_Init();

	/* Configure LED RING */
	ws281x_init();

	/* PWM output */
	PWMInit();

	/* Init Superdirective Beamforming */
	BeamFormingSD_Init();

	/* Configure logs */
	log_init();

	UART3_Init();

	/* 2 channels:16Khz Audio USB */
	USB_Audio_Config();

	/* Configure I2C bus */
	I2C1_Init();
	 STA321MP_Ini();

//	CPU_I2C4_Init();
	DEBUG_PRINT("Starting.......!");

	__disable_irq();
	MIC1TO6_Init();
	StartRecord();
	__enable_irq();

	/* Configure EXTI Line0 (connected to PB0 pin) in interrupt mode */
	MBR3_HOST_INT_Config();

	/* Configure MBR3 */
	// result = ConfigureMBR3(&hi2c1);
	// if(result != CY8CMBR3116_Result_OK)
	// {
	// 	DEBUG_ERROR("Configure MBR3");
	// }

	// Uart3Tx_Ready = RESET;
	// if(HAL_UART_Transmit_DMA(&UART3_Handle, (uint8_t *)aTxBuffer, TXBUFFERSIZE) != HAL_OK)
	// {
	// 	DEBUG_ERROR("UART Transmission");
	// 	return -1;
	// }
	// while(Uart3Tx_Ready != SET)
	// {
	// }
	DEBUG_PRINT("Init done.......!");

	while(1)
	{

		BF_Update();
		// Uart3Rx_Ready = RESET;
		// if(HAL_UART_Receive_IT(&UART3_Handle, (uint8_t *)aRxBuffer, 2) != HAL_OK)
		// {
		// 	DEBUG_ERROR("UART Transmission");
		// }
		// while(Uart3Rx_Ready != SET)
		// {
		// }
		// printf("%d \r\n", aRxBuffer[0]);
		// printf("%d \r\n", aRxBuffer[1]);
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
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
	HAL_StatusTypeDef ret = HAL_OK;

	/* Enable HSE Oscillator and activate PLL with HSE as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 432;  // 432
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 9;

	ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
	if(ret != HAL_OK)
	{
	//while(1) { ; }
	}

	/* Activate the OverDrive to reach the 216 MHz Frequency */
	ret = HAL_PWREx_EnableOverDrive();
	if(ret != HAL_OK)
	{
	//while(1) { ; }
	}

	/* Select PLLSAI output as USB clock source */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48 ;
	PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;

  
	PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
	PeriphClkInitStruct.PLLSAI.PLLSAIQ = 4; 
	PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV4;
	PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;


	ret = HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct); 
	if(ret != HAL_OK)
	{
		//while(1) { ; }
	}
  
	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);
	if(ret != HAL_OK)
	{
		//while(1) { ; }
	}
  
	//sop1hc 344/7 = 49.142 MHz
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI2|RCC_PERIPHCLK_I2S;
	PeriphClkInitStruct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLI2S;
	PeriphClkInitStruct.I2sClockSelection = RCC_I2SCLKSOURCE_PLLI2S;
	PeriphClkInitStruct.PLLI2S.PLLI2SP = 8;
	PeriphClkInitStruct.PLLI2S.PLLI2SN = 344;//244
	PeriphClkInitStruct.PLLI2S.PLLI2SQ = 7;
	PeriphClkInitStruct.PLLI2S.PLLI2SR = 7;
	PeriphClkInitStruct.PLLI2SDivQ = 1;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);  
}

/**
  * @brief  Clock Config.
  * @param  hsai: might be required to set audio peripheral predivider if any.
  * @param  AudioFreq: Audio frequency used to play the audio stream.
  * @note   This API is called by BSP_AUDIO_OUT_Init() and BSP_AUDIO_OUT_SetFrequency()
  *         Being __weak it can be overwritten by the application     
  * @retval None
  */
void BSP_AUDIO_OUT_ClockConfig(uint32_t AudioFreq, void *Params)
{
  RCC_PeriphCLKInitTypeDef RCC_ExCLKInitStruct;

  HAL_RCCEx_GetPeriphCLKConfig(&RCC_ExCLKInitStruct);
  
  /* Set the PLL configuration according to the audio frequency */
  if((AudioFreq == AUDIO_FREQUENCY_11K) || (AudioFreq == AUDIO_FREQUENCY_22K) || (AudioFreq == AUDIO_FREQUENCY_44K))
  {
	/* Configure PLLSAI prescalers */
	/* PLLI2S_VCO: VCO_429M
	SAI_CLK(first level) = PLLI2S_VCO/PLLSAIQ = 429/2 = 214.5 Mhz
	SAI_CLK_x = SAI_CLK(first level)/PLLI2SDivQ = 214.5/19 = 11.289 Mhz */
	RCC_ExCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI2;
	RCC_ExCLKInitStruct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLI2S;
	RCC_ExCLKInitStruct.PLLI2S.PLLI2SP = 8;
	RCC_ExCLKInitStruct.PLLI2S.PLLI2SN = 429;
	RCC_ExCLKInitStruct.PLLI2S.PLLI2SQ = 2;
	RCC_ExCLKInitStruct.PLLI2SDivQ = 19;
	HAL_RCCEx_PeriphCLKConfig(&RCC_ExCLKInitStruct);
  }
  else /* AUDIO_FREQUENCY_8K, AUDIO_FREQUENCY_16K, AUDIO_FREQUENCY_48K), AUDIO_FREQUENCY_96K */
  {
	/* SAI clock config
	PLLI2S_VCO: VCO_344M
	SAI_CLK(first level) = PLLI2S_VCO/PLLSAIQ = 344/7 = 49.142 Mhz
	SAI_CLK_x = SAI_CLK(first level)/PLLI2SDivQ = 49.142/1 = 49.142 Mhz */
	RCC_ExCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI2;
	RCC_ExCLKInitStruct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLI2S;
	//RCC_ExCLKInitStruct.I2sClockSelection = RCC_I2SCLKSOURCE_PLLI2S;
	//RCC_ExCLKInitStruct.PLLI2S.PLLI2SP = 8;
	RCC_ExCLKInitStruct.PLLI2S.PLLI2SN = 344;//244
	RCC_ExCLKInitStruct.PLLI2S.PLLI2SQ = 7;
	//RCC_ExCLKInitStruct.PLLI2S.PLLI2SR = 1;
	RCC_ExCLKInitStruct.PLLI2SDivQ = 1;
	HAL_RCCEx_PeriphCLKConfig(&RCC_ExCLKInitStruct);
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
	hi2c1.Instance              = I2Cx_MASTER;
	hi2c1.Init.Timing           = I2CX_TIMING;
	hi2c1.Init.OwnAddress1      = 0;
	hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2      = 0;
	hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

	if(HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		/* Initialization Error */
		DEBUG_ERROR("I2C1 Configure");
	}

	/* Enable the Analog I2C Filter */
	HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
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
	hi2c4.Instance              = I2Cx_CPU;
	hi2c4.Init.Timing           = I2CX_TIMING;
	hi2c4.Init.OwnAddress1      = STM32F7_ADDR;
	hi2c4.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
	hi2c4.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
	hi2c4.Init.OwnAddress2      = 0;
	hi2c4.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
	hi2c4.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

	if(HAL_I2C_Init(&hi2c4) != HAL_OK)
	{
		/* Initialization Error */
		DEBUG_ERROR("I2C4 Configure");
	}
}


void SubFrameFinished(void)
{
	Audio_Play_Out();
}

void PWMInit(void)
{

	/* Clocks structure declaration */
	RCC_PeriphCLKInitTypeDef        RCC_PeriphCLKInitStruct;

	/* ### - 1 - Re-target the LSE to Clock the LPTIM Counter ################# */
	/* Select the LSE clock as LPTIM peripheral clock */
	RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LPTIM1;
	RCC_PeriphCLKInitStruct.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_LSE;  
	HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

  /* ### - 2 - Initialize the LPTIM peripheral ############################## */
  /*
   *  Instance        = LPTIM1
   *  Clock Source    = APB or LowPowerOSCillator (in this example LSI is
   *                    already selected from the RCC stage)
   *  Counter source  = External event.
   *  Clock prescaler = 1 (No division)
   *  Counter Trigger = Software trigger
   *  Output Polarity = High
   *  Update mode     = Immediate (Registers are immediately updated after any
   *                    write access) 
   */

  	LptimHandle.Instance = LPTIM1;

	LptimHandle.Init.Clock.Source    = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
	LptimHandle.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;  
	LptimHandle.Init.CounterSource   = LPTIM_COUNTERSOURCE_INTERNAL;
	LptimHandle.Init.Trigger.Source  = LPTIM_TRIGSOURCE_SOFTWARE; 
	LptimHandle.Init.OutputPolarity  = LPTIM_OUTPUTPOLARITY_HIGH;
	LptimHandle.Init.UpdateMode      = LPTIM_UPDATE_IMMEDIATE;
  
  	/* Initialize LPTIM peripheral according to the passed parameters */
  	if (HAL_LPTIM_Init(&LptimHandle) != HAL_OK)
  	{
		////Error_Handler();
  	}

  	/* ### - 3 - Start counting in interrupt mode ############################# */
  	/*
   	*  Period = 99
   	*  Pulse  = 49
   	*  According to this configuration, the duty cycle will be equal to 50%
   	*/
  	if (HAL_LPTIM_PWM_Start(&LptimHandle, 359, 0) != HAL_OK)
  	{
	 ////Error_Handler();
  	}

}
/**
  * @brief  This function handles External line 0 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void)
{
	/* EXTI line 0 interrupt detected */
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0) != RESET)
	{    	
		//Read button status
		result = ReadandDisplaySensorStatus(&hi2c1);
		if(result != CY8CMBR3116_Result_OK)
		{
			DEBUG_ERROR("CYPRESS read status");
		}

		// switch(result)
		// {
		// 	case 1: 
		// 		break;
		// 	case 2:
		// 		break;
		// 	case 3:
		// 		break;
		// 	case 4:
		// 		break;
		// }
	}
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
}

void EXTI4_IRQHandler(void)
{
	/* EXTI line interrupt detected */
  	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_4) != RESET)
  	{
	
	  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);

  	}
}
			  
void EXTI15_10_IRQHandler(void)
{

}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{  

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

static void USB_Audio_Config(void)
{
#if (USB_STREAMING)	
	/* Initialize USB descriptor basing on channels number and sampling frequency */
	USBD_AUDIO_Init_Microphone_Descriptor(&hUSBDDevice, AUDIO_SAMPLING_FREQUENCY, AUDIO_CHANNELS);
	/* Init Device Library */
	USBD_Init(&hUSBDDevice, &AUDIO_Desc, 0);
	/* Add Supported Class */
	USBD_RegisterClass(&hUSBDDevice, &USBD_AUDIO);
	/* Add Interface callbacks for AUDIO Class */  
	USBD_AUDIO_RegisterInterface(&hUSBDDevice, &USBD_AUDIO_fops);
	/* Start Device Process */
	USBD_Start(&hUSBDDevice);                          
#endif 
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
