#include "ws281x.h"

/* Variables -----------------------------------------------*/
static uint8_t LEDbuffer[LED_BUFFER_SIZE];

TIM_HandleTypeDef       TimHandle;
TIM_OC_InitTypeDef      sConfig;
GPIO_InitTypeDef        GPIO_InitStruct;
DMA_HandleTypeDef       hdma_tim;

/* Functions -----------------------------------------------*/

/**
 * @brief TIM MSP Initialization
 *        This function configures the hardware resources used in this example:
 *           - Peripheral's clock enable
 *           - Peripheral's GPIO Configuration
 *           - DMA configuration for transmission request by peripheral
 * @param htim: TIM handle pointer
 * @retval None
 */

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	/*##-1- Enable peripherals and GPIO Clocks #################################*/
	/* TIMx clock enable */
	TIMx_CLK_ENABLE();

	/* Enable GPIO Channel Clock */
	TIMx_CHANNEL1_GPIO_CLK_ENABLE();

	/* Enable DMA clock */
	DMAx_CLK_ENABLE();

	/* Configure TIM1 channel 1 in output, push-pull & alternate function mode */
	GPIO_InitStruct.Pin = GPIO_PIN_CHANNEL1;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF_TIMx;
	HAL_GPIO_Init(TIMx_GPIO_CHANNEL1_PORT, &GPIO_InitStruct);

	
}