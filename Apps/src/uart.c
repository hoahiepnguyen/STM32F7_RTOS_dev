#include "uart.h"
#include "main.h"
/* Private macro -------------------------------------------------------------*/
#define BUFFERSIZE				2
/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef UART3_Handle;
/* Private function prototypes -----------------------------------------------*/
__IO ITStatus Uart3Tx_Ready = RESET;
__IO ITStatus Uart3Rx_Ready = RESET;

/* Private functions ---------------------------------------------------------*/

void UART3_Init(void)
{
	static DMA_HandleTypeDef hdma_tx;
	static DMA_HandleTypeDef hdma_rx;

	GPIO_InitTypeDef    GPIO_InitStruct;

	/** UART3 GPIO configuration
		PD8 ------> USART3_TX
		PD9 ------> USART3_RX
	*/
	/* Enable GPIO TX/RX clock */
	USARTX_MAIN_TX_GPIO_CLK_ENABLE();
	USARTX_MAIN_RX_GPIO_CLK_ENABLE();

	/* Enable USARTx clock */
	USARTX_MAIN_CLK_ENABLE();

	/* Enable DMA clock */
	USARTX_MAIN_DMAx_CLK_ENABLE();

	/* UART TX GPIO pin configuration  */
	GPIO_InitStruct.Pin       = USARTX_MAIN_TX_PIN;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_PULLUP;
	GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate = USARTX_MAIN_TX_AF;

	HAL_GPIO_Init(USARTX_MAIN_TX_GPIO_PORT, &GPIO_InitStruct);

	/* UART RX GPIO pin configuration  */
	GPIO_InitStruct.Pin = USARTX_MAIN_RX_PIN;
	GPIO_InitStruct.Alternate = USARTX_MAIN_RX_AF;

	HAL_GPIO_Init(USARTX_MAIN_RX_GPIO_PORT, &GPIO_InitStruct);


	/*##-3- Configure the DMA ##################################################*/
	/* Configure the DMA handler for Transmission process */
	hdma_tx.Instance                 = USARTX_MAIN_TX_DMA_STREAM;
	hdma_tx.Init.Channel             = USARTX_MAIN_TX_DMA_CHANNEL;
	hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
	hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
	hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
	hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
	hdma_tx.Init.Mode                = DMA_NORMAL;
	hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;

	HAL_DMA_Init(&hdma_tx);

	/* Associate the initialized DMA handle to the UART handle */
	__HAL_LINKDMA(&UART3_Handle, hdmatx, hdma_tx);

	/* Configure the DMA handler for reception process */
	hdma_rx.Instance                 = USARTX_MAIN_RX_DMA_STREAM;
	hdma_rx.Init.Channel             = USARTX_MAIN_RX_DMA_CHANNEL;
	hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
	hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
	hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
	hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
	hdma_rx.Init.Mode                = DMA_NORMAL;
	hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;

	HAL_DMA_Init(&hdma_rx);

	/* Associate the initialized DMA handle to the the UART handle */
	__HAL_LINKDMA(&UART3_Handle, hdmarx, hdma_rx);
	  
	/*##-4- Configure the NVIC for DMA #########################################*/
	/* NVIC configuration for DMA transfer complete interrupt (USART3_TX) */
	HAL_NVIC_SetPriority(USARTX_MAIN_DMA_TX_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
	
	/* NVIC configuration for DMA transfer complete interrupt (USART3_RX) */
	HAL_NVIC_SetPriority(USARTX_MAIN_DMA_RX_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USARTX_MAIN_DMA_RX_IRQn);

	/* NVIC for USART */
	HAL_NVIC_SetPriority(USARTX_MAIN_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(USARTX_MAIN_IRQn);

	UART3_Handle.Instance        = USARTX_MAIN;
	UART3_Handle.Init.BaudRate   = 115200;
	UART3_Handle.Init.WordLength = UART_WORDLENGTH_8B;
	UART3_Handle.Init.StopBits   = UART_STOPBITS_1;
	UART3_Handle.Init.Parity     = UART_PARITY_NONE;
	UART3_Handle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	UART3_Handle.Init.Mode       = UART_MODE_TX_RX;
	UART3_Handle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT; 

	HAL_UART_Init(&UART3_Handle);
}
/**
  * @brief  This function handles UART transmision.  
  * @param  msg:	message to Main
  			size	size of message
  * @retval -1 ERROR
  			 0 SUCCESS
  * @Note
  */
int UART_TransmitMsg(uint8_t *msg, uint8_t size)
{
	Uart3Tx_Ready = RESET;
	if(HAL_UART_Transmit_DMA(&UART3_Handle, msg, size) != HAL_OK)
	{
		DEBUG_ERROR("UART Transmission");
		return -1;
	}
	while(Uart3Tx_Ready != SET)
	{
	}

	return 0;
}

/**
  * @brief  This function handles UART receiving.  
  * @param  msg:  message to Main
  			size: size of message
  * @retval -1 ERROR
  			 0 SUCCESS
  * @Note
  */
int UART_ReceiveMsg(uint8_t *msg, uint8_t size)
{
	Uart3Rx_Ready = RESET;
	if(HAL_UART_Receive_DMA(&UART3_Handle, msg, size) != HAL_OK)
	{
		DEBUG_ERROR("UART Receiving");
		return -1;
	}
	while(Uart3Rx_Ready != SET)
	{
	}

	return 0;
}
/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   This example shows a simple way to report end of DMA Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	/* Set transmission flag: trasfer complete*/
	Uart3Tx_Ready = SET;
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
  /* Set transmission flag: trasfer complete*/
  Uart3Rx_Ready = SET;
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
	DEBUG_ERROR("UART error callbacks");
}
/**
  * @brief  This function handles UART interrupt request.  
  * @param  None
  * @retval None
  * @Note   This function is redefined in "main.h" and related to DMA  
  *         used for USART data transmission     
  */
void USARTX_MAIN_IRQHandler(void)
{
  HAL_UART_IRQHandler(&UART3_Handle);
}

/**
  * @brief  This function handles DMA interrupt request.  
  * @param  None
  * @retval None
  * @Note   This function is redefined in "main.h" and related to DMA  
  *         used for USART data transmission     
  */
void USARTX_MAIN_DMA_TX_IRQHandler(void)
{
  HAL_DMA_IRQHandler(UART3_Handle.hdmatx);
}

/**
  * @brief  This function handles DMA interrupt request.  
  * @param  None
  * @retval None
  * @Note   This function is redefined in "main.h" and related to DMA  
  *         used for USART data transmission     
  */
void USARTX_MAIN_DMA_RX_IRQHandler(void)
{
  HAL_DMA_IRQHandler(UART3_Handle.hdmarx);
}
