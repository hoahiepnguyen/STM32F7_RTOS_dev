#include "main.h"
#include "uart.h"
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef UART3_Handle;
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

void UART3_Init(void)
{
    static DMA_HandleTypeDef hdma_tx;
    static DMA_HandleTypeDef hdma_rx;

    GPIO_InitTypeDef    GPIO_InitStruct;

    /** UART3 GPIO configuration
        PD8 ------> UART6_TX
        PD9 ------> UART6_RX
    */
    /* Enable GPIO TX/RX clock */
    USART3_TX_GPIO_CLK_ENABLE();
    USART3_RX_GPIO_CLK_ENABLE();

    /* Enable USARTx clock */
    USART3_CLK_ENABLE();

    /* UART TX GPIO pin configuration  */
    GPIO_InitStruct.Pin       = USART3_TX_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = USART3_TX_AF;

    HAL_GPIO_Init(USART3_TX_GPIO_PORT, &GPIO_InitStruct);

    /* UART RX GPIO pin configuration  */
    GPIO_InitStruct.Pin = USART3_RX_PIN;
    GPIO_InitStruct.Alternate = USART3_RX_AF;

    HAL_GPIO_Init(USART3_RX_GPIO_PORT, &GPIO_InitStruct);


    /*##-3- Configure the DMA ##################################################*/
    /* Configure the DMA handler for Transmission process */
    hdma_tx.Instance                 = USART3_TX_DMA_STREAM;
    hdma_tx.Init.Channel             = USART3_TX_DMA_CHANNEL;
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
    hdma_rx.Instance                 = USART3_RX_DMA_STREAM;
    hdma_rx.Init.Channel             = USART3_RX_DMA_CHANNEL;
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
    HAL_NVIC_SetPriority(USART3_DMA_TX_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(USART3_DMA_TX_IRQn);
    
    /* NVIC configuration for DMA transfer complete interrupt (USART3_RX) */
    HAL_NVIC_SetPriority(USART3_DMA_RX_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_DMA_RX_IRQn);

    /* NVIC for USART */
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(USART3_IRQn);

    UART3_Handle.Instance        = USARTX_CPU;
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
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   This example shows a simple way to report end of DMA Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: trasfer complete*/
  //Uart3_TxReady = SET;
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
  //Uart3_RxReady = SET;
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
    //Error_Handler();
}