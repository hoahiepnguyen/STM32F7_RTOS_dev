#include "main.h"
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* UART handler declaration */
UART_HandleTypeDef Uart6Handle;

/* Private function prototypes -----------------------------------------------*/

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
