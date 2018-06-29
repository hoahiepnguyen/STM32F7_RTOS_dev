#include "log.h"
#include "main.h"
UART_HandleTypeDef UART1_Handle;

void log_init(void)
{
    UART1_Handle.Instance        = USARTx;

    UART1_Handle.Init.BaudRate   = 115200;
    UART1_Handle.Init.WordLength = UART_WORDLENGTH_8B;
    UART1_Handle.Init.StopBits   = UART_STOPBITS_1;
    UART1_Handle.Init.Parity     = UART_PARITY_NONE;
    UART1_Handle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    UART1_Handle.Init.Mode       = UART_MODE_TX_RX;
    UART1_Handle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT; 
    if(HAL_UART_DeInit(&UART1_Handle) != HAL_OK)
    {
        // Error_Handler();
    }  
    if(HAL_UART_Init(&UART1_Handle) != HAL_OK)
    {
        // Error_Handler();
    }
}

int _write(int fd, char * str, int len)
{
    HAL_UART_Transmit(&UART1_Handle, (uint8_t*)str, len , 100);
    return len;
}

