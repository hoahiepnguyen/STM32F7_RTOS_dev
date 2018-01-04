#include "log.h"
#include "main.h"
UART_HandleTypeDef UartHandle;

void log_init(void)
{
    UartHandle.Instance        = USARTx;

    UartHandle.Init.BaudRate   = 115200;
    UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    UartHandle.Init.StopBits   = UART_STOPBITS_1;
    UartHandle.Init.Parity     = UART_PARITY_NONE;
    UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    UartHandle.Init.Mode       = UART_MODE_TX_RX;
    UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT; 
    if(HAL_UART_DeInit(&UartHandle) != HAL_OK)
    {
        // Error_Handler();
    }  
    if(HAL_UART_Init(&UartHandle) != HAL_OK)
    {
        // Error_Handler();
    }
}

int _write(int fd, char * str, int len)
{
    HAL_UART_Transmit(&UartHandle, (uint8_t*)str, len , 100);
    return len;
}

