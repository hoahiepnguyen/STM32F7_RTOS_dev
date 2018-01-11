#include "log.h"
#include "main.h"
UART_HandleTypeDef Uart1Handle;

void log_init(void)
{
    Uart1Handle.Instance        = USARTx;

    Uart1Handle.Init.BaudRate   = 115200;
    Uart1Handle.Init.WordLength = UART_WORDLENGTH_8B;
    Uart1Handle.Init.StopBits   = UART_STOPBITS_1;
    Uart1Handle.Init.Parity     = UART_PARITY_NONE;
    Uart1Handle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    Uart1Handle.Init.Mode       = UART_MODE_TX_RX;
    Uart1Handle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT; 
    if(HAL_UART_DeInit(&Uart1Handle) != HAL_OK)
    {
        // Error_Handler();
    }  
    if(HAL_UART_Init(&Uart1Handle) != HAL_OK)
    {
        // Error_Handler();
    }
}

int _write(int fd, char * str, int len)
{
    HAL_UART_Transmit(&Uart1Handle, (uint8_t*)str, len , 100);
    return len;
}

