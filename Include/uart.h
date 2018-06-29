#ifndef __UART_H
#define __UART_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "stm32746g_discovery.h"

/* Definition for USART3 clock resources */
#define USARTX_CPU                       USART3
#define USART3_CLK_ENABLE()              __USART3_CLK_ENABLE()
#define DMAx_CLK_ENABLE()                __HAL_RCC_DMA1_CLK_ENABLE()
#define USART3_RX_GPIO_CLK_ENABLE()      __GPIOC_CLK_ENABLE()
#define USART3_TX_GPIO_CLK_ENABLE()      __GPIOC_CLK_ENABLE()

#define USART3_FORCE_RESET()             __USART3_FORCE_RESET()
#define USART3_RELEASE_RESET()           __USART3_RELEASE_RESET()

/* Definition for USART3 Pins */
#define USART3_TX_PIN                    GPIO_PIN_8
#define USART3_TX_GPIO_PORT              GPIOD
#define USART3_TX_AF                     GPIO_AF7_USART3
#define USART3_RX_PIN                    GPIO_PIN_9
#define USART3_RX_GPIO_PORT              GPIOD
#define USART3_RX_AF                     GPIO_AF7_USART3

/* Definition for USART3's DMA */
#define USART3_TX_DMA_STREAM              DMA1_Stream3
#define USART3_RX_DMA_STREAM              DMA1_Stream1
#define USART3_TX_DMA_CHANNEL             DMA_CHANNEL_4
#define USART3_RX_DMA_CHANNEL             DMA_CHANNEL_4


/* Definition for USART3's NVIC */
#define USART3_DMA_TX_IRQn                DMA1_Stream2_IRQn
#define USART3_DMA_RX_IRQn                DMA1_Stream1_IRQn
#define USART3_DMA_TX_IRQHandler          DMA1_Stream2_IRQHandler
#define USART3_DMA_RX_IRQHandler          DMA1_Stream1_IRQHandler

/* Exported functions ------------------------------------------------------- */
void UART3_Init(void);
#endif /* __UART_H */
