#ifndef __UART_H
#define __UART_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "stm32746g_discovery.h"

/* Definition for USART6 clock resources */
#define USART6_CLK_ENABLE()              __USART6_CLK_ENABLE()
#define DMAx_CLK_ENABLE()                __HAL_RCC_DMA2_CLK_ENABLE()
#define USART6_RX_GPIO_CLK_ENABLE()      __GPIOC_CLK_ENABLE()
#define USART6_TX_GPIO_CLK_ENABLE()      __GPIOC_CLK_ENABLE()

#define USART6_FORCE_RESET()             __USART6_FORCE_RESET()
#define USART6_RELEASE_RESET()           __USART6_RELEASE_RESET()

/* Definition for USART6 Pins */
#define USART6_TX_PIN                    GPIO_PIN_6
#define USART6_TX_GPIO_PORT              GPIOC
#define USART6_TX_AF                     GPIO_AF8_USART6
#define USART6_RX_PIN                    GPIO_PIN_7
#define USART6_RX_GPIO_PORT              GPIOC
#define USART6_RX_AF                     GPIO_AF8_USART6

/* Definition for USART6's DMA */
#define USART6_TX_DMA_STREAM              DMA2_Stream6
#define USART6_RX_DMA_STREAM              DMA2_Stream1
#define USART6_TX_DMA_CHANNEL             DMA_CHANNEL_5
#define USART6_RX_DMA_CHANNEL             DMA_CHANNEL_5


/* Definition for USART6's NVIC */
#define USART6_DMA_TX_IRQn                DMA2_Stream6_IRQn
#define USART6_DMA_RX_IRQn                DMA2_Stream1_IRQn
#define USART6_DMA_TX_IRQHandler          DMA2_Stream6_IRQHandler
#define USART6_DMA_RX_IRQHandler          DMA2_Stream1_IRQHandler


void UART6_Init(void);
int Uart6_Transmit_DMA(uint8_t *Txbuffer, uint8_t size);
int Uart6_Receive_DMA(uint8_t *RxBuff, uint8_t size);

void USART6_IRQHandler(void);
void USART6_DMA_TX_IRQHandler(void);
void USART6_DMA_RX_IRQHandler(void);
#endif /* __UART_H */
