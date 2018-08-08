#ifndef __UART_H
#define __UART_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "stm32746g_discovery.h"

/* Definition for USART3 clock resources */
#define USARTX_MAIN                             USART3
#define USARTX_MAIN_CLK_ENABLE()                __USART3_CLK_ENABLE()
#define USARTX_MAIN_DMAx_CLK_ENABLE()           __HAL_RCC_DMA1_CLK_ENABLE()
#define USARTX_MAIN_RX_GPIO_CLK_ENABLE()        __GPIOD_CLK_ENABLE()
#define USARTX_MAIN_TX_GPIO_CLK_ENABLE()        __GPIOD_CLK_ENABLE()

#define USARTX_MAIN_FORCE_RESET()               __USART3_FORCE_RESET()
#define USARTX_MAIN_RELEASE_RESET()             __USART3_RELEASE_RESET()

/* Definition for USART3 Pins */
#define USARTX_MAIN_TX_PIN                      GPIO_PIN_8
#define USARTX_MAIN_TX_GPIO_PORT                GPIOD
#define USARTX_MAIN_TX_AF                       GPIO_AF7_USART3
#define USARTX_MAIN_RX_PIN                      GPIO_PIN_9
#define USARTX_MAIN_RX_GPIO_PORT                GPIOD
#define USARTX_MAIN_RX_AF                       GPIO_AF7_USART3

/* Definition for USART3's DMA */
#define USARTX_MAIN_TX_DMA_STREAM               DMA1_Stream3
#define USARTX_MAIN_RX_DMA_STREAM               DMA1_Stream1
#define USARTX_MAIN_TX_DMA_CHANNEL              DMA_CHANNEL_4
#define USARTX_MAIN_RX_DMA_CHANNEL              DMA_CHANNEL_4


/* Definition for USART3's NVIC */
#define USARTX_MAIN_DMA_TX_IRQn                 DMA1_Stream3_IRQn
#define USARTX_MAIN_DMA_RX_IRQn                 DMA1_Stream1_IRQn
#define USARTX_MAIN_DMA_TX_IRQHandler           DMA1_Stream3_IRQHandler
#define USARTX_MAIN_DMA_RX_IRQHandler           DMA1_Stream1_IRQHandler

#define USARTX_MAIN_IRQn                        USART3_IRQn
#define USARTX_MAIN_IRQHandler                  USART3_IRQHandler

/* Command for communication */
#define WAKEWORD_BUTTON         0x00
#define VOLUMEUP_BUTTON         0x01
#define VOLUMEDOWN_BUTTON       0x02
#define VOLUMEMUTE_BUTTON       0x03
/* Exported functions ------------------------------------------------------- */
void USARTX_MAIN_IRQHandler(void);
void USARTX_MAIN_DMA_TX_IRQHandler(void);
void USARTX_MAIN_DMA_RX_IRQHandler(void);
void UART3_Init(void);
int UART_TransmitMsg(uint8_t *msg, uint8_t size);
int UART_ReceiveMsg(uint8_t *msg, uint8_t size);
#endif /* __UART_H */
