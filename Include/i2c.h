#ifndef __I2C_H
#define __I2C_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "stm32746g_discovery.h"


/* Definition for I2Cx_SLAVE clock resources */
#define I2Cx_SLAVE                             I2C1
#define RCC_PERIPHCLK_I2Cx_SLAVE               RCC_PERIPHCLK_I2C1
#define RCC_I2Cx_SLAVE_CLKSOURCE_SYSCLK        RCC_I2C1CLKSOURCE_PCLK1
#define I2Cx_SLAVE_CLK_ENABLE()                __HAL_RCC_I2C1_CLK_ENABLE()
#define I2Cx_SLAVE_SDA_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()
#define I2Cx_SLAVE_SCL_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE() 
#define I2Cx_SLAVE_DMA_CLK_ENABLE()            __HAL_RCC_DMA1_CLK_ENABLE()

#define I2Cx_SLAVE_FORCE_RESET()               __HAL_RCC_I2C1_FORCE_RESET()
#define I2Cx_SLAVE_RELEASE_RESET()             __HAL_RCC_I2C1_RELEASE_RESET()

/* Definition for I2Cx_SLAVE Pins */
#define I2Cx_SLAVE_SCL_PIN                     GPIO_PIN_8
#define I2Cx_SLAVE_SDA_PIN                     GPIO_PIN_9
#define I2Cx_SLAVE_SCL_GPIO_PORT               GPIOB
#define I2Cx_SLAVE_SDA_GPIO_PORT               GPIOB
#define I2Cx_SLAVE_SCL_SDA_AF                  GPIO_AF4_I2C1

/* Definition for I2Cx_SLAVE's NVIC */
#define I2Cx_SLAVE_EV_IRQn                     I2C1_EV_IRQn
#define I2Cx_SLAVE_ER_IRQn                     I2C1_ER_IRQn
#define I2Cx_SLAVE_EV_IRQHandler               I2C1_EV_IRQHandler
#define I2Cx_SLAVE_ER_IRQHandler               I2C1_ER_IRQHandler

/* Definition for I2Cx_SLAVE's DMA */
#define I2Cx_SLAVE_DMA                         DMA1   
#define I2Cx_SLAVE_DMA_INSTANCE_TX             DMA1_Stream6
#define I2Cx_SLAVE_DMA_INSTANCE_RX             DMA1_Stream0
#define I2Cx_SLAVE_DMA_CHANNEL_TX              DMA_CHANNEL_1
#define I2Cx_SLAVE_DMA_CHANNEL_RX              DMA_CHANNEL_1

/* Definition for I2Cx_SLAVE's DMA NVIC */
#define I2Cx_SLAVE_DMA_TX_IRQn                 DMA1_Stream6_IRQn
#define I2Cx_SLAVE_DMA_RX_IRQn                 DMA1_Stream0_IRQn
#define I2Cx_SLAVE_DMA_TX_IRQHandler           DMA1_Stream6_IRQHandler
#define I2Cx_SLAVE_DMA_RX_IRQHandler           DMA1_Stream0_IRQHandler


/* Definition for I2Cx_SLAVE clock resources */
#define I2Cx_MASTER                              I2C2
#define RCC_PERIPHCLK_I2Cx_MASTER                RCC_PERIPHCLK_I2C2
#define RCC_I2Cx_MASTER_CLKSOURCE_SYSCLK         RCC_I2C2CLKSOURCE_PCLK1
#define I2Cx_MASTER_CLK_ENABLE()                __HAL_RCC_I2C2_CLK_ENABLE()
#define I2Cx_MASTER_SDA_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()
#define I2Cx_MASTER_SCL_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE() 
#define I2Cx_MASTER_DMA_CLK_ENABLE()            __HAL_RCC_DMA2_CLK_ENABLE()

#define I2Cx_MASTER_FORCE_RESET()               __HAL_RCC_I2C2_FORCE_RESET()
#define I2Cx_MASTER_RELEASE_RESET()             __HAL_RCC_I2C2_RELEASE_RESET()

/* Definition for I2Cx_SLAVE Pins */
#define I2Cx_MASTER_SCL_PIN                     GPIO_PIN_10
#define I2Cx_MASTER_SDA_PIN                     GPIO_PIN_11
#define I2Cx_MASTER_SCL_GPIO_PORT               GPIOB
#define I2Cx_MASTER_SDA_GPIO_PORT               GPIOB
#define I2Cx_MASTER_SCL_SDA_AF                  GPIO_AF4_I2C2

/* Definition for I2Cx_SLAVE's NVIC */
#define I2Cx_MASTER_EV_IRQn                     I2C2_EV_IRQn
#define I2Cx_MASTER_ER_IRQn                     I2C2_ER_IRQn
#define I2Cx_MASTER_EV_IRQHandler               I2C2_EV_IRQHandler
#define I2Cx_MASTER_ER_IRQHandler               I2C2_ER_IRQHandler
/* Size of Trasmission buffer */
#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)
/* Size of Reception buffer */
#define RXBUFFERSIZE                      TXBUFFERSIZE
  
/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

/* Exported functions ------------------------------------------------------- */
void I2C1_Init(I2C_HandleTypeDef *I2C1_Handle);
//void i2c2_master_init(I2C_HandleTypeDef I2C2_Handle);

#endif