#ifndef __GPIO_H
#define __GPIO_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "stm32746g_discovery.h"

#define CPU_INT0_PIN               GPIO_PIN_0
#define CPU_INT0_GPIO_PORT         GPIOD
#define CPU_INT0_GPIO_CLK_ENABLE() __HAL_RCC_GPIOD_CLK_ENABLE()

#define CPU_INT1_PIN               GPIO_PIN_0
#define CPU_INT1_GPIO_PORT         GPIOE
#define CPU_INT1_GPIO_CLK_ENABLE() __HAL_RCC_GPIOE_CLK_ENABLE()
/* Exported functions ------------------------------------------------------- */
void GPIO_INT_Init(void);
void GPIO_INT0_Toggle(void);

#endif /* __GPIO_H */