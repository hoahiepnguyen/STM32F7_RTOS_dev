#include "gpio.h"

void GPIO_INT_Init(void)
{
    GPIO_InitTypeDef    GPIO_Init;

    CPU_INT0_GPIO_CLK_ENABLE();
    CPU_INT1_GPIO_CLK_ENABLE();

    GPIO_Init.Pin   = CPU_INT0_PIN;
    GPIO_Init.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_Init.Pull  = GPIO_PULLUP;
    GPIO_Init.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(CPU_INT0_GPIO_PORT, &GPIO_Init);

    /* Set Pin as low level */
    HAL_GPIO_WritePin(CPU_INT0_GPIO_PORT, CPU_INT0_PIN, GPIO_PIN_RESET);
}

void GPIO_INT0_Toggle(void)
{
    HAL_GPIO_WritePin(CPU_INT0_GPIO_PORT, CPU_INT0_PIN, GPIO_PIN_SET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(CPU_INT0_GPIO_PORT, CPU_INT0_PIN, GPIO_PIN_RESET);
}

