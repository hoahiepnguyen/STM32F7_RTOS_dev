#include "main.h"
/* Private macro -------------------------------------------------------------*/
/* I2C TIMING Register define when I2C clock source is APB1 (SYSCLK/4) */
/* I2C TIMING is calculated in case of the I2C Clock source is the APB1CLK = 50 MHz */
/* This example use TIMING to 0x40912732 to reach 100 kHz speed (Rise time = 700 ns, Fall time = 100 ns) */
/* Private variables ---------------------------------------------------------*/
/* I2C handler declaration */
I2C_HandleTypeDef I2c1Handle;
I2C_HandleTypeDef I2c2Handle;
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
void i2c1_slave_init(void)
{
    /*##Configure the I2C peripheral ######################################*/
    I2c1Handle.Instance              = I2Cx_SLAVE;
    I2c1Handle.Init.Timing           = I2C_TIMING;
    I2c1Handle.Init.AddressingMode   = I2C_ADDRESSINGMODE_10BIT;
    I2c1Handle.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    I2c1Handle.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    I2c1Handle.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    I2c1Handle.Init.OwnAddress1      = STM32F7_ADDRESS;
    I2c1Handle.Init.OwnAddress2      = 0xFF;

    if(HAL_I2C_Init(&I2c1Handle) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }

    /* Enable the Analog I2C Filter */
    HAL_I2CEx_ConfigAnalogFilter(&I2c1Handle, I2C_ANALOGFILTER_ENABLE);
}

void i2c2_master_init(void)
{
    GPIO_InitTypeDef    GPIO_InitStruct;
    RCC_PeriphCLKInitTypeDef    RCC_PeriphCLKInitStruct;

    /*##-1- Configure the I2C clock source. The clock is derived from the SYSCLK #*/
    RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2Cx_MASTER;
    RCC_PeriphCLKInitStruct.I2c1ClockSelection = RCC_I2Cx_MASTER_CLKSOURCE_SYSCLK;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

    /*##-2- Enable peripherals and GPIO Clocks #################################*/
    /* Enable GPIO TX/RX clock */
    I2Cx_MASTER_SDA_GPIO_CLK_ENABLE();
    I2Cx_MASTER_SCL_GPIO_CLK_ENABLE();

    /* Enable I2Cx clock */
    I2Cx_MASTER_CLK_ENABLE();

    /*##-3- Configure peripheral GPIO ##########################################*/
    /** I2C2 GPIO configuration
        PB10 ------> I2C2_SCL
        PB11 ------> I2C2_SDA
    */
    /* I2C SCL GPIO pin configuration  */
    GPIO_InitStruct.Pin         = I2Cx_MASTER_SCL_PIN;
    GPIO_InitStruct.Mode        = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull        = GPIO_PULLUP;
    GPIO_InitStruct.Speed       = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate   = I2Cx_MASTER_SCL_SDA_AF;
    HAL_GPIO_Init(I2Cx_MASTER_SCL_GPIO_PORT, &GPIO_InitStruct);

    /* I2C SDA GPIO pin configuration  */
    GPIO_InitStruct.Pin       = I2Cx_MASTER_SDA_PIN;
    GPIO_InitStruct.Alternate = I2Cx_MASTER_SCL_SDA_AF;
    HAL_GPIO_Init(I2Cx_MASTER_SDA_GPIO_PORT, &GPIO_InitStruct);

    /* NVIC for I2Cx */
    HAL_NVIC_SetPriority(I2Cx_MASTER_ER_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(I2Cx_MASTER_ER_IRQn);
    HAL_NVIC_SetPriority(I2Cx_MASTER_EV_IRQn, 0, 2);
    HAL_NVIC_EnableIRQ(I2Cx_MASTER_EV_IRQn);

    /* Configure the I2C peripheral */
    I2c2Handle.Instance              = I2Cx_MASTER;
    I2c2Handle.Init.Timing           = I2C_TIMING;
    I2c2Handle.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    I2c2Handle.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    I2c2Handle.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    I2c2Handle.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    I2c2Handle.Init.OwnAddress1      = CYPRESS_ADDR;
    I2c2Handle.Init.OwnAddress2      = STA321MP_ADDR;
    if(HAL_I2C_Init(&I2c2Handle) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }

    /* Enable the Analog I2C Filter */
    HAL_I2CEx_ConfigAnalogFilter(&I2c2Handle, I2C_ANALOGFILTER_ENABLE);
}

