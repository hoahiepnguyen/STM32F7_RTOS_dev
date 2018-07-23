#include <stdio.h>
#include <stdlib.h>
#include "cy8cmbr3.h"

/* Above are the Command Codes used to configure MBR3*/
uint8_t configData[128] = {
    //The below configuration array enables all 4 buttons, Proximity
    0x79, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00, 
    0x40, 0x15, 0x00, 0x00, 0x80, 0x7F, 0x7F, 0x80, 
    0x80, 0x80, 0x80, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 
    0x7F, 0x7F, 0x7F, 0x7F, 0x03, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 
    0x05, 0x00, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x00, 0x00, 
    0x00, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x81, 0x01, 
    0x00, 0xFF, 0xFF, 0xFF, 0x0F, 0x0F, 0x0F, 0x0F, 
    0xFF, 0x00, 0x00, 0x00, 0x03, 0x03, 0x01, 0x48, 
    0x00, 0x37, 0x01, 0x00, 0x00, 0x0A, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB8, 0xA7
};

// uint8_t configData[128] = {
// //Buzzer and Host Int Enabled: Jumper J15 in Configuration A on CY3280-MBR3 Kit
//     0x78u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
//     0x00u, 0x00u, 0x00u, 0x00u, 0x7Fu, 0x7Fu, 0x7Fu, 0x80u,
//     0x80u, 0x80u, 0x80u, 0x7Fu, 0x7Fu, 0x7Fu, 0x7Fu, 0x7Fu,
//     0x7Fu, 0x7Fu, 0x7Fu, 0x7Fu, 0x03u, 0x00u, 0x00u, 0x00u,
//     0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x80u,
//     0x05u, 0x00u, 0x00u, 0x02u, 0x00u, 0x02u, 0x00u, 0x00u,
//     0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
//     0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x81u, 0x01u,
//     0x00u, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu,
//     0xFFu, 0x00u, 0x00u, 0x00u, 0x43u, 0x03u, 0x01u, 0x58u,
//     0x00u, 0x37u, 0x06u, 0x00u, 0x00u, 0x0Au, 0x00u, 0x00u,
//     0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
//     0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
//     0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
//     0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
//     0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x96u, 0x1Eu
// };

uint8_t *TxBuffer;

void MBR3_HOST_INT_Config(void)
{
    GPIO_InitTypeDef    GPIO_InitStructure;

    /* Enable GPIOB clock */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Configure PB0 pin as input floating */
    GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Pin  = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Enable adn set EXTI Line0 Interrupt to the lowest priority */
    HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

CY8CMBR3116_Result ConfigureMBR3(I2C_HandleTypeDef *I2Cx)
{
    uint8_t REGMAP_REG = (uint8_t)REGMAP_ORIGIN;
    I2C_HandleTypeDef* handle = I2Cx;

    //Wake up the MBR3 device
    /* fix me, following cypress document, MBR3 need to be wrote
        data more than 3 times to wake up */ 
    if(HAL_I2C_IsDeviceReady(handle, (uint16_t)MBR3_ADDR, 4, 5) != HAL_OK)
    {
        DEBUG_ERROR("CY3280-MBR3 is not ready");
    }

    if (HAL_I2C_Master_Transmit(handle, (uint16_t)MBR3_ADDR, 
                &REGMAP_REG, 1, 1000) != HAL_OK)
    {
        DEBUG_ERROR("Wake up MBR3");
        return CY8CMBR3116_Result_ERROR;
    }
    // send configuration
    if (HAL_I2C_Master_Transmit(handle, (uint16_t)MBR3_ADDR, 
                (uint8_t *)configData, 128, 1000) != HAL_OK)
    {
        DEBUG_ERROR("Send configuration");
        return CY8CMBR3116_Result_ERROR;
    }

    //apply configuration to MBR3
    TxBuffer[0] = CTRL_CMD;
    TxBuffer[1] = SAVE_CHECK_CRC;
    if (HAL_I2C_Master_Transmit(handle, (uint16_t)MBR3_ADDR, 
                (uint8_t *)TxBuffer, 2, 1000) != HAL_OK)
    {
        DEBUG_ERROR("CRC checking");
        return CY8CMBR3116_Result_ERROR;
    }

    TxBuffer[0] = CTRL_CMD;
    TxBuffer[1] = SW_RESET;
    if (HAL_I2C_Master_Transmit(handle, (uint16_t)MBR3_ADDR, 
                (uint8_t *)TxBuffer, 2, 1000) != HAL_OK)
    {
        DEBUG_ERROR("Reset SW");
        return CY8CMBR3116_Result_ERROR;
    }

    //Provide a delay to calculate and save CRC
    HAL_Delay(200);
    return CY8CMBR3116_Result_OK;
}

CY8CMBR3116_Result ReadandDisplaySensorStatus(I2C_HandleTypeDef *I2Cx)
{
    uint8_t button_stat;
    uint8_t BUTTON_REG = (uint8_t)BUTTON_STATUS;
    I2C_HandleTypeDef* handle = I2Cx;

    /* fix me, following cypress document, MBR3 need to be wrote
        data more than 3 times to wake up */ 
    if(HAL_I2C_IsDeviceReady(handle, (uint16_t)MBR3_ADDR, 4, 5) != HAL_OK)
    {
        DEBUG_ERROR("CY3280-MBR3 is not ready");
    }
    //Config button
    if (HAL_I2C_Master_Transmit(handle, (uint16_t)MBR3_ADDR, 
                &BUTTON_REG, 1, 1000) != HAL_OK)
    {
        DEBUG_ERROR("Configure button");
        return CY8CMBR3116_Result_ERROR;
    }

    if(HAL_I2C_Master_Receive(handle, (uint16_t)MBR3_ADDR,
                &button_stat, 1, 1000) != HAL_OK)
    {
        DEBUG_ERROR("Read button status");
        return CY8CMBR3116_Result_ERROR;
    }

    DisplaySensorStatus(button_stat);

    return CY8CMBR3116_Result_OK;
}

int DisplaySensorStatus(uint8_t buffer)
{
    static int touched=0, prox=0;
    if(touched)
    {
        touched = 0;
        DEBUG_PRINT("Button released");
    }

    if(buffer & BUTTON_1)
    {
        //do something
        DEBUG_PRINT("Button 1 TOUCHED");
        touched = 1;
        return 1;
    }
    if((buffer & BUTTON_2))
    {
        DEBUG_PRINT("Button 2 TOUCHED");
        touched = 1;
        return 2;
    }
    if(buffer & BUTTON_3)
    {
        DEBUG_PRINT("Button 3 TOUCHED");
        touched = 1;
        return 3;
    }
    if(buffer & BUTTON_4)
    {
        DEBUG_PRINT("Button 4 TOUCHED");
        touched = 1;
        return 4;
    }
    return 0;
}