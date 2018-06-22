#include <stdio.h>
#include <stdlib.h>
#include "cy8cmbr3.h"

//Paste/Initialize the configuration array copied from the IIC file of Ez-Click here under the configData array  
uint8_t configData[128] = {
    //Buzzer and Host Int Enabled: Jumper J15 in Configuration A on CY3280-MBR3 Kit
    0x78u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,0x00u, 0x00u, 0x00u, 0x00u, 0x7Fu, 0x7Fu, 0x7Fu, 0x80u,0x80u, 0x80u, 0x80u, 0x7Fu, 0x7Fu, 0x7Fu, 0x7Fu, 0x7Fu,0x7Fu, 0x7Fu, 0x7Fu, 0x7Fu, 0x03u, 0x00u, 0x00u, 0x00u,0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x80u,0x05u, 0x00u, 0x00u, 0x02u, 0x00u, 0x02u, 0x00u, 0x00u,0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x81u, 0x01u,0x00u, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu,0xFFu, 0x00u, 0x00u, 0x00u, 0x43u, 0x03u, 0x01u, 0x58u,0x00u, 0x37u, 0x06u, 0x00u, 0x00u, 0x0Au, 0x00u, 0x00u,0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x96u, 0x1Eu
};

int readflag=0;
uint8_t *TxBuffer;

CY8CMBR3116_Result ConfigureMBR3(I2C_HandleTypeDef *I2Cx)
{
    uint8_t REGMAP_REG = (uint8_t)REGMAP_ORIGIN;
    I2C_HandleTypeDef* handle = I2Cx;

    //Wake up the MBR3 device
    if (HAL_I2C_Master_Transmit(handle, (uint16_t)CYPRESS_ADDR, 
                &REGMAP_REG, 1, 1000))
    {
        return CY8CMBR3116_Result_ERROR;
    } 
    // send configuration
    if (HAL_I2C_Master_Transmit(handle, (uint16_t)CYPRESS_ADDR, 
                (uint8_t *)configData, 128, 1000))
    {
        return CY8CMBR3116_Result_ERROR;
    }

    //apply configuration to MBR3
    TxBuffer[0] = CTRL_CMD;
    TxBuffer[1] = SAVE_CHECK_CRC;
    if (HAL_I2C_Master_Transmit(handle, (uint16_t)CYPRESS_ADDR, 
                (uint8_t *)TxBuffer, 2, 1000))
    {
        return CY8CMBR3116_Result_ERROR;
    }

    TxBuffer[0] = CTRL_CMD;
    TxBuffer[1] = SW_RESET;
    if (HAL_I2C_Master_Transmit(handle, (uint16_t)CYPRESS_ADDR, 
                (uint8_t *)TxBuffer, 2, 1000))
    {
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

    //Config button
    if (HAL_I2C_Master_Transmit(handle, (uint16_t)CYPRESS_ADDR, 
                &BUTTON_REG, 1, 1000))
    {
        return CY8CMBR3116_Result_ERROR;
    }
    printf("run here\n");
    if(HAL_I2C_Master_Receive(handle, (uint16_t)CYPRESS_ADDR,
                &button_stat, 1, 1000))
    {
        return CY8CMBR3116_Result_ERROR;
    }

    DisplaySensorStatus(button_stat);

    return CY8CMBR3116_Result_OK;
}

void DisplaySensorStatus(uint8_t buffer)
{
    static int touched=0, prox=0;
    if(touched)
    {
        touched = 0;
        printf("Button released\n");
    }

    if((buffer & 0x08) != 0)
    {
        printf("Button 1 TOUCHED\n");
        touched = 1;
    }
    else if((buffer & 0x10) != 0)
    {
        printf("Button 2 TOUCHED\n");
        touched = 1;
    }
    else if((buffer & 0x20) != 0)
    {
        printf("Button 3 TOUCHED\n");
        touched = 1;
    }
    else if((buffer & 0x40) != 0)
    {
        printf("Button 4 TOUCHED\n");
        touched = 1;
    }
}