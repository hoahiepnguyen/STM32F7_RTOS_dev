#include "i2c.h"
#include "cy8cmbr3.h"
/* Private macro -------------------------------------------------------------*/
/* I2C TIMING Register define when I2C clock source is APB1 (SYSCLK/4) */
/* I2C TIMING is calculated in case of the I2C Clock source is the APB1CLK = 50 MHz */
/* This example use TIMING to 0x40912732 to reach 100 kHz speed (Rise time = 700 ns, Fall time = 100 ns) */
#define I2C_TIMING              0x40912732
#define STA321MP_ADDR           0x40
#define STM32F7_ADDR            0xD0
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/**
  * @brief I2C MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  *           - DMA configuration for transmission request by peripheral 
  *           - NVIC configuration for DMA interrupt request enable
  * @param hi2c: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  static DMA_HandleTypeDef hdma_tx;
  static DMA_HandleTypeDef hdma_rx;
  RCC_PeriphCLKInitTypeDef  RCC_PeriphCLKInitStruct;
  
  /*##-1- Configure the I2C clock source. The clock is derived from the SYSCLK #*/
  RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2Cx_SLAVE;
  RCC_PeriphCLKInitStruct.I2c1ClockSelection = RCC_I2Cx_SLAVE_CLKSOURCE_SYSCLK;
  HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

  /*##-2- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO TX/RX clock */
  I2Cx_SLAVE_SCL_GPIO_CLK_ENABLE();
  I2Cx_SLAVE_SDA_GPIO_CLK_ENABLE();
  /* Enable I2Cx clock */
  I2Cx_SLAVE_CLK_ENABLE(); 

  /* Enable DMAx clock */
  I2Cx_SLAVE_DMA_CLK_ENABLE();
  
  /*##-3- Configure peripheral GPIO ##########################################*/
    /** I2C1 GPIO configuration
      PB8 ------> I2C1_SCL
      PB9 ------> I2C1_SDA
  */
  /* I2C SCL GPIO pin configuration  */
  GPIO_InitStruct.Pin       = I2Cx_SLAVE_SCL_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = I2Cx_SLAVE_SCL_SDA_AF;
  HAL_GPIO_Init(I2Cx_SLAVE_SCL_GPIO_PORT, &GPIO_InitStruct);
    
  /* I2C SDA GPIO pin configuration  */
  GPIO_InitStruct.Pin       = I2Cx_SLAVE_SDA_PIN;
  GPIO_InitStruct.Alternate = I2Cx_SLAVE_SCL_SDA_AF;
  HAL_GPIO_Init(I2Cx_SLAVE_SDA_GPIO_PORT, &GPIO_InitStruct);
    
  /*##-4- Configure the DMA Channels #########################################*/
  /* Configure the DMA handler for Transmission process */
  hdma_tx.Instance                 = I2Cx_SLAVE_DMA_INSTANCE_TX;
  hdma_tx.Init.Channel             = I2Cx_SLAVE_DMA_CHANNEL_TX;                     
  hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_tx.Init.Mode                = DMA_NORMAL;
  hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;
  hdma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;           /* FIFO mode disabled               */
  hdma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  hdma_tx.Init.MemBurst            = DMA_MBURST_SINGLE;              /* Memory burst                     */
  hdma_tx.Init.PeriphBurst         = DMA_PBURST_SINGLE;           /* Peripheral burst                 */

  HAL_DMA_Init(&hdma_tx);   
  
  /* Associate the initialized DMA handle to the the I2C handle */
  __HAL_LINKDMA(hi2c, hdmatx, hdma_tx);
    
  /* Configure the DMA handler for Transmission process */
  hdma_rx.Instance                 = I2Cx_SLAVE_DMA_INSTANCE_RX;
  hdma_rx.Init.Channel             = I2Cx_SLAVE_DMA_CHANNEL_RX;                     
  hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_rx.Init.Mode                = DMA_NORMAL;
  hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;
  hdma_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;           /* FIFO mode disabled               */
  hdma_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  hdma_rx.Init.MemBurst            = DMA_MBURST_SINGLE;              /* Memory burst                     */
  hdma_rx.Init.PeriphBurst         = DMA_PBURST_SINGLE;           /* Peripheral burst                 */

  HAL_DMA_Init(&hdma_rx);
    
  /* Associate the initialized DMA handle to the the I2C handle */
  __HAL_LINKDMA(hi2c, hdmarx, hdma_rx);
    
  /*##-5- Configure the NVIC for DMA #########################################*/
  /* NVIC configuration for DMA transfer complete interrupt (I2Cx_TX) */
  HAL_NVIC_SetPriority(I2Cx_SLAVE_DMA_TX_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(I2Cx_SLAVE_DMA_TX_IRQn);
    
  /* NVIC configuration for DMA transfer complete interrupt (I2Cx_RX) */
  HAL_NVIC_SetPriority(I2Cx_SLAVE_DMA_RX_IRQn, 0, 0);   
  HAL_NVIC_EnableIRQ(I2Cx_SLAVE_DMA_RX_IRQn);
  
  /* NVIC for I2Cx */
  HAL_NVIC_SetPriority(I2Cx_SLAVE_ER_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(I2Cx_SLAVE_ER_IRQn);
  HAL_NVIC_SetPriority(I2Cx_SLAVE_EV_IRQn, 0, 2);
  HAL_NVIC_EnableIRQ(I2Cx_SLAVE_EV_IRQn);
}

/**
  * @brief I2C MSP De-Initialization 
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO, DMA and NVIC configuration to their default state
  * @param hi2c: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
  
  static DMA_HandleTypeDef hdma_tx;
  static DMA_HandleTypeDef hdma_rx;

  /*##-1- Reset peripherals ##################################################*/
  I2Cx_SLAVE_FORCE_RESET();
  I2Cx_SLAVE_RELEASE_RESET();

  /*##-2- Disable peripherals and GPIO Clocks #################################*/
  /* Configure I2C Tx as alternate function  */
  HAL_GPIO_DeInit(I2Cx_SLAVE_SCL_GPIO_PORT, I2Cx_SLAVE_SCL_PIN);
  /* Configure I2C Rx as alternate function  */
  HAL_GPIO_DeInit(I2Cx_SLAVE_SDA_GPIO_PORT, I2Cx_SLAVE_SDA_PIN);
   
  /*##-3- Disable the DMA Channels ###########################################*/
  /* De-Initialize the DMA Channel associated to transmission process */
  HAL_DMA_DeInit(&hdma_tx); 
  /* De-Initialize the DMA Channel associated to reception process */
  HAL_DMA_DeInit(&hdma_rx);
  
  /*##-4- Disable the NVIC for DMA ###########################################*/
  HAL_NVIC_DisableIRQ(I2Cx_SLAVE_DMA_TX_IRQn);
  HAL_NVIC_DisableIRQ(I2Cx_SLAVE_DMA_RX_IRQn);
  
  /*##-5- Disable the NVIC for I2C ##########################################*/
  HAL_NVIC_DisableIRQ(I2Cx_SLAVE_ER_IRQn);
  HAL_NVIC_DisableIRQ(I2Cx_SLAVE_EV_IRQn);
}
/* Private functions ---------------------------------------------------------*/
void I2C1_Init(I2C_HandleTypeDef *I2C1_Handle)
{
    /*##Configure the I2C peripheral ######################################*/
    I2C1_Handle->Instance              = I2Cx_SLAVE;
    I2C1_Handle->Init.Timing           = I2C_TIMING;
    I2C1_Handle->Init.AddressingMode   = I2C_ADDRESSINGMODE_10BIT;
    I2C1_Handle->Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    I2C1_Handle->Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    I2C1_Handle->Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    I2C1_Handle->Init.OwnAddress1      = STM32F7_ADDR;
    I2C1_Handle->Init.OwnAddress2      = CYPRESS_ADDR;

    if(HAL_I2C_Init(I2C1_Handle) != HAL_OK)
    {
        /* Initialization Error */
    }

    /* Enable the Analog I2C Filter */
    HAL_I2CEx_ConfigAnalogFilter(I2C1_Handle, I2C_ANALOGFILTER_ENABLE);
}

// void i2c2_master_init(I2C_HandleTypeDef I2C2_Handle)
// {

//     GPIO_InitTypeDef    GPIO_InitStruct;
//     RCC_PeriphCLKInitTypeDef    RCC_PeriphCLKInitStruct;

//     /*##-1- Configure the I2C clock source. The clock is derived from the SYSCLK #*/
//     RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2Cx_MASTER;
//     RCC_PeriphCLKInitStruct.I2c1ClockSelection = RCC_I2Cx_MASTER_CLKSOURCE_SYSCLK;
//     HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

//     /*##-2- Enable peripherals and GPIO Clocks #################################*/
//     /* Enable GPIO TX/RX clock */
//     I2Cx_MASTER_SDA_GPIO_CLK_ENABLE();
//     I2Cx_MASTER_SCL_GPIO_CLK_ENABLE();

//     /* Enable I2Cx clock */
//     I2Cx_MASTER_CLK_ENABLE();

//     /*##-3- Configure peripheral GPIO ##########################################*/
//     /** I2C2 GPIO configuration
//         PB10 ------> I2C2_SCL
//         PB11 ------> I2C2_SDA
//     */
//     /* I2C SCL GPIO pin configuration  */
//     GPIO_InitStruct.Pin         = I2Cx_MASTER_SCL_PIN;
//     GPIO_InitStruct.Mode        = GPIO_MODE_AF_OD;
//     GPIO_InitStruct.Pull        = GPIO_PULLUP;
//     GPIO_InitStruct.Speed       = GPIO_SPEED_HIGH;
//     GPIO_InitStruct.Alternate   = I2Cx_MASTER_SCL_SDA_AF;
//     HAL_GPIO_Init(I2Cx_MASTER_SCL_GPIO_PORT, &GPIO_InitStruct);

//     /* I2C SDA GPIO pin configuration  */
//     GPIO_InitStruct.Pin       = I2Cx_MASTER_SDA_PIN;
//     GPIO_InitStruct.Alternate = I2Cx_MASTER_SCL_SDA_AF;
//     HAL_GPIO_Init(I2Cx_MASTER_SDA_GPIO_PORT, &GPIO_InitStruct);

//     /* NVIC for I2Cx */
//     HAL_NVIC_SetPriority(I2Cx_MASTER_ER_IRQn, 0, 1);
//     HAL_NVIC_EnableIRQ(I2Cx_MASTER_ER_IRQn);
//     HAL_NVIC_SetPriority(I2Cx_MASTER_EV_IRQn, 0, 2);
//     HAL_NVIC_EnableIRQ(I2Cx_MASTER_EV_IRQn);

//     /* Configure the I2C peripheral */
//     I2C2_Handle.Instance              = I2Cx_MASTER;
//     I2C2_Handle.Init.Timing           = I2C_TIMING;
//     I2C2_Handle.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
//     I2C2_Handle.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
//     I2C2_Handle.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
//     I2C2_Handle.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
//     I2C2_Handle.Init.OwnAddress1      = CYPRESS_ADDR;
//     I2C2_Handle.Init.OwnAddress2      = STA321MP_ADDR;
//     if(HAL_I2C_Init(&I2C2_Handle) != HAL_OK)
//     {
//         /* Initialization Error */
//         Error_Handler();
//     }

//     /* Enable the Analog I2C Filter */
//     HAL_I2CEx_ConfigAnalogFilter(&I2C2_Handle, I2C_ANALOGFILTER_ENABLE);
// }

