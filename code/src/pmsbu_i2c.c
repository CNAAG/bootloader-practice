#include <stdint.h>
#include "pmbus_i2c.h"
#include "stm32g4xx.h"

#define I2C_TIMING_100K             (0x30420F13) // 100KHz
#define I2C_TIMING_400K             (0x10320309) // 400KHz
#define I2C_TIMEOUTR                (0x803F80C7) // Ttimeout=25ms Tlow:ext=8ms
#define SETTING_I2C_DEFAULT_ADDRESS (0xB0)

#define I2C1_EV_INT_PRE_PRIORITY    (5)
#define I2C1_EV_INT_SUB_PRIORITY    (0)
#define I2C1_ERROR_INT_PRE_PRIORITY (9)
#define I2C1_ERROR_INT_SUB_PRIORITY (0)

static I2C_HandleTypeDef hi2c1 = {0};

void PMBUS_I2C_Slave_Init(void)
{    
    hi2c1.Instance              = I2C1;
    hi2c1.Init.Timing           = I2C_TIMING_100K;
    hi2c1.Init.OwnAddress1      = SETTING_I2C_DEFAULT_ADDRESS;
    hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2      = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    (void)HAL_I2C_Init(&hi2c1);

    hi2c1.Instance->TIMEOUTR &= ~(I2C_TIMEOUTR_TEXTEN | I2C_TIMEOUTR_TIMOUTEN);
    hi2c1.Instance->TIMEOUTR  = I2C_TIMEOUTR;
    hi2c1.Instance->TIMEOUTR |= (I2C_TIMEOUTR_TEXTEN | I2C_TIMEOUTR_TIMOUTEN);

    __HAL_I2C_ENABLE_IT(&hi2c1,(I2C_IT_ADDRI |
                                I2C_IT_RXI |
                                I2C_IT_TXI |
                                I2C_IT_ERRI));

/** @brief  Enable the specified I2C interrupt.
  * @param  __HANDLE__ specifies the I2C Handle.
  * @param  __INTERRUPT__ specifies the interrupt source to enable.
  *        This parameter can be one of the following values:
  *            @arg @ref I2C_IT_ERRI  Errors interrupt enable
  *            @arg @ref I2C_IT_TCI   Transfer complete interrupt enable
  *            @arg @ref I2C_IT_STOPI STOP detection interrupt enable
  *            @arg @ref I2C_IT_NACKI NACK received interrupt enable
  *            @arg @ref I2C_IT_ADDRI Address match interrupt enable
  *            @arg @ref I2C_IT_RXI   RX interrupt enable
  *            @arg @ref I2C_IT_TXI   TX interrupt enable
  *
  * @retval None
  */
#define __HAL_I2C_ENABLE_IT(__HANDLE__, __INTERRUPT__)          ((__HANDLE__)->Instance->CR1 |= (__INTERRUPT__))

}

/* Init the low level hardware : GPIO, CLOCK, CORTEX...etc */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (hi2c->Instance == I2C1) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        GPIO_InitStruct.Pin         = GPIO_PIN_15; /* PA15 ------ I2C1_SCL */
        GPIO_InitStruct.Mode        = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull        = GPIO_NOPULL;
        GPIO_InitStruct.Speed       = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate   = GPIO_AF4_I2C1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        __HAL_RCC_GPIOB_CLK_ENABLE();
        GPIO_InitStruct.Pin         = GPIO_PIN_7; /* PB7 ------ I2C1_SDA */
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        __HAL_RCC_I2C1_CLK_ENABLE();

        HAL_NVIC_SetPriority(I2C1_EV_IRQn, I2C1_EV_INT_PRE_PRIORITY, I2C1_EV_INT_SUB_PRIORITY);
        HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
        HAL_NVIC_SetPriority(I2C1_ER_IRQn, I2C1_ERROR_INT_PRE_PRIORITY, I2C1_ERROR_INT_SUB_PRIORITY);
        HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
    }
}


void I2C1_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&hi2c1);
}
