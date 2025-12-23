#include "gpio.h"
#include "stm32g4xx.h"

void GPIO_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct = {0};

    /* Enable the GPIO Port Clock */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* GPIOB output pin initilize */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin   = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_9 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = ;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // ??
}
