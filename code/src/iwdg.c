#include "iwdg.h"
#include "stm32g4xx.h"

static IWDG_HandleTypeDef IWDG_Handler = {0};

void IWDG_Init(void)
{
    IWDG_Handler.Instance = IWDG;
    IWDG_Handler.Init.Prescaler = IWDG_PRESCALER_128;
    IWDG_Handler.Init.Reload = 500; // 2s / (32KHZ / 128) = 500
    IWDG_Handler.Init.Window = IWDG_WINDOW_DISABLE;
    (void)HAL_IWDG_Init(&IWDG_Handler);
}

void IWDG_Refresh(void)
{   
    (void)HAL_IWDG_Refresh(&IWDG_Handler);
}

void IWDG_Start(void)
{   
    __HAL_IWDG_START(&IWDG_Handler);
}
