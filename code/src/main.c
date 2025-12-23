#include "stm32g4xx.h"
#include "system_clock_config.h"
#include "iwdg.h"

void Initialization(void)
{
    __disable_irq();
    HAL_Init();
    SystemClock_Config();
    IWDG_Init();
    GPIO_Init();
    I2C_Slave_Init();

    __enable_irq();
}


int main(void) {
    Initialization();

    return 0;
}