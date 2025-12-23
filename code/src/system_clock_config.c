#include "system_clock_config.h"
#include "stm32g4xx.h"

void Error_Handler(void)
{
    while (1) {
        /* ...... */
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
    RCC_PeriphCLKInitTypeDef pclk = {0};

    /* Configure the main internal regulator output voltage */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    /* Initializes the RCC Oscillators according to the specified parameters in the RCC_OscInitTypeDef structure. */
    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI /* | RCC_OSCILLATORTYPE_HSE */;
    osc.HSEState            = RCC_HSE_OFF; // 暂时屏蔽 RCC_HSE_ON 待电路更新
    osc.LSEState            = RCC_LSE_OFF;
    osc.HSIState            = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.LSIState            = RCC_LSI_ON;
    osc.HSI48State          = RCC_HSI48_OFF;
    osc.PLL.PLLState    = RCC_PLL_ON;
    osc.PLL.PLLSource   = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM        = RCC_PLLM_DIV4;
    osc.PLL.PLLN        = 85;
    osc.PLL.PLLP        = RCC_PLLP_DIV2;
    osc.PLL.PLLQ        = RCC_PLLQ_DIV2;
    osc.PLL.PLLR        = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        Error_Handler();
    }

    /* Initializes the CPU, AHB and APB buses clocks */
    clk.ClockType       = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource    = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider   = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider  = RCC_HCLK_DIV1;
    clk.APB2CLKDivider  = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }

    /* Initializes the Usart I2C */
    pclk.PeriphClockSelection   = RCC_PERIPHCLK_USART3 | RCC_PERIPHCLK_I2C1;
    pclk.Usart3ClockSelection   = RCC_USART3CLKSOURCE_PCLK1;
	pclk.I2c1ClockSelection     = RCC_I2C1CLKSOURCE_HSI;
    HAL_RCCEx_PeriphCLKConfig(&pclk);

    /* Enable the Clock Security System */
	HAL_RCC_EnableCSS();
}