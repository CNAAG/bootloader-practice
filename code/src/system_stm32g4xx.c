#include "stm32g4xx.h"
#include "stm32g4xx_hal_conf.h"

#if defined(VECT_TAB_SRAM)
    #define VECT_TAB_BASE_ADDRESS   SRAM_BASE       /*!< Vector Table base address field. This value must be a multiple of 0x200. */
    #define VECT_TAB_OFFSET         0x00000000U     /*!< Vector Table base offset field. This value must be a multiple of 0x200. */
#else
    #define VECT_TAB_BASE_ADDRESS   FLASH_BASE      /*!< Vector Table base address field. This value must be a multiple of 0x200. */
    #define VECT_TAB_OFFSET         0x00000000U     /*!< Vector Table base offset field. This value must be a multiple of 0x200. */
#endif /* VECT_TAB_SRAM */


/* The SystemCoreClock variable is updated in three ways:
    1) by calling CMSIS function SystemCoreClockUpdate()
    2) by calling HAL API function HAL_RCC_GetHCLKFreq()
    3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
        Note: If you use this function to configure the system clock; then there
            is no need to call the 2 first functions listed above, since SystemCoreClock
            variable is updated automatically.
*/
uint32_t SystemCoreClock = HSI_VALUE;

const uint8_t AHBPrescTable[16] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U, 6U, 7U, 8U, 9U};
const uint8_t APBPrescTable[8] =  {0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U};

void SystemInit(void) {
    /* FPU settings ------------------------------------------------------------*/
    #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << (10*2))|(3UL << (11*2))); /* set CP10 and CP11 Full Access */
    #endif

    /* Configure the Vector Table location add offset address ------------------*/
    SCB->VTOR = VECT_TAB_BASE_ADDRESS | VECT_TAB_OFFSET;
}

void SystemCoreClockUpdate(void) {
    uint32_t tmp, pllvco, pllr, pllsource, pllm;

    /* Get SYSCLK source -------------------------------------------------------*/
    switch (RCC->CFGR & RCC_CFGR_SWS) {
        case 0x04:  /* HSI used as system clock source */
            SystemCoreClock = HSI_VALUE;
        break;
        case 0x08:  /* HSE used as system clock source */
            SystemCoreClock = HSE_VALUE;
        break;
        case 0x0C:  /* PLL used as system clock source */
            /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLLM) * PLLN */
            /* SYSCLK = PLL_VCO / PLLR */
            pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC);
            pllm = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLM) >> 4) + 1U;
            if (pllsource == 0x02UL) { /* HSI used as PLL clock source */
                pllvco = (HSI_VALUE / pllm);
            } else { /* HSE used as PLL clock source */
                pllvco = (HSE_VALUE / pllm);
            }
            pllvco = pllvco * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 8);
            pllr = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLR) >> 25) + 1U) * 2U;
            SystemCoreClock = pllvco/pllr;
        break;
        default:
        break;
    }

    /* Compute HCLK clock frequency --------------------------------------------*/
    /* Get HCLK prescaler */
    tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
    /* HCLK clock frequency */
    SystemCoreClock >>= tmp;
}