#include <stdint.h>
#include "stm32g4xx.h"

volatile uint32_t g_nmi_cause = 0;
volatile uint32_t g_nmi_ecc_addr = 0;
void NMI_Handler(void)
{
    uint32_t cause = 0;

    /* 1. 时钟安全系统(CSS)触发的 NMI */
    if (RCC->CIFR & RCC_CIFR_CSSF) { // set by hardware when a failure is detected in the HSE oscillator
        RCC->CR |= RCC_CR_HSION; // HSI16 Oscillator ON
        while (!(RCC->CR & RCC_CR_HSIRDY)) { // check HSI16 clock if raedy
            /**/
        }
        
        RCC->CICR |= RCC_CICR_CSSC; // clear CSSF(clock security system interrupt clear)
        cause |= 1 << 0;
    }

    /* 2. Flash ECC 双错触发的 NMI */
    if (FALSH->ECCD & FLASH_SR_ECCD) { // set by hardware when two ECC errors have been detected (only if ECCC/ECCD are previously cleared). When this bit is set, a NMI is generated.
        g_nmi_ecc_addr = FLASH->ECCR & FLASH_ECCR_ADDR_ECC; // ECC fail address
        FLASH->ECCD |= FLASH_SR_ECCD; // cleared by writing 1
        cause |= 1 << 1;
    }

    /* 3. 把原因记下来，主循环或看门狗复位前可查看 */
    g_nmi_cause = cause;

    /* 4. 立即返回，绝不 while(1) 死等。若不清标志就死等，NMI线一直被锁，后续任何NMI再也进不来 */
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

