#ifndef STM32G4xx_HAL_CONF_H
#define STM32G4xx_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* ########################## Module Selection ############################## */
#define HAL_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
#define HAL_COMP_MODULE_ENABLED
#define HAL_CORDIC_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_CRC_MODULE_ENABLED
#define HAL_CRYP_MODULE_ENABLED
#define HAL_DAC_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_EXTI_MODULE_ENABLED
#define HAL_FDCAN_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_FMAC_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_HRTIM_MODULE_ENABLED
#define HAL_IRDA_MODULE_ENABLED
#define HAL_IWDG_MODULE_ENABLED
#define HAL_I2C_MODULE_ENABLED
#define HAL_I2S_MODULE_ENABLED
#define HAL_LPTIM_MODULE_ENABLED
#define HAL_NAND_MODULE_ENABLED
#define HAL_NOR_MODULE_ENABLED
#define HAL_OPAMP_MODULE_ENABLED
#define HAL_PCD_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_QSPI_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_RNG_MODULE_ENABLED
#define HAL_RTC_MODULE_ENABLED
#define HAL_SAI_MODULE_ENABLED
#define HAL_SMARTCARD_MODULE_ENABLED
#define HAL_SMBUS_MODULE_ENABLED
#define HAL_SPI_MODULE_ENABLED
#define HAL_SRAM_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED
#define HAL_USART_MODULE_ENABLED
#define HAL_WWDG_MODULE_ENABLED

/* ########################## Register Callbacks selection ############################## */
#define USE_HAL_ADC_REGISTER_CALLBACKS        0U
#define USE_HAL_COMP_REGISTER_CALLBACKS       0U
#define USE_HAL_CORDIC_REGISTER_CALLBACKS     0U
#define USE_HAL_CRYP_REGISTER_CALLBACKS       0U
#define USE_HAL_DAC_REGISTER_CALLBACKS        0U
#define USE_HAL_EXTI_REGISTER_CALLBACKS       0U
#define USE_HAL_FDCAN_REGISTER_CALLBACKS      0U
#define USE_HAL_FMAC_REGISTER_CALLBACKS       0U
#define USE_HAL_HRTIM_REGISTER_CALLBACKS      0U
#define USE_HAL_I2C_REGISTER_CALLBACKS        0U
#define USE_HAL_I2S_REGISTER_CALLBACKS        0U
#define USE_HAL_IRDA_REGISTER_CALLBACKS       0U
#define USE_HAL_LPTIM_REGISTER_CALLBACKS      0U
#define USE_HAL_NAND_REGISTER_CALLBACKS       0U
#define USE_HAL_NOR_REGISTER_CALLBACKS        0U
#define USE_HAL_OPAMP_REGISTER_CALLBACKS      0U
#define USE_HAL_PCD_REGISTER_CALLBACKS        0U
#define USE_HAL_QSPI_REGISTER_CALLBACKS       0U
#define USE_HAL_RNG_REGISTER_CALLBACKS        0U
#define USE_HAL_RTC_REGISTER_CALLBACKS        0U
#define USE_HAL_SAI_REGISTER_CALLBACKS        0U
#define USE_HAL_SMARTCARD_REGISTER_CALLBACKS  0U
#define USE_HAL_SMBUS_REGISTER_CALLBACKS      0U
#define USE_HAL_SPI_REGISTER_CALLBACKS        0U
#define USE_HAL_SRAM_REGISTER_CALLBACKS       0U
#define USE_HAL_TIM_REGISTER_CALLBACKS        0U
#define USE_HAL_UART_REGISTER_CALLBACKS       0U
#define USE_HAL_USART_REGISTER_CALLBACKS      0U
#define USE_HAL_WWDG_REGISTER_CALLBACKS       0U

/* ########################## Oscillator Values adaptation ####################*/
#if !defined  (HSE_VALUE)
#define HSE_VALUE (8000000UL)           /*!< Value of the External oscillator in Hz */
#endif

#if !defined  (HSE_STARTUP_TIMEOUT)
#define HSE_STARTUP_TIMEOUT (100UL)     /*!< Time out for HSE start up, in ms */
#endif

#if !defined  (HSI_VALUE)
#define HSI_VALUE (16000000UL)          /*!< Value of the Internal oscillator in Hz*/
#endif

#if !defined  (HSI48_VALUE)
#define HSI48_VALUE (48000000UL)        /*!< Value of the Internal High Speed oscillator for USB FS/RNG in Hz. The real value my vary depending on manufacturing process variations.*/
#endif

#if !defined  (LSI_VALUE)
#define LSI_VALUE (32000UL)             /*!< Value of the Internal Low Speed oscillator in Hz. The real value may vary depending on the variations in voltage and temperature.*/
#endif

#if !defined  (LSE_VALUE)
#define LSE_VALUE (32768UL)             /*!< Value of the External Low Speed oscillator in Hz. This value is used by the UART, RTC HAL module to compute the system frequency */
#endif

#if !defined  (LSE_STARTUP_TIMEOUT)
#define LSE_STARTUP_TIMEOUT (5000UL)    /*!< Time out for LSE start up, in ms */
#endif

#if !defined  (EXTERNAL_CLOCK_VALUE)
#define EXTERNAL_CLOCK_VALUE (48000UL)  /*!< This value is used by the I2S and SAI HAL modules to compute the I2S and SAI clock source frequency, this source is inserted directly through I2S_CKIN pad.*/
#endif

/* Tip: To avoid modifying this file each time you need to use different HSE, you can define the HSE value in your toolchain compiler preprocessor. */

/* ########################### System Configuration ######################### */
/* This is the HAL system configuration section */
#define  VDD_VALUE                  (3300UL)    /*!< Value of VDD in mv */
#define  TICK_INT_PRIORITY          (0UL)       /*!< tick interrupt priority */
#define  USE_RTOS                   0U
#define  PREFETCH_ENABLE            1U
#define  INSTRUCTION_CACHE_ENABLE   1U
#define  DATA_CACHE_ENABLE          1U

/* ########################## Assert Selection ############################## */
/* Uncomment the line below to expanse the "assert_param" macro in the HAL drivers code */
#define USE_FULL_ASSERT               1U

/* ################## SPI peripheral configuration ########################## */
/* CRC FEATURE: Use to activate CRC feature inside HAL SPI Driver
 * Activated: CRC code is present inside driver
 * Deactivated: CRC code cleaned from driver
 */
#define USE_SPI_CRC                   1U

/* Includes ------------------------------------------------------------------*/
#ifdef HAL_RCC_MODULE_ENABLED
#include "stm32g4xx_hal_rcc.h"
#endif

#ifdef HAL_GPIO_MODULE_ENABLED
#include "stm32g4xx_hal_gpio.h"
#endif

#ifdef HAL_DMA_MODULE_ENABLED
#include "stm32g4xx_hal_dma.h"
#endif

#ifdef HAL_CORTEX_MODULE_ENABLED
#include "stm32g4xx_hal_cortex.h"
#endif

#ifdef HAL_ADC_MODULE_ENABLED
#include "stm32g4xx_hal_adc.h"
#endif

#ifdef HAL_COMP_MODULE_ENABLED
#include "stm32g4xx_hal_comp.h"
#endif

#ifdef HAL_CORDIC_MODULE_ENABLED
#include "stm32g4xx_hal_cordic.h"
#endif

#ifdef HAL_CRC_MODULE_ENABLED
#include "stm32g4xx_hal_crc.h"
#endif

#ifdef HAL_CRYP_MODULE_ENABLED
#include "stm32g4xx_hal_cryp.h"
#endif

#ifdef HAL_DAC_MODULE_ENABLED
#include "stm32g4xx_hal_dac.h"
#endif

#ifdef HAL_EXTI_MODULE_ENABLED
#include "stm32g4xx_hal_exti.h"
#endif

#ifdef HAL_FDCAN_MODULE_ENABLED
#include "stm32g4xx_hal_fdcan.h"
#endif

#ifdef HAL_FLASH_MODULE_ENABLED
#include "stm32g4xx_hal_flash.h"
#endif

#ifdef HAL_FMAC_MODULE_ENABLED
#include "stm32g4xx_hal_fmac.h"
#endif

#ifdef HAL_HRTIM_MODULE_ENABLED
#include "stm32g4xx_hal_hrtim.h"
#endif

#ifdef HAL_IRDA_MODULE_ENABLED
#include "stm32g4xx_hal_irda.h"
#endif

#ifdef HAL_IWDG_MODULE_ENABLED
#include "stm32g4xx_hal_iwdg.h"
#endif

#ifdef HAL_I2C_MODULE_ENABLED
#include "stm32g4xx_hal_i2c.h"
#endif

#ifdef HAL_I2S_MODULE_ENABLED
#include "stm32g4xx_hal_i2s.h"
#endif

#ifdef HAL_LPTIM_MODULE_ENABLED
#include "stm32g4xx_hal_lptim.h"
#endif

#ifdef HAL_NAND_MODULE_ENABLED
#include "stm32g4xx_hal_nand.h"
#endif

#ifdef HAL_NOR_MODULE_ENABLED
#include "stm32g4xx_hal_nor.h"
#endif

#ifdef HAL_OPAMP_MODULE_ENABLED
#include "stm32g4xx_hal_opamp.h"
#endif

#ifdef HAL_PCD_MODULE_ENABLED
#include "stm32g4xx_hal_pcd.h"
#endif

#ifdef HAL_PWR_MODULE_ENABLED
#include "stm32g4xx_hal_pwr.h"
#endif

#ifdef HAL_QSPI_MODULE_ENABLED
#include "stm32g4xx_hal_qspi.h"
#endif

#ifdef HAL_RNG_MODULE_ENABLED
#include "stm32g4xx_hal_rng.h"
#endif

#ifdef HAL_RTC_MODULE_ENABLED
#include "stm32g4xx_hal_rtc.h"
#endif

#ifdef HAL_SAI_MODULE_ENABLED
#include "stm32g4xx_hal_sai.h"
#endif

#ifdef HAL_SMARTCARD_MODULE_ENABLED
#include "stm32g4xx_hal_smartcard.h"
#endif

#ifdef HAL_SMBUS_MODULE_ENABLED
#include "stm32g4xx_hal_smbus.h"
#endif

#ifdef HAL_SPI_MODULE_ENABLED
#include "stm32g4xx_hal_spi.h"
#endif

#ifdef HAL_SRAM_MODULE_ENABLED
#include "stm32g4xx_hal_sram.h"
#endif

#ifdef HAL_TIM_MODULE_ENABLED
#include "stm32g4xx_hal_tim.h"
#endif

#ifdef HAL_UART_MODULE_ENABLED
#include "stm32g4xx_hal_uart.h"
#endif

#ifdef HAL_USART_MODULE_ENABLED
#include "stm32g4xx_hal_usart.h"
#endif

#ifdef HAL_WWDG_MODULE_ENABLED
#include "stm32g4xx_hal_wwdg.h"
#endif

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT
/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function
  *         which reports the name of the source file and the source
  *         line number of the call that failed.
  *         If expr is true, it returns no value.
  * @retval None
  */
#define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
void assert_failed(uint8_t *file, uint32_t line);
#else
#define assert_param(expr) ((void)0U)
#endif

#ifdef __cplusplus
}
#endif

#endif /* STM32G4xx_HAL_CONF_H */