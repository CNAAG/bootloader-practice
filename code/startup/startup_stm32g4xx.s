/********************************************************************************
 * @file    startup_stm32g474xx.s
 * @brief    STM32G474xx 设备向量表GCC工具链.
 *            ·设置SP栈指针
 *            ·设置PC（Program Counter）为 Reset_Handler
 *            ·将异常中断服务程序(ISR)地址设置为向量表条目
 *            ·配置时钟系统
 *            ·跳到Main函数
 *            重置后，Cortex-M4处理器处于线程模式，优先权为特权，堆栈设置为主
*******************************************************************************/

.syntax unified
.cpu cortex-m4
.fpu fpv4-sp-d16
.thumb

.global g_pfnVectors
.global Default_Handler

/* start address for the initialization values of the .data section. defined in linker script */
.word _sidata
/* start address for the .data section. defined in linker script */
.word _sdata
/* end address for the .data section. defined in linker script */
.word _edata
/* start address for the .bss section. defined in linker script */
.word _sbss
/* end address for the .bss section. defined in linker script */
.word _ebss
/* start address for the initialization values of the .ccmram section. defined in linker script */
.word _siccmram
/* start address for the .ccmram section. defined in linker script */
.word _sccmram
/* end address for the .ccmram section. defined in linker script */
.word _eccmram

.equ    BootRAM,    0xF1E0F85F
/**
 * @brief  This is the code that gets called when the processor first
 *          starts execution following a reset event. Only the absolutely
 *          necessary set is performed, after which the application
 *          supplied main() routine is called.
 * @param  None
 * @retval : None
*/

    .section    .text.Reset_Handler
    .weak    Reset_Handler
    .type    Reset_Handler, %function
Reset_Handler:
    /* Initialize stack pointer */
    ldr sp, =_estack

    /* Call the clock system initialization function.*/
    bl SystemInit

    /* Copy the data segment initializers from flash to SRAM */
    ldr r0, =_sdata
    ldr r1, =_edata
    ldr r2, =_sidata
    movs r3, #0
    b LoopCopyDataInit
CopyDataInit:
    ldr r4, [r2, r3]
    str r4, [r0, r3]
    adds r3, r3, #4
LoopCopyDataInit:
    adds r4, r0, r3
    cmp r4, r1
    bcc CopyDataInit

    /* Zero fill the bss segment. */
    ldr r2, =_sbss
    ldr r4, =_ebss
    movs r3, #0
    b LoopFillZerobss
FillZerobss:
    str r3, [r2]
    adds r2, r2, #4
LoopFillZerobss:
    cmp r2, r4
    bcc FillZerobss

    /* Copy CCM segment from flash to CCMRAM */
    ldr r0, =_sccmram
    ldr r1, =_eccmram
    ldr r2, =_siccmram
    movs r3, #0
    b LoopCopyCcmDataInit
CopyCcmDataInit:
    ldr r4, [r2, r3]
    str r4, [r0, r3]
    adds r3, r3, #4
LoopCopyCcmDataInit:
    adds r4, r0, r3
    cmp r4, r1
    bcc CopyCcmDataInit

    /* Call static constructors */
    bl __libc_init_array
    /* Call the application's entry point.*/
    bl main

LoopForever:
    b LoopForever

.size    Reset_Handler, .-Reset_Handler

/**
 * @brief  This is the code that gets called when the processor receives an
 *         unexpected interrupt.  This simply enters an infinite loop, preserving
 *         the system state for examination by a debugger.
 * @param  None
 * @retval : None
*/
    .section    .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
    b    Infinite_Loop
    .size    Default_Handler, .-Default_Handler


    .section    .isr_vector,"a",%progbits
    .type    g_pfnVectors, %object
g_pfnVectors:
    .word    _estack            /* Initial Stack Pointer */
    .word    Reset_Handler      /* Reset Handler */
    .word    NMI_Handler        /* NMI Handler */
    .word    HardFault_Handler  /* Hard Fault Handler */
    .word    MemManage_Handler  /* MPU Fault Handler */
    .word    BusFault_Handler   /* Bus Fault Handler */
    .word    UsageFault_Handler /* Usage Fault Handler */
    .word    0
    .word    0
    .word    0
    .word    0
    .word    SVC_Handler        /* SVCall Handler */
    .word    DebugMon_Handler   /* Debug Monitor Handler */
    .word    0
    .word    PendSV_Handler     /* PendSV Handler */
    .word    SysTick_Handler    /* SysTick Handler */
    /* External Interrupts */
    .word    WWDG_IRQHandler                /* Window WatchDog */
    .word    PVD_PVM_IRQHandler             /* PVD/PVM1/PVM2/PVM3/PVM4 through EXTI Line detection */
    .word    RTC_TAMP_LSECSS_IRQHandler     /* RTC, TAMP and RCC LSE_CSS through the EXTI line */
    .word    RTC_WKUP_IRQHandler            /* RTC Wakeup through the EXTI line */
    .word    FLASH_IRQHandler               /* FLASH */
    .word    RCC_IRQHandler                 /* RCC */
    .word    EXTI0_IRQHandler               /* EXTI Line0 */
    .word    EXTI1_IRQHandler               /* EXTI Line1 */
    .word    EXTI2_IRQHandler               /* EXTI Line2 */
    .word    EXTI3_IRQHandler               /* EXTI Line3 */
    .word    EXTI4_IRQHandler               /* EXTI Line4 */
    .word    DMA1_Channel1_IRQHandler       /* DMA1 Channel 1 */
    .word    DMA1_Channel2_IRQHandler       /* DMA1 Channel 2 */
    .word    DMA1_Channel3_IRQHandler       /* DMA1 Channel 3 */
    .word    DMA1_Channel4_IRQHandler       /* DMA1 Channel 4 */
    .word    DMA1_Channel5_IRQHandler       /* DMA1 Channel 5 */
    .word    DMA1_Channel6_IRQHandler       /* DMA1 Channel 6 */
    .word    DMA1_Channel7_IRQHandler       /* DMA1 Channel 7 */
    .word    ADC1_2_IRQHandler              /* ADC1 and ADC2 */
    .word    USB_HP_IRQHandler              /* USB Device High Priority */
    .word    USB_LP_IRQHandler              /* USB Device Low Priority */
    .word    FDCAN1_IT0_IRQHandler          /* FDCAN1 Interrupt line 0 */
    .word    FDCAN1_IT1_IRQHandler          /* FDCAN1 Interrupt line 1 */
    .word    EXTI9_5_IRQHandler             /* External Line[9:5]s */
    .word    TIM1_BRK_TIM15_IRQHandler      /* TIM1 Break and TIM15 */
    .word    TIM1_UP_TIM16_IRQHandler       /* TIM1 Update and TIM16 */
    .word    TIM1_TRG_COM_TIM17_IRQHandler  /* TIM1 Trigger and Commutation and TIM17 */
    .word    TIM1_CC_IRQHandler             /* TIM1 Capture Compare */
    .word    TIM2_IRQHandler                /* TIM2 */
    .word    TIM3_IRQHandler                /* TIM3 */
    .word    TIM4_IRQHandler                /* TIM4 */
    .word    I2C1_EV_IRQHandler             /* I2C1 Event */
    .word    I2C1_ER_IRQHandler             /* I2C1 Error */
    .word    I2C2_EV_IRQHandler             /* I2C2 Event */
    .word    I2C2_ER_IRQHandler             /* I2C2 Error */
    .word    SPI1_IRQHandler                /* SPI1 */
    .word    SPI2_IRQHandler                /* SPI2 */
    .word    USART1_IRQHandler              /* USART1 */
    .word    USART2_IRQHandler              /* USART2 */
    .word    USART3_IRQHandler              /* USART3 */
    .word    EXTI15_10_IRQHandler           /* External Line[15:10]s */
    .word    RTC_Alarm_IRQHandler           /* RTC Alarm (A and B) through EXTI line */
    .word    USBWakeUp_IRQHandler           /* USB Wakeup through EXTI line */
    .word    TIM8_BRK_IRQHandler            /* TIM8 Break */
    .word    TIM8_UP_IRQHandler             /* TIM8 Update */
    .word    TIM8_TRG_COM_IRQHandler        /* TIM8 Trigger and Commutation */
    .word    TIM8_CC_IRQHandler             /* TIM8 Capture Compare */
    .word    ADC3_IRQHandler                /* ADC3 */
    .word    FMC_IRQHandler                 /* FMC */
    .word    LPTIM1_IRQHandler              /* LPTIM1 */
    .word    TIM5_IRQHandler                /* TIM5 */
    .word    SPI3_IRQHandler                /* SPI3 */
    .word    UART4_IRQHandler               /* UART4 */
    .word    UART5_IRQHandler               /* UART5 */
    .word    TIM6_DAC_IRQHandler            /* TIM6 and DAC1&2 underrun errors */
    .word    TIM7_DAC_IRQHandler            /* TIM7 and DAC1&2 underrun errors */
    .word    DMA2_Channel1_IRQHandler       /* DMA2 Channel 1 */
    .word    DMA2_Channel2_IRQHandler       /* DMA2 Channel 2 */
    .word    DMA2_Channel3_IRQHandler       /* DMA2 Channel 3 */
    .word    DMA2_Channel4_IRQHandler       /* DMA2 Channel 4 */
    .word    DMA2_Channel5_IRQHandler       /* DMA2 Channel 5 */
    .word    ADC4_IRQHandler                /* ADC4 */
    .word    ADC5_IRQHandler                /* ADC5 */
    .word    UCPD1_IRQHandler               /* UCPD1 */
    .word    COMP1_2_3_IRQHandler           /* COMP1, COMP2 and COMP3 */
    .word    COMP4_5_6_IRQHandler           /* COMP4, COMP5 and COMP6 */
    .word    COMP7_IRQHandler               /* COMP7 */
    .word    HRTIM1_Master_IRQHandler       /* HRTIM1 Master */
    .word    HRTIM1_TIMA_IRQHandler         /* HRTIM1 Timer A */
    .word    HRTIM1_TIMB_IRQHandler         /* HRTIM1 Timer B */
    .word    HRTIM1_TIMC_IRQHandler         /* HRTIM1 Timer C */
    .word    HRTIM1_TIMD_IRQHandler         /* HRTIM1 Timer D */
    .word    HRTIM1_TIME_IRQHandler         /* HRTIM1 Timer E */
    .word    HRTIM1_FLT_IRQHandler          /* HRTIM1 Fault */
    .word    HRTIM1_TIMF_IRQHandler         /* HRTIM1 Timer F */
    .word    CRS_IRQHandler                 /* CRS */
    .word    SAI1_IRQHandler                /* SAI1 */
    .word    TIM20_BRK_IRQHandler           /* TIM20 Break */
    .word    TIM20_UP_IRQHandler            /* TIM20 Update */
    .word    TIM20_TRG_COM_IRQHandler       /* TIM20 Trigger and Commutation */
    .word    TIM20_CC_IRQHandler            /* TIM20 Capture Compare */
    .word    FPU_IRQHandler                 /* FPU */
    .word    I2C4_EV_IRQHandler             /* I2C4 Event */
    .word    I2C4_ER_IRQHandler             /* I2C4 Error */
    .word    SPI4_IRQHandler                /* SPI4 */
    .word    0
    .word    FDCAN2_IT0_IRQHandler          /* FDCAN2 Interrupt line 0 */
    .word    FDCAN2_IT1_IRQHandler          /* FDCAN2 Interrupt line 1 */
    .word    FDCAN3_IT0_IRQHandler          /* FDCAN3 Interrupt line 0 */
    .word    FDCAN3_IT1_IRQHandler          /* FDCAN3 Interrupt line 1 */
    .word    RNG_IRQHandler                 /* RNG */
    .word    LPUART1_IRQHandler             /* LPUART1 */
    .word    I2C3_EV_IRQHandler             /* I2C3 Event */
    .word    I2C3_ER_IRQHandler             /* I2C3 Error */
    .word    DMAMUX_OVR_IRQHandler          /* DMAMUX Overrun */
    .word    QUADSPI_IRQHandler             /* QUADSPI */
    .word    DMA1_Channel8_IRQHandler       /* DMA1 Channel 8 */
    .word    DMA2_Channel6_IRQHandler       /* DMA2 Channel 6 */
    .word    DMA2_Channel7_IRQHandler       /* DMA2 Channel 7 */
    .word    DMA2_Channel8_IRQHandler       /* DMA2 Channel 8 */
    .word    CORDIC_IRQHandler              /* CORDIC */
    .word    FMAC_IRQHandler                /* FMAC */

    .size    g_pfnVectors, .-g_pfnVectors

/*******************************************************************************
* Provide weak aliases for each Exception handler to the Default_Handler.
* As they are weak aliases, any function with the same name will override
* this definition.
*******************************************************************************/
    .weak    NMI_Handler
    .thumb_set NMI_Handler,Default_Handler

    .weak    HardFault_Handler
    .thumb_set HardFault_Handler,Default_Handler

    .weak    MemManage_Handler
    .thumb_set MemManage_Handler,Default_Handler

    .weak    BusFault_Handler
    .thumb_set BusFault_Handler,Default_Handler

    .weak    UsageFault_Handler
    .thumb_set UsageFault_Handler,Default_Handler

    .weak    SVC_Handler
    .thumb_set SVC_Handler,Default_Handler

    .weak    DebugMon_Handler
    .thumb_set DebugMon_Handler,Default_Handler

    .weak    PendSV_Handler
    .thumb_set PendSV_Handler,Default_Handler

    .weak    SysTick_Handler
    .thumb_set SysTick_Handler,Default_Handler

    .weak    WWDG_IRQHandler
    .thumb_set WWDG_IRQHandler,Default_Handler

    .weak    PVD_PVM_IRQHandler
    .thumb_set PVD_PVM_IRQHandler,Default_Handler

    .weak    RTC_TAMP_LSECSS_IRQHandler
    .thumb_set RTC_TAMP_LSECSS_IRQHandler,Default_Handler

    .weak    RTC_WKUP_IRQHandler
    .thumb_set RTC_WKUP_IRQHandler,Default_Handler

    .weak    FLASH_IRQHandler
    .thumb_set FLASH_IRQHandler,Default_Handler

    .weak    RCC_IRQHandler
    .thumb_set RCC_IRQHandler,Default_Handler

    .weak    EXTI0_IRQHandler
    .thumb_set EXTI0_IRQHandler,Default_Handler

    .weak    EXTI1_IRQHandler
    .thumb_set EXTI1_IRQHandler,Default_Handler

    .weak    EXTI2_IRQHandler
    .thumb_set EXTI2_IRQHandler,Default_Handler

    .weak    EXTI3_IRQHandler
    .thumb_set EXTI3_IRQHandler,Default_Handler

    .weak    EXTI4_IRQHandler
    .thumb_set EXTI4_IRQHandler,Default_Handler

    .weak    DMA1_Channel1_IRQHandler
    .thumb_set DMA1_Channel1_IRQHandler,Default_Handler

    .weak    DMA1_Channel2_IRQHandler
    .thumb_set DMA1_Channel2_IRQHandler,Default_Handler

    .weak    DMA1_Channel3_IRQHandler
    .thumb_set DMA1_Channel3_IRQHandler,Default_Handler

    .weak    DMA1_Channel4_IRQHandler
    .thumb_set DMA1_Channel4_IRQHandler,Default_Handler

    .weak    DMA1_Channel5_IRQHandler
    .thumb_set DMA1_Channel5_IRQHandler,Default_Handler

    .weak    DMA1_Channel6_IRQHandler
    .thumb_set DMA1_Channel6_IRQHandler,Default_Handler

    .weak    DMA1_Channel7_IRQHandler
    .thumb_set DMA1_Channel7_IRQHandler,Default_Handler

    .weak    ADC1_2_IRQHandler
    .thumb_set ADC1_2_IRQHandler,Default_Handler

    .weak    USB_HP_IRQHandler
    .thumb_set USB_HP_IRQHandler,Default_Handler

    .weak    USB_LP_IRQHandler
    .thumb_set USB_LP_IRQHandler,Default_Handler

    .weak    FDCAN1_IT0_IRQHandler
    .thumb_set FDCAN1_IT0_IRQHandler,Default_Handler

    .weak    FDCAN1_IT1_IRQHandler
    .thumb_set FDCAN1_IT1_IRQHandler,Default_Handler

    .weak    EXTI9_5_IRQHandler
    .thumb_set EXTI9_5_IRQHandler,Default_Handler

    .weak    TIM1_BRK_TIM15_IRQHandler
    .thumb_set TIM1_BRK_TIM15_IRQHandler,Default_Handler

    .weak    TIM1_UP_TIM16_IRQHandler
    .thumb_set TIM1_UP_TIM16_IRQHandler,Default_Handler

    .weak    TIM1_TRG_COM_TIM17_IRQHandler
    .thumb_set TIM1_TRG_COM_TIM17_IRQHandler,Default_Handler

    .weak    TIM1_CC_IRQHandler
    .thumb_set TIM1_CC_IRQHandler,Default_Handler

    .weak    TIM2_IRQHandler
    .thumb_set TIM2_IRQHandler,Default_Handler

    .weak    TIM3_IRQHandler
    .thumb_set TIM3_IRQHandler,Default_Handler

    .weak    TIM4_IRQHandler
    .thumb_set TIM4_IRQHandler,Default_Handler

    .weak    I2C1_EV_IRQHandler
    .thumb_set I2C1_EV_IRQHandler,Default_Handler

    .weak    I2C1_ER_IRQHandler
    .thumb_set I2C1_ER_IRQHandler,Default_Handler

    .weak    I2C2_EV_IRQHandler
    .thumb_set I2C2_EV_IRQHandler,Default_Handler

    .weak    I2C2_ER_IRQHandler
    .thumb_set I2C2_ER_IRQHandler,Default_Handler

    .weak    SPI1_IRQHandler
    .thumb_set SPI1_IRQHandler,Default_Handler

    .weak    SPI2_IRQHandler
    .thumb_set SPI2_IRQHandler,Default_Handler

    .weak    USART1_IRQHandler
    .thumb_set USART1_IRQHandler,Default_Handler

    .weak    USART2_IRQHandler
    .thumb_set USART2_IRQHandler,Default_Handler

    .weak    USART3_IRQHandler
    .thumb_set USART3_IRQHandler,Default_Handler

    .weak    EXTI15_10_IRQHandler
    .thumb_set EXTI15_10_IRQHandler,Default_Handler

    .weak    RTC_Alarm_IRQHandler
    .thumb_set RTC_Alarm_IRQHandler,Default_Handler

    .weak    USBWakeUp_IRQHandler
    .thumb_set USBWakeUp_IRQHandler,Default_Handler

    .weak    TIM8_BRK_IRQHandler
    .thumb_set TIM8_BRK_IRQHandler,Default_Handler

    .weak    TIM8_UP_IRQHandler
    .thumb_set TIM8_UP_IRQHandler,Default_Handler

    .weak    TIM8_TRG_COM_IRQHandler
    .thumb_set TIM8_TRG_COM_IRQHandler,Default_Handler

    .weak    TIM8_CC_IRQHandler
    .thumb_set TIM8_CC_IRQHandler,Default_Handler

    .weak    ADC3_IRQHandler
    .thumb_set ADC3_IRQHandler,Default_Handler

    .weak    FMC_IRQHandler
    .thumb_set FMC_IRQHandler,Default_Handler

    .weak    LPTIM1_IRQHandler
    .thumb_set LPTIM1_IRQHandler,Default_Handler

    .weak    TIM5_IRQHandler
    .thumb_set TIM5_IRQHandler,Default_Handler

    .weak    SPI3_IRQHandler
    .thumb_set SPI3_IRQHandler,Default_Handler

    .weak    UART4_IRQHandler
    .thumb_set UART4_IRQHandler,Default_Handler

    .weak    UART5_IRQHandler
    .thumb_set UART5_IRQHandler,Default_Handler

    .weak    TIM6_DAC_IRQHandler
    .thumb_set TIM6_DAC_IRQHandler,Default_Handler

    .weak    TIM7_DAC_IRQHandler
    .thumb_set TIM7_DAC_IRQHandler,Default_Handler

    .weak    DMA2_Channel1_IRQHandler
    .thumb_set DMA2_Channel1_IRQHandler,Default_Handler

    .weak    DMA2_Channel2_IRQHandler
    .thumb_set DMA2_Channel2_IRQHandler,Default_Handler

    .weak    DMA2_Channel3_IRQHandler
    .thumb_set DMA2_Channel3_IRQHandler,Default_Handler

    .weak    DMA2_Channel4_IRQHandler
    .thumb_set DMA2_Channel4_IRQHandler,Default_Handler

    .weak    DMA2_Channel5_IRQHandler
    .thumb_set DMA2_Channel5_IRQHandler,Default_Handler

    .weak    ADC4_IRQHandler
    .thumb_set ADC4_IRQHandler,Default_Handler

    .weak    ADC5_IRQHandler
    .thumb_set ADC5_IRQHandler,Default_Handler

    .weak    UCPD1_IRQHandler
    .thumb_set UCPD1_IRQHandler,Default_Handler

    .weak    COMP1_2_3_IRQHandler
    .thumb_set COMP1_2_3_IRQHandler,Default_Handler

    .weak    COMP4_5_6_IRQHandler
    .thumb_set COMP4_5_6_IRQHandler,Default_Handler

    .weak    COMP7_IRQHandler
    .thumb_set COMP7_IRQHandler,Default_Handler

    .weak    HRTIM1_Master_IRQHandler
    .thumb_set HRTIM1_Master_IRQHandler,Default_Handler

    .weak    HRTIM1_TIMA_IRQHandler
    .thumb_set HRTIM1_TIMA_IRQHandler,Default_Handler

    .weak    HRTIM1_TIMB_IRQHandler
    .thumb_set HRTIM1_TIMB_IRQHandler,Default_Handler

    .weak    HRTIM1_TIMC_IRQHandler
    .thumb_set HRTIM1_TIMC_IRQHandler,Default_Handler

    .weak    HRTIM1_TIMD_IRQHandler
    .thumb_set HRTIM1_TIMD_IRQHandler,Default_Handler

    .weak    HRTIM1_TIME_IRQHandler
    .thumb_set HRTIM1_TIME_IRQHandler,Default_Handler

    .weak    HRTIM1_FLT_IRQHandler
    .thumb_set HRTIM1_FLT_IRQHandler,Default_Handler

    .weak    HRTIM1_TIMF_IRQHandler
    .thumb_set HRTIM1_TIMF_IRQHandler,Default_Handler

    .weak    CRS_IRQHandler
    .thumb_set CRS_IRQHandler,Default_Handler

    .weak    SAI1_IRQHandler
    .thumb_set SAI1_IRQHandler,Default_Handler

    .weak    TIM20_BRK_IRQHandler
    .thumb_set TIM20_BRK_IRQHandler,Default_Handler

    .weak    TIM20_UP_IRQHandler
    .thumb_set TIM20_UP_IRQHandler,Default_Handler

    .weak    TIM20_TRG_COM_IRQHandler
    .thumb_set TIM20_TRG_COM_IRQHandler,Default_Handler

    .weak    TIM20_CC_IRQHandler
    .thumb_set TIM20_CC_IRQHandler,Default_Handler

    .weak    FPU_IRQHandler
    .thumb_set FPU_IRQHandler,Default_Handler

    .weak    I2C4_EV_IRQHandler
    .thumb_set I2C4_EV_IRQHandler,Default_Handler

    .weak    I2C4_ER_IRQHandler
    .thumb_set I2C4_ER_IRQHandler,Default_Handler

    .weak    SPI4_IRQHandler
    .thumb_set SPI4_IRQHandler,Default_Handler

    .weak    FDCAN2_IT0_IRQHandler
    .thumb_set FDCAN2_IT0_IRQHandler,Default_Handler

    .weak    FDCAN2_IT1_IRQHandler
    .thumb_set FDCAN2_IT1_IRQHandler,Default_Handler

    .weak    FDCAN3_IT0_IRQHandler
    .thumb_set FDCAN3_IT0_IRQHandler,Default_Handler

    .weak    FDCAN3_IT1_IRQHandler
    .thumb_set FDCAN3_IT1_IRQHandler,Default_Handler

    .weak    RNG_IRQHandler
    .thumb_set RNG_IRQHandler,Default_Handler

    .weak    LPUART1_IRQHandler
    .thumb_set LPUART1_IRQHandler,Default_Handler

    .weak    I2C3_EV_IRQHandler
    .thumb_set I2C3_EV_IRQHandler,Default_Handler

    .weak    I2C3_ER_IRQHandler
    .thumb_set I2C3_ER_IRQHandler,Default_Handler

    .weak    DMAMUX_OVR_IRQHandler
    .thumb_set DMAMUX_OVR_IRQHandler,Default_Handler

    .weak    QUADSPI_IRQHandler
    .thumb_set QUADSPI_IRQHandler,Default_Handler

    .weak    DMA1_Channel8_IRQHandler
    .thumb_set DMA1_Channel8_IRQHandler,Default_Handler

    .weak    DMA2_Channel6_IRQHandler
    .thumb_set DMA2_Channel6_IRQHandler,Default_Handler

    .weak    DMA2_Channel7_IRQHandler
    .thumb_set DMA2_Channel7_IRQHandler,Default_Handler

    .weak    DMA2_Channel8_IRQHandler
    .thumb_set DMA2_Channel8_IRQHandler,Default_Handler

    .weak    CORDIC_IRQHandler
    .thumb_set CORDIC_IRQHandler,Default_Handler

    .weak    FMAC_IRQHandler
    .thumb_set FMAC_IRQHandler,Default_Handler

