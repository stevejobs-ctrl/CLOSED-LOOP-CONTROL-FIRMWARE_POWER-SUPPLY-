/*
 * startup_stm32f103c8t6.c
 *
 *  Created on: Mar 30, 2026
 *      Author: USER
 */
/*
 * startup_stm32f103c8t6.c
 *
 * Provides:
 *   – Stack pointer initialisation
 *   – Vector table (placed in .isr_vector section → Flash 0x08000000)
 *   – Reset_Handler: copies .data, zeroes .bss, calls SystemInit + main
 *
 * All handlers default to an infinite loop; override in your own .c files
 * by defining a function with the same name (weak linkage allows this).
 */

#include <stdint.h>

/* ── Symbols from the linker script ─────────────────────────────────────── */
extern uint32_t _estack;    /* top of SRAM stack                             */
extern uint32_t _sidata;    /* start of .data load address (in Flash)        */
extern uint32_t _sdata;     /* start of .data run address  (in SRAM)         */
extern uint32_t _edata;     /* end   of .data run address  (in SRAM)         */
extern uint32_t _sbss;      /* start of .bss                                 */
extern uint32_t _ebss;      /* end   of .bss                                 */

/* ── Forward declarations of handlers ──────────────────────────────────── */
void Reset_Handler(void);
void Default_Handler(void);

/* CMSIS-provided handlers we use */
extern void SysTick_Handler(void);
extern void DMA1_Channel1_IRQHandler(void);

/* SystemInit from CMSIS (optional; safe NOP if not linked)                  */
extern void SystemInit(void) __attribute__((weak));

/* ── Default weak handlers (redirect to infinite loop) ─────────────────── */
#define WEAK_DEFAULT __attribute__((weak, alias("Default_Handler")))

void NMI_Handler(void)                WEAK_DEFAULT;
void HardFault_Handler(void)          WEAK_DEFAULT;
void MemManage_Handler(void)          WEAK_DEFAULT;
void BusFault_Handler(void)           WEAK_DEFAULT;
void UsageFault_Handler(void)         WEAK_DEFAULT;
void SVC_Handler(void)                WEAK_DEFAULT;
void DebugMon_Handler(void)           WEAK_DEFAULT;
void PendSV_Handler(void)             WEAK_DEFAULT;

/* Peripheral IRQs — all default to Default_Handler */
void WWDG_IRQHandler(void)            WEAK_DEFAULT;
void PVD_IRQHandler(void)             WEAK_DEFAULT;
void TAMPER_IRQHandler(void)          WEAK_DEFAULT;
void RTC_IRQHandler(void)             WEAK_DEFAULT;
void FLASH_IRQHandler(void)           WEAK_DEFAULT;
void RCC_IRQHandler(void)             WEAK_DEFAULT;
void EXTI0_IRQHandler(void)           WEAK_DEFAULT;
void EXTI1_IRQHandler(void)           WEAK_DEFAULT;
void EXTI2_IRQHandler(void)           WEAK_DEFAULT;
void EXTI3_IRQHandler(void)           WEAK_DEFAULT;
void EXTI4_IRQHandler(void)           WEAK_DEFAULT;
/* DMA1_Channel1_IRQHandler defined in main.c */
void DMA1_Channel2_IRQHandler(void)   WEAK_DEFAULT;
void DMA1_Channel3_IRQHandler(void)   WEAK_DEFAULT;
void DMA1_Channel4_IRQHandler(void)   WEAK_DEFAULT;
void DMA1_Channel5_IRQHandler(void)   WEAK_DEFAULT;
void DMA1_Channel6_IRQHandler(void)   WEAK_DEFAULT;
void DMA1_Channel7_IRQHandler(void)   WEAK_DEFAULT;
void ADC1_2_IRQHandler(void)          WEAK_DEFAULT;
void USB_HP_CAN1_TX_IRQHandler(void)  WEAK_DEFAULT;
void USB_LP_CAN1_RX0_IRQHandler(void) WEAK_DEFAULT;
void CAN1_RX1_IRQHandler(void)        WEAK_DEFAULT;
void CAN1_SCE_IRQHandler(void)        WEAK_DEFAULT;
void EXTI9_5_IRQHandler(void)         WEAK_DEFAULT;
void TIM1_BRK_IRQHandler(void)        WEAK_DEFAULT;
void TIM1_UP_IRQHandler(void)         WEAK_DEFAULT;
void TIM1_TRG_COM_IRQHandler(void)    WEAK_DEFAULT;
void TIM1_CC_IRQHandler(void)         WEAK_DEFAULT;
void TIM2_IRQHandler(void)            WEAK_DEFAULT;
void TIM3_IRQHandler(void)            WEAK_DEFAULT;
void TIM4_IRQHandler(void)            WEAK_DEFAULT;
void I2C1_EV_IRQHandler(void)         WEAK_DEFAULT;
void I2C1_ER_IRQHandler(void)         WEAK_DEFAULT;
void I2C2_EV_IRQHandler(void)         WEAK_DEFAULT;
void I2C2_ER_IRQHandler(void)         WEAK_DEFAULT;
void SPI1_IRQHandler(void)            WEAK_DEFAULT;
void SPI2_IRQHandler(void)            WEAK_DEFAULT;
void USART1_IRQHandler(void)          WEAK_DEFAULT;
void USART2_IRQHandler(void)          WEAK_DEFAULT;
void USART3_IRQHandler(void)          WEAK_DEFAULT;
void EXTI15_10_IRQHandler(void)       WEAK_DEFAULT;
void RTC_Alarm_IRQHandler(void)       WEAK_DEFAULT;
void USBWakeUp_IRQHandler(void)       WEAK_DEFAULT;

/* ── Vector table ────────────────────────────────────────────────────────── */
/*
 * Placed in .isr_vector which the linker script maps to the very start of
 * Flash (0x08000000).  The Cortex-M3 boot ROM reads:
 *   [0] = initial MSP value  (top of stack)
 *   [1] = Reset_Handler address
 *   [2…] = exception / IRQ handlers
 */
typedef void (*vector_fn)(void);

__attribute__((section(".isr_vector")))
const vector_fn vector_table[] =
{
    /* Stack pointer initial value */
    (vector_fn)&_estack,

    /* Cortex-M3 core exceptions */
    Reset_Handler,          /* 1  Reset                                      */
    NMI_Handler,            /* 2  NMI                                        */
    HardFault_Handler,      /* 3  Hard Fault                                 */
    MemManage_Handler,      /* 4  MPU Fault                                  */
    BusFault_Handler,       /* 5  Bus Fault                                  */
    UsageFault_Handler,     /* 6  Usage Fault                                */
    0, 0, 0, 0,             /* 7–10  Reserved                                */
    SVC_Handler,            /* 11 SVCall                                     */
    DebugMon_Handler,       /* 12 Debug Monitor                              */
    0,                      /* 13 Reserved                                   */
    PendSV_Handler,         /* 14 PendSV                                     */
    SysTick_Handler,        /* 15 SysTick                                    */

    /* STM32F103 peripheral IRQs (position 0 = IRQ0) */
    WWDG_IRQHandler,            /* 0  Window Watchdog                        */
    PVD_IRQHandler,             /* 1  PVD via EXTI                           */
    TAMPER_IRQHandler,          /* 2  Tamper                                 */
    RTC_IRQHandler,             /* 3  RTC global                             */
    FLASH_IRQHandler,           /* 4  Flash global                           */
    RCC_IRQHandler,             /* 5  RCC global                             */
    EXTI0_IRQHandler,           /* 6  EXTI0                                  */
    EXTI1_IRQHandler,           /* 7  EXTI1                                  */
    EXTI2_IRQHandler,           /* 8  EXTI2                                  */
    EXTI3_IRQHandler,           /* 9  EXTI3                                  */
    EXTI4_IRQHandler,           /* 10 EXTI4                                  */
    DMA1_Channel1_IRQHandler,   /* 11 DMA1 Ch1  ← ADC scan complete          */
    DMA1_Channel2_IRQHandler,   /* 12 DMA1 Ch2                               */
    DMA1_Channel3_IRQHandler,   /* 13 DMA1 Ch3                               */
    DMA1_Channel4_IRQHandler,   /* 14 DMA1 Ch4                               */
    DMA1_Channel5_IRQHandler,   /* 15 DMA1 Ch5                               */
    DMA1_Channel6_IRQHandler,   /* 16 DMA1 Ch6                               */
    DMA1_Channel7_IRQHandler,   /* 17 DMA1 Ch7                               */
    ADC1_2_IRQHandler,          /* 18 ADC1 & ADC2                            */
    USB_HP_CAN1_TX_IRQHandler,  /* 19                                        */
    USB_LP_CAN1_RX0_IRQHandler, /* 20                                        */
    CAN1_RX1_IRQHandler,        /* 21                                        */
    CAN1_SCE_IRQHandler,        /* 22                                        */
    EXTI9_5_IRQHandler,         /* 23 EXTI[9:5]                              */
    TIM1_BRK_IRQHandler,        /* 24 TIM1 Break                             */
    TIM1_UP_IRQHandler,         /* 25 TIM1 Update                            */
    TIM1_TRG_COM_IRQHandler,    /* 26 TIM1 Trigger / Commutation             */
    TIM1_CC_IRQHandler,         /* 27 TIM1 Capture/Compare                   */
    TIM2_IRQHandler,            /* 28                                        */
    TIM3_IRQHandler,            /* 29                                        */
    TIM4_IRQHandler,            /* 30                                        */
    I2C1_EV_IRQHandler,         /* 31 I2C1 Event                             */
    I2C1_ER_IRQHandler,         /* 32 I2C1 Error                             */
    I2C2_EV_IRQHandler,         /* 33                                        */
    I2C2_ER_IRQHandler,         /* 34                                        */
    SPI1_IRQHandler,            /* 35                                        */
    SPI2_IRQHandler,            /* 36                                        */
    USART1_IRQHandler,          /* 37                                        */
    USART2_IRQHandler,          /* 38                                        */
    USART3_IRQHandler,          /* 39                                        */
    EXTI15_10_IRQHandler,       /* 40 EXTI[15:10]                            */
    RTC_Alarm_IRQHandler,       /* 41 RTC Alarm via EXTI                     */
    USBWakeUp_IRQHandler,       /* 42 USB wakeup from suspend via EXTI       */
};

/* ── Reset_Handler ──────────────────────────────────────────────────────── */

void Reset_Handler(void)
{
    /* 1. Copy .data section from Flash to SRAM */
    uint32_t *src = &_sidata;
    uint32_t *dst = &_sdata;
    while (dst < &_edata)
        *dst++ = *src++;

    /* 2. Zero-initialise .bss section */
    dst = &_sbss;
    while (dst < &_ebss)
        *dst++ = 0u;

    /* 3. Call SystemInit (sets up clock if you include system_stm32f10x.c) */
    if (SystemInit)
        SystemInit();

    /* 4. Enter application */
    extern int main(void);
    main();

    /* Should never reach here */
    for (;;) {}
}

/* ── Default_Handler — catchall for unimplemented IRQs ─────────────────── */

void Default_Handler(void)
{
    /* Spin forever.  In debug builds, break here to identify the IRQ number
       by inspecting IPSR (bits [8:0] of xPSR).                              */
    for (;;) {}
}


