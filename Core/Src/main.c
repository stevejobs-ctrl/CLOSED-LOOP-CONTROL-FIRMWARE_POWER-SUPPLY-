
#include "stm32f103xb.h"
#include <stdint.h>
#include <string.h>

/* ── Compile-time constants */

#define PWM_ARR              719u      /* TIM1 ARR → 100 kHz PWM             */
#define DUTY_MIN             36u       /* 5%  of 720 counts                  */
#define DUTY_MAX             648u      /* 90% of 720 counts                  */

#define ADC_VREF_MV          3300u
#define ADC_RESOLUTION       4095u

/* ── INA219 */

/* 7-bit I2C address — A1=GND, A0=GND (both address pins to ground)          */
#define INA219_ADDR          0x40u

/* INA219 internal register addresses                                         */
#define INA219_REG_CONFIG    0x00u
#define INA219_REG_SHUNT_V   0x01u
#define INA219_REG_BUS_V     0x02u
#define INA219_REG_CURRENT   0x04u     /* valid only after calibration write  */
#define INA219_REG_CALIB     0x05u



#define INA219_CONFIG_VALUE  0x019Fu

/*
 *  Calibration register
 *  R_shunt = 0.01 Ohm, Current_LSB = 1 mA:
 *  Cal = trunc(0.04096 / (0.001 x 0.01)) = 4096 = 0x1000
 *  Change these two if you use a different shunt value.
 */
#define INA219_CAL_VALUE       0x1000u
#define INA219_CURRENT_LSB_MA  1u     /* 1 LSB of current register = 1 mA   */

/* OCP in milliamps — trip if measured current exceeds this                  */
#define OCP_THRESHOLD_MA     2500

/* ── LCD  */
#define LCD_I2C_ADDR         0x27u

/* ── PID */
#define GAIN_SCHED_THR_MV    5000u
#define KP_STRONG            10000
#define KP_STANDARD           8192
#define KI                     120
#define KD                      40

#define INTEGRAL_MAX         2000000
#define INTEGRAL_MIN        -2000000

/* ── Timing */
#define TICK_PERIOD_MS       10u
#define LCD_PERIOD_MS        100u

/* ── ADC DMA buffer — 2 channels─ */
/* [0] = V_feedback (PA0 / ADC1_IN0)
   [1] = Pot        (PA1 / ADC1_IN1)                                         */
volatile uint16_t adc_data[2];

/* ── Shared I2C bus arbitration */
/*
 *  The ISR and the main loop both use I2C1.
 *  The ISR has higher priority and will always preempt the main loop, so a
 *  simple flag lets the main loop know not to start an LCD transaction while
 *  the ISR might be mid-transaction.
 *
 *  In practice the INA219 read in the ISR is so short (~10 us) that
 *  collisions are extremely unlikely, but we guard anyway.
 */
static volatile uint8_t i2c_bus_busy = 0;

/* ── Control-loop state */
static volatile uint32_t current_setpoint_mv = 0;
static volatile uint32_t final_target_mv     = 0;
static volatile int32_t  integral            = 0;
static volatile int32_t  prev_error          = 0;
static volatile uint32_t duty               = DUTY_MIN;
static volatile uint8_t  fault_flag         = 0;

/* Latest INA219 current reading in mA, updated every ISR tick              */
static volatile int16_t  ina219_current_ma   = 0;

/* ── Timing counters  */
static volatile uint32_t sys_tick_ms = 0;
static volatile uint32_t last_tick   = 0;
static volatile uint32_t last_lcd    = 0;

/*
 *  Forward declarations
 * */
static void clock_init(void);
static void gpio_init(void);
static void tim1_pwm_init(void);
static void adc_dma_init(void);
static void i2c1_init(void);
static void systick_init(void);

static void     ina219_init(void);
static int16_t  ina219_read_current_ma(void);

static void     soft_start_update(void);
static uint32_t adc_to_mv(uint16_t raw);
static void     i2c_lcd_update(uint32_t v_mv, int16_t i_ma);

/* I2C primitives (shared) */
static void    i2c_start(void);
static void    i2c_stop(void);
static void    i2c_send_addr(uint8_t addr7, uint8_t rw);
static void    i2c_write_byte(uint8_t byte);
static uint8_t i2c_read_byte(uint8_t send_ack);

/* LCD helpers */
static void lcd_send_nibble(uint8_t nibble, uint8_t rs, uint8_t bl);
static void lcd_send_byte(uint8_t byte, uint8_t rs);
static void lcd_write_string(const char *s);
static void uint_to_str(uint32_t val, char *buf, uint8_t width);

/* Fault helper */
static inline void trigger_fault(void);

/*
 *  main
  */

int main(void)
{
    clock_init();
    gpio_init();
    systick_init();
    tim1_pwm_init();
    adc_dma_init();
    i2c1_init();
    ina219_init();

    __enable_irq();

    for (;;)
    {
        /* 10 ms tick — soft-start ramp */
        if ((sys_tick_ms - last_tick) >= TICK_PERIOD_MS)
        {
            last_tick = sys_tick_ms;
            if (!fault_flag)
                soft_start_update();
        }

        /* 100 ms tick — update LCD with live voltage and current */
        if ((sys_tick_ms - last_lcd) >= LCD_PERIOD_MS)
        {
            last_lcd = sys_tick_ms;

            /* Only write to I2C if the ISR is not currently using the bus   */
            if (!i2c_bus_busy)
            {
                uint32_t v_mv = adc_to_mv(adc_data[0]);
                i2c_lcd_update(v_mv, ina219_current_ma);
            }
        }
    }
}

/*
 *  DMA1 Channel 1 IRQ — fires after every 2-channel ADC scan
 * */

void DMA1_Channel1_IRQHandler(void)
{
    /* Clear Transfer-Complete flag (CTCIF1 in IFCR)  */
    DMA1->IFCR = DMA_IFCR_CTCIF1;

    if (fault_flag)
        return;

    /* ── Read current from INA219*/
    /*
     *  Mark bus busy so the main loop will not start an LCD transaction.
     *  The INA219 read is:
     *    START | addr+W | reg(0x04) | STOP | START | addr+R | hiB | loB | STOP
     *  At 400 kHz this takes roughly 10 us — acceptable inside this ISR.
     */
    i2c_bus_busy = 1;
    ina219_current_ma = ina219_read_current_ma();
    i2c_bus_busy = 0;

    /* ── OCP check — now in milliamps from INA219 */
    /*
     *  We check the signed value against the positive threshold.
     *  A negative reading means reverse current (wiring error or back-EMF)
     *  and is also caught here if it somehow exceeds magnitude threshold.
     */
    if (ina219_current_ma > (int16_t)OCP_THRESHOLD_MA)
    {
        trigger_fault();
        return;
    }

    /* ── Calculate error ────────────────────────────────────────────────── */
    int32_t v_measured_mv = (int32_t)adc_to_mv(adc_data[0]);
    int32_t setpoint_mv   = (int32_t)current_setpoint_mv;
    int32_t error         = setpoint_mv - v_measured_mv;

    /* ── Gain scheduling ────────────────────────────────────────────────── */
    int32_t kp = (current_setpoint_mv < GAIN_SCHED_THR_MV)
                 ? KP_STRONG
                 : KP_STANDARD;

    /* ── PID terms ──────────────────────────────────────────────────────── */
    int32_t p_term = kp * error;

    integral += error;
    if (integral >  INTEGRAL_MAX) integral =  INTEGRAL_MAX;
    if (integral <  INTEGRAL_MIN) integral =  INTEGRAL_MIN;
    int32_t i_term = KI * integral;

    int32_t d_term = KD * (error - prev_error);
    prev_error = error;

    /* Sum and scale back from Q12 fixed-point domain                        */
    int32_t pid_out = (p_term + i_term + d_term) >> 12;

    /* ── Update duty with anti-windup clamp ─────────────────────────────── */
    int32_t new_duty = (int32_t)duty + pid_out;
    if (new_duty > (int32_t)DUTY_MAX) new_duty = (int32_t)DUTY_MAX;
    if (new_duty < (int32_t)DUTY_MIN) new_duty = (int32_t)DUTY_MIN;
    duty = (uint32_t)new_duty;

    /* Write to TIM1 CCR1 shadow register — takes effect at next update event*/
    TIM1->CCR1 = duty;
}

/* ══════════════════════════════════════════════════════════════════════════
 *  SysTick — 1 ms
 * ══════════════════════════════════════════════════════════════════════════ */

void SysTick_Handler(void)
{
    sys_tick_ms++;
}

/* ══════════════════════════════════════════════════════════════════════════
 *  INA219 — initialisation
 *
 *  Must be called once after i2c1_init() and before the DMA ISR starts.
 *  Writes two registers:
 *    1. Configuration register — sets PGA range, ADC resolution, and mode
 *    2. Calibration register   — tells the INA219 how to scale raw shunt
 *                                 voltage into a current value in LSB units
 * ══════════════════════════════════════════════════════════════════════════ */

static void ina219_init(void)
{
    /* ── Write Configuration Register (0x00) ────────────────────────────── */
    i2c_start();
    i2c_send_addr(INA219_ADDR, 0);                          /* write mode    */
    i2c_write_byte(INA219_REG_CONFIG);
    i2c_write_byte((uint8_t)(INA219_CONFIG_VALUE >> 8));    /* MSB first     */
    i2c_write_byte((uint8_t)(INA219_CONFIG_VALUE & 0xFFu)); /* LSB           */
    i2c_stop();

    /*
     *  Wait for the first conversion to complete.
     *  12-bit mode takes ~532 us per channel.  We wait 2 ms to be safe.
     *  We poll sys_tick_ms instead of a busy delay so SysTick keeps running.
     */
    uint32_t t = sys_tick_ms;
    while ((sys_tick_ms - t) < 2u) {}

    /* ── Write Calibration Register (0x05) ─────────────────────────────── */
    /*
     *  This is what enables the Current Register (0x04).
     *  Without writing calibration, the current register always reads 0.
     *
     *  Formula: Cal = trunc( 0.04096 / (Current_LSB_A x R_shunt_Ohm) )
     *  With R_shunt=0.01 Ohm and LSB=0.001 A:  Cal = 4096 = 0x1000
     */
    i2c_start();
    i2c_send_addr(INA219_ADDR, 0);
    i2c_write_byte(INA219_REG_CALIB);
    i2c_write_byte((uint8_t)(INA219_CAL_VALUE >> 8));
    i2c_write_byte((uint8_t)(INA219_CAL_VALUE & 0xFFu));
    i2c_stop();
}

/* ══════════════════════════════════════════════════════════════════════════
 *  INA219 — read Current Register (0x04)
 *
 *  Returns signed current in mA.
 *  With INA219_CURRENT_LSB_MA = 1, each LSB of the Current Register = 1 mA.
 *
 *  Transaction sequence:
 *    1. Write register pointer 0x04 to the INA219
 *    2. Repeated START, then read 2 bytes MSB-first
 *    3. Combine bytes into a signed 16-bit value
 *
 *  Positive = current flowing from high side to load (normal operation).
 *  Negative = reverse current — flagged but not tripped by OCP (adjust if
 *  your application requires reverse-current protection too).
 * ══════════════════════════════════════════════════════════════════════════ */

static int16_t ina219_read_current_ma(void)
{
    uint8_t hi, lo;

    /* Step 1: Point the INA219 internal register pointer at Current Register*/
    i2c_start();
    i2c_send_addr(INA219_ADDR, 0);      /* write                             */
    i2c_write_byte(INA219_REG_CURRENT);
    i2c_stop();

    /* Step 2: Read 2 bytes from the register just selected                  */
    i2c_start();
    i2c_send_addr(INA219_ADDR, 1);      /* read                              */
    hi = i2c_read_byte(1);              /* high byte — ACK to request more   */
    lo = i2c_read_byte(0);              /* low byte  — NACK signals last byte */
    i2c_stop();

    /* Step 3: Reconstruct signed 16-bit two's-complement current value      */
    int16_t raw = (int16_t)(((uint16_t)hi << 8) | (uint16_t)lo);

    /* Scale by LSB factor (1 mA/LSB with default calibration — multiply is
       a no-op here but kept explicit for portability to other LSB choices)   */
    return (int16_t)(raw * (int16_t)INA219_CURRENT_LSB_MA);
}

/*
 *  Fault — kill PWM output and light fault LED
  */

static inline void trigger_fault(void)
{
    /* Set CCR1 to 0 so compare unit forces output low                       */
    TIM1->CCR1 = 0;

    /* Disable CH1 and CH1N outputs via CCER                                 */
    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE);

    /* Light the fault LED (PB12 active high)                                */
    GPIOB->BSRR = (1u << 12);

    fault_flag = 1;
}

/*
 *  Soft-start — called every 10 ms from the main loop
 *
 *  Reads the potentiometer (adc_data[1] — now IN1 instead of IN2)
 *  and increments the active setpoint by one count per tick.
 * */

static void soft_start_update(void)
{
    /* adc_data[1] is the Pot channel (PA1 / ADC1_IN1) in Rev 2             */
    uint32_t pot_mv = adc_to_mv(adc_data[1]);
    final_target_mv = pot_mv;

    if (current_setpoint_mv < final_target_mv)
    {
        current_setpoint_mv++;
        if (current_setpoint_mv > final_target_mv)
            current_setpoint_mv = final_target_mv;
    }
    else if (current_setpoint_mv > final_target_mv)
    {
        /* Allow immediate step-down if user turns the pot down              */
        current_setpoint_mv = final_target_mv;
    }
}

/*
 *  ADC count to millivolts
 *  */

static uint32_t adc_to_mv(uint16_t raw)
{
    return ((uint32_t)raw * ADC_VREF_MV) / ADC_RESOLUTION;
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Clock init — 72 MHz via HSE + PLL
 * ══════════════════════════════════════════════════════════════════════════ */

static void clock_init(void)
{
    /* Enable external high-speed oscillator                                 */
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY)) {}

    /* Flash: 2 wait states for SYSCLK 48–72 MHz, enable prefetch buffer     */
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;

    /* Bus prescalers:
       AHB  = SYSCLK / 1 = 72 MHz
       APB1 = AHB    / 2 = 36 MHz  (I2C1 source)
       APB2 = AHB    / 1 = 72 MHz  (TIM1, ADC1, GPIO source)
       ADC  = APB2   / 6 = 12 MHz  (must be <= 14 MHz)                      */
    RCC->CFGR = RCC_CFGR_HPRE_DIV1
              | RCC_CFGR_PPRE1_DIV2
              | RCC_CFGR_PPRE2_DIV1
              | RCC_CFGR_ADCPRE_DIV6;

    /* PLL: source = HSE, multiplier = x9 → 8 MHz x 9 = 72 MHz              */
    RCC->CFGR |= RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL9;

    /* Enable PLL and wait for lock                                          */
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {}

    /* Switch SYSCLK source to PLL and confirm                               */
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {}

    SystemCoreClockUpdate();
}

/*
 *  GPIO init
 *
 *  PA0 — ADC1_IN0  Analog input (V_feedback)     CNF=00 MODE=00 → 0x0
 *  PA1 — ADC1_IN1  Analog input (Pot)             CNF=00 MODE=00 → 0x0
 *  PA8 — TIM1_CH1  AF push-pull 50 MHz            CNF=10 MODE=11 → 0xB
 *  PB6 — I2C1_SCL  AF open-drain 50 MHz           CNF=11 MODE=11 → 0xF
 *  PB7 — I2C1_SDA  AF open-drain 50 MHz           CNF=11 MODE=11 → 0xF
 *  PB12— Fault LED GP push-pull 2 MHz             CNF=00 MODE=10 → 0x2
 * */

static void gpio_init(void)
{
    /* Enable GPIOA and GPIOB clocks on APB2                                 */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;

    /* PA0 and PA1 — analog input (clear their nibbles in CRL)               */
    /* CRL bits [7:0] cover PA0 (bits 3:0) and PA1 (bits 7:4)               */
    GPIOA->CRL &= ~(0xFFu);      /* both nibbles → 0x00 = analog input       */

    /* PA8 — AF push-pull 50 MHz
       CRH bit offsets: (pin - 8) * 4.  PA8 → (8-8)*4 = offset 0            */
    GPIOA->CRH &= ~(0xFu << 0);
    GPIOA->CRH |=  (0xBu << 0);  /* CNF=10 MODE=11                          */

    /* PB6 and PB7 — AF open-drain 50 MHz
       CRL bit offsets: pin * 4.  PB6 → offset 24, PB7 → offset 28          */
    GPIOB->CRL &= ~(0xFFu << 24);
    GPIOB->CRL |=  (0xFFu << 24); /* both 0xF: CNF=11 MODE=11               */

    /* PB12 — GP push-pull 2 MHz
       CRH bit offset: (12-8)*4 = 16                                         */
    GPIOB->CRH &= ~(0xFu << 16);
    GPIOB->CRH |=  (0x2u << 16); /* CNF=00 MODE=10                          */
    GPIOB->BRR  =  (1u   << 12); /* ensure LED starts off                   */
}

/*
 *  TIM1 — 100 kHz PWM on PA8
 * */

static void tim1_pwm_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    TIM1->PSC  = 0;           /* no prescale — timer clock = 72 MHz          */
    TIM1->ARR  = PWM_ARR;     /* 719 → 720 ticks = 10 us period = 100 kHz    */
    TIM1->CCR1 = DUTY_MIN;    /* start at minimum duty (5%)                  */

    /* CCMR1: OC1M = 110 (PWM mode 1), OC1PE = 1 (preload enable)           */
    TIM1->CCMR1 = (6u << 4) | (1u << 3);

    /* CCER: CC1E = 1 (CH1 output enable), active high polarity              */
    TIM1->CCER  = TIM_CCER_CC1E;

    /* BDTR: MOE = 1 — mandatory for TIM1 advanced timer outputs             */
    TIM1->BDTR  = TIM_BDTR_MOE;

    /* CR1: ARPE = 1 (ARR buffered), CEN = 1 (counter run)                  */
    TIM1->CR1   = TIM_CR1_ARPE | TIM_CR1_CEN;

    /* Force update event to immediately load PSC and ARR shadow registers   */
    TIM1->EGR   = TIM_EGR_UG;
}

/*
 *  ADC1 + DMA1 Ch1 — 2-channel scan (V_feedback + Pot)
 *
 *  Rev 2: CNDTR = 2, SQR1.L = 1, SQR3 uses IN0 and IN1 only.
 *  Current sensing removed; the INA219 handles it over I2C.
 **/

static void adc_dma_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->AHBENR  |= RCC_AHBENR_DMA1EN;

    /* ── DMA1 Channel 1 */
    DMA1_Channel1->CCR   = 0;
    DMA1_Channel1->CPAR  = (uint32_t)&ADC1->DR;   /* source: ADC data reg   */
    DMA1_Channel1->CMAR  = (uint32_t)adc_data;    /* dest: our 2-word buffer */
    DMA1_Channel1->CNDTR = 2u;                    /* 2 transfers per scan    */

    DMA1_Channel1->CCR =
          DMA_CCR_CIRC    /* circular — auto-reloads CNDTR after each scan   */
        | DMA_CCR_MINC    /* memory address auto-increments                  */
        | DMA_CCR_PSIZE_0 /* peripheral size 16-bit (ADC_DR is 16-bit)       */
        | DMA_CCR_MSIZE_0 /* memory size 16-bit (uint16_t array)             */
        | DMA_CCR_PL_1    /* priority: very high (PL = 11)                   */
        | DMA_CCR_PL_0
        | DMA_CCR_TCIE    /* transfer-complete interrupt → ISR               */
        | DMA_CCR_EN;     /* enable channel                                  */

    /* DMA1 Ch1 IRQ: priority 1 (high, but below priority 0 if needed)      */
    NVIC_SetPriority(DMA1_Channel1_IRQn, 1);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    /* ── ADC1  */

    /* Power on ADC — first write to ADON just turns the analog block on.
       A second write later starts the actual conversion.                     */
    ADC1->CR2 = ADC_CR2_ADON;

    /* Wait at least 1 us (tSTAB) for analog circuitry to stabilise          */
    for (volatile uint32_t i = 0; i < 100; i++) __NOP();

    /* Calibration — always run after power-on to minimise offset error      */
    ADC1->CR2 |= ADC_CR2_RSTCAL;
    while (ADC1->CR2 & ADC_CR2_RSTCAL) {}
    ADC1->CR2 |= ADC_CR2_CAL;
    while (ADC1->CR2 & ADC_CR2_CAL) {}

    /* Scan mode — ADC converts all channels in the sequence automatically   */
    ADC1->CR1 = ADC_CR1_SCAN;

    /* Sequence length: L[3:0] = 1 → 2 conversions total                    */
    ADC1->SQR1 = (1u << 20);

    /* Conversion sequence:
       SQ1 [4:0]  = 0 → ADC1_IN0 (PA0) = V_feedback
       SQ2 [9:5]  = 1 → ADC1_IN1 (PA1) = Pot                                */
    ADC1->SQR3 = (0u << 0)      /* SQ1 = IN0 */
               | (1u << 5);     /* SQ2 = IN1 */

    /* Sample time: 239.5 ADC clock cycles per channel
       At ADC CLK = 12 MHz: 239.5 / 12 MHz = ~20 us per channel.
       SMPR2 covers IN0 (bits 2:0) and IN1 (bits 5:3); 7 = 239.5 cycles.    */
    ADC1->SMPR2 = (7u << 0)     /* IN0 */
                | (7u << 3);    /* IN1 */

    /* Final CR2: continuous mode + DMA handshake + SWSTART trigger          */
    ADC1->CR2 = ADC_CR2_ADON
              | ADC_CR2_CONT       /* continuous — restarts scan automatically */
              | ADC_CR2_DMA        /* DMA request after each conversion        */
              | ADC_CR2_EXTTRIG    /* external trigger enable (needed for SW)  */
              | (7u << 17);        /* EXTSEL[2:0] = 111 → SWSTART             */

    /* Start the first conversion — CONT keeps it running indefinitely       */
    ADC1->CR2 |= ADC_CR2_SWSTART;
}

/*
 *  I2C1 — 400 kHz fast mode, APB1 = 36 MHz
 *
 *  CCR  = Fpclk / (3 x Fi2c) = 36 000 000 / (3 x 400 000) = 30
 *  TRISE = floor(300 ns x 36 MHz) + 1 = 11
 * */

static void i2c1_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    /* Software reset clears any sticky BUSY flag left from a previous run   */
    I2C1->CR1 = I2C_CR1_SWRST;
    I2C1->CR1 = 0;

    I2C1->CR2   = 36u;               /* FREQ = APB1 in MHz = 36             */
    I2C1->CCR   = I2C_CCR_FS | 30u; /* fast mode, CCR = 30                 */
    I2C1->TRISE = 11u;
    I2C1->CR1   = I2C_CR1_PE;        /* enable I2C peripheral               */
}

/*
 *  SysTick — 1 ms tick
 */

static void systick_init(void)
{
    /* SysTick_Config sets LOAD, resets VAL, and enables HCLK source + IRQ  */
    SysTick_Config(SystemCoreClock / 1000u);

    /* Lowest priority — control ISR (priority 1) must preempt this          */
    NVIC_SetPriority(SysTick_IRQn, 15);
}

/*
 *  I2C primitives — polled, shared by INA219 and LCD
 *
 *  Production note: add timeout counters on every while() loop to
 *  recover gracefully from bus lockup (e.g. slave holding SDA low).
 */

static void i2c_start(void)
{
    /*
     *  Writing START while PE=1 generates a START condition.
     *  If the bus was idle this is a normal START.
     *  If called right after a write without a STOP it becomes a
     *  Repeated START — both are legal and used in the INA219 read sequence.
     */
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB)) {}   /* SB set when START is sent     */
}

static void i2c_stop(void)
{
    I2C1->CR1 |= I2C_CR1_STOP;
    /* Wait until the BUSY flag clears (bus released to idle)                */
    while (I2C1->SR2 & I2C_SR2_BUSY) {}
}

static void i2c_send_addr(uint8_t addr7, uint8_t rw)
{
    /*
     *  Address byte on the wire = 7-bit address shifted left by 1, OR'd
     *  with the R/nW bit: 0 = write, 1 = read.
     *  Writing to DR while SB is set clears SB and starts the address phase.
     */
    I2C1->DR = (uint8_t)((addr7 << 1) | rw);
    /* ADDR flag set when the address byte is sent and ACK received          */
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) {}
    /* Clear ADDR by reading SR1 then SR2 — this is required by hardware     */
    (void)I2C1->SR1;
    (void)I2C1->SR2;
}

static void i2c_write_byte(uint8_t byte)
{
    I2C1->DR = byte;
    /* TXE goes high when DR is emptied into the shift register              */
    while (!(I2C1->SR1 & I2C_SR1_TXE)) {}
}

static uint8_t i2c_read_byte(uint8_t send_ack)
{
    /*
     *  ACK control must be programmed BEFORE the byte shifts in, because the
     *  hardware generates the ACK/NACK immediately after the 8th data clock.
     *
     *  For all bytes except the last: set ACK.
     *  For the last byte: clear ACK so the slave stops sending after this one.
     */
    if (send_ack)
        I2C1->CR1 |=  I2C_CR1_ACK;
    else
        I2C1->CR1 &= ~I2C_CR1_ACK;

    while (!(I2C1->SR1 & I2C_SR1_RXNE)) {}  /* wait for receive data ready  */
    return (uint8_t)I2C1->DR;
}

/*
 *  LCD — PCF8574 I/O expander → HD44780 in 4-bit mode
 *
 *  PCF8574 wiring assumed:
 *    P7 P6 P5 P4 | P3  P2 P1 P0
 *    D7 D6 D5 D4 | BL  EN RW RS
 *
 *  Line 1 (DDRAM 0x00): "V= xx.xxx V     "  — live output voltage
 *  Line 2 (DDRAM 0x40): "I=  xxxx mA    "  — live INA219 current
 *  */

static void lcd_send_nibble(uint8_t nibble, uint8_t rs, uint8_t bl)
{
    uint8_t data = (uint8_t)((nibble & 0x0Fu) << 4)
                 | (bl ? 0x08u : 0x00u)   /* backlight (P3)                 */
                 | (rs ? 0x01u : 0x00u);  /* RS (P0): 0=command, 1=data     */

    /* Latch on falling edge of EN (P2).  Two I2C transactions: EN=1, EN=0. */
    i2c_start();
    i2c_send_addr(LCD_I2C_ADDR, 0);
    i2c_write_byte((uint8_t)(data | 0x04u));  /* EN high                    */
    i2c_write_byte(data);                      /* EN low — LCD latches data  */
    i2c_stop();
}

static void lcd_send_byte(uint8_t byte, uint8_t rs)
{
    lcd_send_nibble(byte >> 4,   rs, 1);  /* high nibble first              */
    lcd_send_nibble(byte & 0x0F, rs, 1);  /* low nibble                     */
}

static void lcd_write_string(const char *s)
{
    while (*s)
        lcd_send_byte((uint8_t)*s++, 1);  /* RS=1 sends to data register    */
}

static void uint_to_str(uint32_t val, char *buf, uint8_t width)
{
    buf[width] = '\0';
    for (int8_t i = (int8_t)(width - 1); i >= 0; i--)
    {
        buf[i] = (char)('0' + (val % 10u));
        val /= 10u;
    }
}

static void i2c_lcd_update(uint32_t v_mv, int16_t i_ma)
{
    char line[17];

    /* ── Line 1: output voltage */
    /* Format: "V= xx.xxx V     "  (2 integer + 3 decimal digits)            */
    uint32_t v_int  = v_mv / 1000u;
    uint32_t v_frac = v_mv % 1000u;

    line[0] = 'V'; line[1] = '='; line[2] = ' ';
    uint_to_str(v_int,  &line[3], 2);
    line[5] = '.';
    uint_to_str(v_frac, &line[6], 3);
    line[9] = ' '; line[10] = 'V';
    for (uint8_t i = 11; i < 16; i++) line[i] = ' ';
    line[16] = '\0';

    lcd_send_byte(0x80u, 0);         /* DDRAM address command → line 1       */
    lcd_write_string(line);

    /* ── Line 2: INA219 current */
    /* Format: "I=  xxxx mA    "  (sign + 4 digits)                          */
    uint8_t  neg   = (i_ma < 0);
    uint32_t i_abs = neg ? (uint32_t)(-(int32_t)i_ma) : (uint32_t)i_ma;

    line[0] = 'I'; line[1] = '='; line[2] = ' ';
    line[3] = neg ? '-' : ' ';
    uint_to_str(i_abs, &line[4], 4);   /* up to 9999 mA display range       */
    line[8] = ' '; line[9] = 'm'; line[10] = 'A';
    for (uint8_t i = 11; i < 16; i++) line[i] = ' ';
    line[16] = '\0';

    lcd_send_byte(0xC0u, 0);         /* DDRAM address command → line 2       */
    lcd_write_string(line);
}
