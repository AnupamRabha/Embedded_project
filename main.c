/*
  PWM + RC-filtered Waveform Generator using TM4C123GH6PM
  Waveforms: SINE, TRIANGLE, SQUARE — switched using SW1 (PF4)
*/

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"

#define SYSCLK_HZ         50000000UL
#define PWM_CARRIER_HZ    200000U
#define TABLE_SIZE        64
#define SAMPLE_RATE_HZ    64000U  // 1 kHz sine (64 samples/cycle)

// PWM Output
#define PWM_BASE          PWM0_BASE
#define PWM_GEN           PWM_GEN_0
#define PWM_OUT_BIT       PWM_OUT_0_BIT
#define PWM_OUT_NUM       PWM_OUT_0
#define PWM_GPIO_PORT     GPIO_PORTB_BASE
#define PWM_GPIO_PIN      GPIO_PIN_6
#define PWM_GPIO_CFG      GPIO_PB6_M0PWM0

// Button (SW1 on PF4)
#define BUTTON_PORT       GPIO_PORTF_BASE
#define BUTTON_PIN        GPIO_PIN_4

static uint16_t sine_table[TABLE_SIZE];
static uint16_t triangle_table[TABLE_SIZE];
static uint16_t square_table[TABLE_SIZE];

volatile uint8_t waveform_mode = 0; // 0=SINE, 1=TRIANGLE, 2=SQUARE

void BuildTables(uint32_t period_counts)
{
    int i;
    for(i = 0; i < TABLE_SIZE; i++)
    {
        // Sine Table
        float theta = (2.0f * M_PI * i) / TABLE_SIZE;
        float s = (sinf(theta) + 1.0f) * 0.5f;
        sine_table[i] = (uint16_t)(s * (period_counts - 2) + 1);

        // Triangle Table
        int mid = TABLE_SIZE / 2;
        float t = (i < mid) ?
            (float)i / mid : (float)(TABLE_SIZE - i) / mid;
        triangle_table[i] = (uint16_t)(t * (period_counts - 2) + 1);

        // Square Table
        if(i < (TABLE_SIZE / 2))
            square_table[i] = (period_counts - 2);
        else
            square_table[i] = 1;
    }
}

// Button Debounce
bool ButtonPressed(void)
{
    static uint8_t lastState = 1;
    uint8_t current = GPIOPinRead(BUTTON_PORT, BUTTON_PIN);
    bool pressed = (current == 0) && (lastState != 0);
    lastState = current;
    return pressed;
}

// Timer ISR
void Timer0A_Handler(void)
{
    static uint32_t idx = 0;

    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    uint16_t duty;
    if(waveform_mode == 0)
        duty = sine_table[idx];
    else if(waveform_mode == 1)
        duty = triangle_table[idx];
    else
        duty = square_table[idx];

    PWMPulseWidthSet(PWM_BASE, PWM_OUT_NUM, duty);

    idx++;
    if(idx >= TABLE_SIZE) idx = 0;
}

int main(void)
{
    SysCtlClockSet(
        SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_SYSDIV_4 | SYSCTL_XTAL_16MHZ);

    uint32_t sysclk = SysCtlClockGet();
    uint32_t pwmPeriodCounts = sysclk / PWM_CARRIER_HZ;

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));

    // PWM Pin Setup
    GPIOPinConfigure(PWM_GPIO_CFG);
    GPIOPinTypePWM(PWM_GPIO_PORT, PWM_GPIO_PIN);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
    PWMGenConfigure(PWM_BASE, PWM_GEN, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(PWM_BASE, PWM_GEN, pwmPeriodCounts);

    BuildTables(pwmPeriodCounts);
    PWMPulseWidthSet(PWM_BASE, PWM_OUT_NUM, sine_table[0]);

    PWMOutputState(PWM_BASE, PWM_OUT_BIT, true);
    PWMGenEnable(PWM_BASE, PWM_GEN);

    // Button PF4 (pull-up)
    GPIOPinTypeGPIOInput(BUTTON_PORT, BUTTON_PIN);
    GPIOPadConfigSet(BUTTON_PORT, BUTTON_PIN,
        GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // Timer Init
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, sysclk / SAMPLE_RATE_HZ - 1);

    IntRegister(INT_TIMER0A, Timer0A_Handler);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntEnable(INT_TIMER0A);
    TimerEnable(TIMER0_BASE, TIMER_A);

    while(1)
    {
        if(ButtonPressed())
        {
            waveform_mode++;
            if(waveform_mode > 2)
                waveform_mode = 0;
            SysCtlDelay(sysclk / 8); // simple debounce delay
        }
    }
}
