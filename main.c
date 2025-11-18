/* main.c
   Tiva C TM4C123 + MCP4725 simple, CPU-driven sine generator (I2C fast mode)
   Uses Timer0A ISR to update the DAC at a fixed sample rate.
*/

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "inc/hw_memmap.h"
#include "inc/hw_i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/i2c.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "inc/hw_ints.h"


#define MCP4725_ADDR       0x60    // A0 = GND -> 0x60 (7-bit address)
#define TABLE_SIZE         8     // 256-sample sine table
#define SAMPLE_RATE_HZ     5000    // target sample rate in Hz (adjustable)
                                  // realistic: try 1k-10k; higher may fail

//static uint16_t sine_table[TABLE_SIZE];      // 12-bit values 0..4095
static uint8_t  i2c_msb[TABLE_SIZE];         // first data byte for MCP4725 fast write
static uint8_t  i2c_lsb[TABLE_SIZE];         // second data byte

// --------------------------------------------------------
// Write 2-bytes to MCP4725 fast write (blocking).
// We assume Device address already set per-transmission.
// --------------------------------------------------------
static inline void MCP4725_WriteBytes(uint8_t msb, uint8_t lsb)
{
    // Set slave address with write (R/W = 0)
    I2CMasterSlaveAddrSet(I2C0_BASE, MCP4725_ADDR, false);

    // Send first data byte (start the burst)
    I2CMasterDataPut(I2C0_BASE, msb);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while (I2CMasterBusy(I2C0_BASE)); // blocking; short on 400kHz

    // Send second data byte and finish
    I2CMasterDataPut(I2C0_BASE, lsb);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while (I2CMasterBusy(I2C0_BASE));
}

// --------------------------------------------------------
// Timer0A ISR: send next sample
// --------------------------------------------------------
void Timer0A_Handler(void)
{
    static uint32_t idx = 0;

    // Clear the timer interrupt
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Write two bytes to the DAC for current sample
    MCP4725_WriteBytes(i2c_msb[idx], i2c_lsb[idx]);

    // advance
    idx++;
    if (idx >= TABLE_SIZE) idx = 0;
}

// --------------------------------------------------------
// I2C0 initialization (PB2=SCL, PB3=SDA) at 400 kHz
// --------------------------------------------------------
void I2C0_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure pin muxing for I2C0 functions on PB2/PB3
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Configure pin types
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Initialize the I2C master at 400 kHz (fast mode)
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true); // true -> 400kHz
}

// --------------------------------------------------------
// Build sine table (12-bit, 0..4095) and precompute bytes
// For MCP4725 fast mode we pack the 12-bit data into two bytes.
// Many implementations send:
//   msb = (value >> 8) & 0x0F;           // upper 4 bits (D11..D8) in lower nibble
//   lsb = value & 0xFF;                  // lower 8 bits
// That works with MCP4725 fast write (address then 2 data bytes).
// --------------------------------------------------------
void BuildSineTable(void)
{   int i;
    for (i = 0; i < TABLE_SIZE; i++)
    {
        float theta = (2.0f * M_PI * i) / (float)TABLE_SIZE;
        float s = (sinf(theta) + 1.0f) * 2047.0f; // 0..4094
        uint16_t val = (uint16_t)(s + 0.5f);
        if (val > 4095) val = 4095;

        // Precompute the two DAC bytes directly
        i2c_msb[i] = (val >> 8) & 0x0F;
        i2c_lsb[i] = val & 0xFF;
    }
}

// --------------------------------------------------------
// Timer0A init to fire at SAMPLE_RATE_HZ (periodic interrupt)
// --------------------------------------------------------
void Timer0A_Init(uint32_t sample_rate_hz)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerDisable(TIMER0_BASE, TIMER_A);

    // Configure as 32-bit periodic timer
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    // compute load for desired rate
    uint32_t load = SysCtlClockGet() / sample_rate_hz;
    if (load == 0) load = 1;
    TimerLoadSet(TIMER0_BASE, TIMER_A, load - 1);

    // Enable interrupt
    IntRegister(INT_TIMER0A, Timer0A_Handler); // alternative: use weak vector if you prefer
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntEnable(INT_TIMER0A);

    // Start timer
    TimerEnable(TIMER0_BASE, TIMER_A);
}

// --------------------------------------------------------
// Main
// --------------------------------------------------------
int main(void)
{
    // Set system clock: 50 MHz using PLL with 16 MHz crystal (common TM4C123 config)
    SysCtlClockSet(
        SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_SYSDIV_4 | SYSCTL_XTAL_16MHZ);

    // Initialize I2C and sine table
    I2C0_Init();
    BuildSineTable();

    // Small delay to let I2C lines settle / device power-up
    SysCtlDelay(SysCtlClockGet() / 10);

    // Initialize timer ISR for the sample rate
    Timer0A_Init(SAMPLE_RATE_HZ);

    // Main does nothing: waveform produced in ISR
    while (1)
    {
        // optional: put MCU to sleep until next interrupt to save power
        // __asm("WFI");  // if you want to use Wait for Interrupt
    }
}
