#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "inc/hw_memmap.h"
#include "inc/hw_i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/i2c.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"


#define MCP4725_ADDR 0x60        // A0 pin = GND
#define TABLE_SIZE   64          // 64-point sine table

uint16_t sine_table[TABLE_SIZE];

// --------------------------------------------------------
// Write value to MCP4725 (fast mode, 12-bit DAC)
// --------------------------------------------------------
void MCP4725_Write(uint16_t value)
{
    I2CMasterSlaveAddrSet(I2C0_BASE, MCP4725_ADDR, false);

    I2CMasterDataPut(I2C0_BASE, (value >> 8) & 0x0F);   // Upper 4 bits
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C0_BASE));

    I2CMasterDataPut(I2C0_BASE, value & 0xFF);          // Lower 8 bits
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(I2CMasterBusy(I2C0_BASE));
}

// --------------------------------------------------------
// Initialize I2C0 on PB2(SCL) and PB3(SDA)
// --------------------------------------------------------
void I2C0_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);  // 400kHz
}

// --------------------------------------------------------
// Generate sine lookup table
// --------------------------------------------------------
void GenerateSineTable(void)
{   int i;
    for (i = 0; i < TABLE_SIZE; i++)
    {
        float theta = (2.0f * M_PI * i) / TABLE_SIZE;
        float s = (sinf(theta) + 1.0f) * 2047.0f;     // 0–4095
        sine_table[i] = (uint16_t)s;
    }
}

// --------------------------------------------------------
// MAIN
// --------------------------------------------------------
int main(void)
{
    SysCtlClockSet(
          SYSCTL_USE_PLL |
          SYSCTL_CFG_VCO_480 |
          SYSCTL_OSC_MAIN |
          SYSCTL_XTAL_16MHZ);

    uint32_t sysclk = SysCtlClockGet();

    I2C0_Init();
    GenerateSineTable();

    int index = 0;

    while (1)
    {
        MCP4725_Write(sine_table[index]);

        index++;
        if (index >= TABLE_SIZE)
            index = 0;

        SysCtlDelay(sysclk / (500 * TABLE_SIZE));
        // Produces approximately 500 Hz sine wave
    }
}
