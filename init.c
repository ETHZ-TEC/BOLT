/*
 * Copyright (c) 2015, Swiss Federal Institute of Technology (ETH Zurich).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors:  Reto Da Forno, Felix Sutton
 */

/*
 * init.c
 * initialization (configuration) of the MCU
 */


#include "main.h"


// prescalers and 1st and 2nd stage modulation coefficients; Note: prescaler = N / 16 where N = SMCLK_SPEED / BAUD_RATE
const uint16_t UARTConfig[NUM_OF_UART_RATES][NUM_OF_MCLK_SPEEDS][3] =
{
  //   1 MHz       8 MHz      16 MHz
  { { 6, 8, 0x11 }, { 52, 1, 0x49 }, { 104,  2, 0xD6 } },    // 9600 Bd
  { { 0, 8, 0xD6 }, {  4, 5, 0x55 }, {   8, 10, 0xFB } },    // 115200 Bd
};


// configures two SPI modules (SPI_A1 and SPI_B0) in 3-wire slave mode
void initSPI()
{
  // Note: In slave mode, no internal clock source is required (clock is provided by the external master), it operates as an unclocked peripheral
  ASSERT((SPI_CKPL < NUM_OF_SPI_CKPL) && (SPI_CKPH < NUM_OF_SPI_CKPH));

  // SPI A, 3-wire slave mode
  // set pins 4 to 6 of port 2 as secondary module function input (UCA0CLK, UCA1SIMO, UCA1SOMI)
  GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN4 + GPIO_PIN5 + GPIO_PIN6);
  GPIO_setAsPeripheralModuleFunctionInputPinInline(GPIO_PORT_P2, GPIO_PIN4 + GPIO_PIN5 + GPIO_PIN6, GPIO_SECONDARY_MODULE_FUNCTION);
  EUSCI_SPI_slaveInitInline(EUSCI_A1_BASE,
                            EUSCI_A_SPI_MSB_FIRST,
                            (SPI_CKPH == SPI_CKPH_0) ? EUSCI_A_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT : EUSCI_A_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT,
                            (SPI_CKPL == SPI_CKPL_0) ? EUSCI_A_SPI_CLOCKPOLARITY_INACTIVITY_LOW : EUSCI_A_SPI_CLOCKPOLARITY_INACTIVITY_HIGH,
                            EUSCI_A_SPI_3PIN);
  //EUSCI_A_SPI_enable(EUSCI_A1_BASE);

  // SPI B, 3-wire slave mode
  // set pin 2 of port 2 to input, secondary module function (UCB0CLK)
  GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN2);  // enable the pulldown resistor to prevent floating inputs
  GPIO_setAsPeripheralModuleFunctionInputPinInline(GPIO_PORT_P2, GPIO_PIN2, GPIO_SECONDARY_MODULE_FUNCTION);
  // set pins 6 and 7 of port 1 to input, secondary module function (UCB0SIMO, UCB0SOMI)
  GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN6 + GPIO_PIN7);
  GPIO_setAsPeripheralModuleFunctionInputPinInline(GPIO_PORT_P1, GPIO_PIN6 + GPIO_PIN7, GPIO_SECONDARY_MODULE_FUNCTION);
  EUSCI_SPI_slaveInitInline(EUSCI_B0_BASE,
                            EUSCI_B_SPI_MSB_FIRST,
                            (SPI_CKPH == SPI_CKPH_0) ? EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT : EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT,
                            (SPI_CKPL == SPI_CKPL_0) ? EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW : EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH,
                            EUSCI_B_SPI_3PIN);
}


// initializes the UART interface (EUSCI_A0)
void initUART()
{
  ASSERT((UART_BAUDRATE == UART_RATE_9600 || UART_BAUDRATE == UART_RATE_115200) &&
         (MCLK_SPEED == MCLK_SPEED_16MHZ || MCLK_SPEED == MCLK_SPEED_8MHZ || MCLK_SPEED == MCLK_SPEED_1MHZ));

  // configure GPIO pin for UART operation
  GPIO_setAsPeripheralModuleFunctionInputPinInline(GPIO_PORT_P2, GPIO_PIN0, GPIO_SECONDARY_MODULE_FUNCTION);  // GPIO_PIN1 = RXD pin is not needed

  // configure UART with the DriverLib (see table 20-5 in the user's guide for more information)
  EUSCI_UART_initAdvanceInline(EUSCI_A_UART_CLOCKSOURCE_SMCLK,
                               UARTConfig[UART_BAUDRATE][MCLK_SPEED][0],    // clock prescaler (is written into UCBRx bits); 8000000 / 9600 / 16 = 52 (for oversampling mode)
                               UARTConfig[UART_BAUDRATE][MCLK_SPEED][1],    // first modulation stage register setting (is written into UCBRFx bits of UCAxMCTLW); int(fractional_portion * 16)
                               UARTConfig[UART_BAUDRATE][MCLK_SPEED][2],    // second modulation stage register setting
                               EUSCI_A_UART_NO_PARITY,
                               EUSCI_A_UART_LSB_FIRST,
                               EUSCI_A_UART_ONE_STOP_BIT,
                               EUSCI_A_UART_MODE,
                               EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION);  // oversampling mode recommended for N >= 16
  GPIO_setAsOutputPinInline(GPIO_PORT_P2, GPIO_PIN1);
  EUSCI_UART_ENABLE;
}


// set all pins to port function (GPIO), output direction and clear the port interrupt flags
void resetGPIOPins()
{
  // there are 5 ports with 8 pins each
  GPIO_P1_RESET;
  GPIO_P2_RESET;
  GPIO_P3_RESET;
  GPIO_P4_RESET;
#ifndef DEBUG
  GPIO_PJ_RESET;
#endif // DEBUG
}


// configures the pins necessary for the control signaling (handhake signal)
void initCtrlPins()
{
  // Note: ACK, IND and PWR_SEL pins are already configured as output pins
  // configure MODE and REQ pins as inputs with a pull-down resistor (default state 0)
  GPIO_setAsInputPinInline(SPI_A_REQ_PORT, SPI_A_REQ_PIN);
  GPIO_setAsInputPinInline(SPI_C_REQ_PORT, SPI_C_REQ_PIN);
  GPIO_setAsInputPinInline(SPI_A_MODE_PORT, SPI_A_MODE_PIN);
  GPIO_setAsInputPinInline(SPI_C_MODE_PORT, SPI_C_MODE_PIN);
#ifndef INDICADE_ISR_WITH_FUTUREUSE // configure as inputs if pins are not used
  #ifdef SPI_A_FUTUREUSE_PORT
  GPIO_setAsInputPinInline(SPI_A_FUTUREUSE_PORT, SPI_A_FUTUREUSE_PIN);
  #endif
  #ifdef SPI_C_FUTUREUSE_PORT
  GPIO_setAsInputPinInline(SPI_C_FUTUREUSE_PORT, SPI_C_FUTUREUSE_PIN);
  #endif
#endif
}


// configures a port interrupt for the given pin
void configPortInterrupt(const uint8_t port, const uint8_t pin, SIGNALEDGE transition)
{
  GPIO_interruptEdgeSelectInline(port, pin, transition);
  GPIO_clearInterruptFlagInline(port, pin);
  GPIO_enableInterruptInline(port, pin);
}


// enables the port interrupts for the REQ pins
void enablePortInterrupts()
{
  configPortInterrupt(SPI_A_REQ_PORT, SPI_A_REQ_PIN, EDGE_RISING);    // configure with pull-down resistor to have REQ == 0 when not connected
  configPortInterrupt(SPI_C_REQ_PORT, SPI_C_REQ_PIN, EDGE_RISING);
}


// configures the clocks (dividers, sources)
void initClocks()
{
  // no need to inline...
  if (MCLK_SPEED_16MHZ == MCLK_SPEED)
  {
    CS_setDCOFreq(CS_DCORSEL_1, CS_DCOFSEL_4);  // 16 MHz
  } else if (MCLK_SPEED_8MHZ == MCLK_SPEED)
  {
    CS_setDCOFreq(CS_DCORSEL_0, CS_DCOFSEL_6);  // 8 MHz (Note: high frequency option is CS_DCORSEL_1, CS_DCOFSEL_3)
  } else
  {
    CS_setDCOFreq(CS_DCORSEL_0, CS_DCOFSEL_0);  // 1 MHz
  }
  CS_clockSignalInit(CS_ACLK, CS_VLOCLK_SELECT, CS_CLOCK_DIVIDER_1);  // set ACLK = VLO (typically 9.4 kHz)
  CS_clockSignalInit(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1); // set SMCLK = DCO with frequency divider of 1
  CS_clockSignalInit(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);  // set MCLK = DCO with frequency divider of 1
}


// configures the FRAM (wait states, interrupts)
void initFRAM()
{
  // no need to inline...
  FRAM_enableInterrupt(FRAM_UNCORRECTABLE_BIT_INTERRUPT + FRAM_ACCESS_TIME_ERROR_INTERRUPT);  // enable uncorrectable bit error and access time interrupt

  // wait states must be configured for frequencies higher than 8 MHz
  if (MCLK_SPEED >= NUM_OF_MCLK_SPEEDS)
  {
    LED_ERROR_ON;
  } else if (MCLK_SPEED_16MHZ == MCLK_SPEED)
  {
    FRAM_configureWaitStateControl(FRAM_ACCESS_TIME_CYCLES_1);
  } else
  {
    FRAM_configureWaitStateControl(FRAM_ACCESS_TIME_CYCLES_0);  // not wait state needed for 8 MHz and below
  }
  //FRAM_delayPowerUpFromLPM(FRAM_DELAY_FROM_LPM_ENABLE);
}


// initializes the MSP430
void initMSP430()
{
  // Note: after a BOR, all port pins are high-impedance with Schmitt triggers and their module functions disabled to prevent any cross currents
  WDT_A_hold(WDT_A_BASE);   // stop Watchdog (the watchdog timer powers up active in watchdog mode after reset)
  initFRAM();               // RAM before Clocks!
  initClocks();             // Clocks before other settings!
  resetGPIOPins();          // configure all pins in output mode
  initCtrlPins();           // configure the GPIO pins needed for the handshake signaling
#ifdef PUSH_BUTTON_PORT
  GPIO_setAsInputPinWithPullDownresistorInline(PUSH_BUTTON_PORT, PUSH_BUTTON_PIN);  // init the user push-button
#endif // PUSH_BUTTON_PORT

  // now init eUSCI modules
//#if defined(DEBUG) || defined(LOG_ERRORS)     // init anyway to print out stats
  initUART();
//#endif
  initSPI();

  // disable the GPIO power-on default high-impedance mode to activate previously configured port settings
  PM5CTL0 &= ~LOCKLPM5;     // PMM_unlockLPM5()
  enablePortInterrupts();

  // log some information
  LOG_INFO("\r\n" LOG_LINE "\r\nMSP430FR5969 initialization\r\n");
  LOG_INFO_2("ACLK frequency: %U kHz\r\nSMCLK frequency: %U MHz", CS_getACLK() / 1000, CS_getSMCLK() / 1000000);
}


