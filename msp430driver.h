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
 * msp430driver.h
 * custom driver library functions / macros for the MSP430 to increase
 * performance (eliminate function call overhead)
 */

#ifndef MSP430DRIVER_H
#define MSP430DRIVER_H

#define USE_CUSTOM_DRIVERLIB        // uncomment this to use the custom driver library functions (inlined code, faster)

// LEDs
#ifdef USE_LEDS
  #define LED_STATUS_ON     ( GPIO_setOutputHighOnPinInline(LED_STATUS_PORT, LED_STATUS_PIN) )
  #define LED_STATUS_OFF    ( GPIO_setOutputLowOnPinInline(LED_STATUS_PORT, LED_STATUS_PIN) )
  #define LED_STATUS_TOGGLE ( GPIO_toggleOutputOnPinInline(LED_STATUS_PORT, LED_STATUS_PIN) )
#else
  #define LED_STATUS_ON     ( NOP4 )
  #define LED_STATUS_OFF    ( NOP4 )
  #define LED_STATUS_TOGGLE ( NOP4 )
#endif
#define LED_ERROR_ON        ( GPIO_setOutputHighOnPinInline(LED_ERROR_PORT, LED_ERROR_PIN) )   // let ERROR LED turn on
#define LED_ERROR_OFF       ( GPIO_setOutputLowOnPinInline(LED_ERROR_PORT, LED_ERROR_PIN) )

// GPIO
#define GPIO_P1_RESET       { P1SEL0 = 0; P1SEL1 = 0; P1REN = 0; P1OUT = 0; P1DIR = 0xff; P1IFG = 0; P1IE = 0; }    // set all pins of port 1 as output (low) and clear interrupt flag
#define GPIO_P2_RESET       { P2SEL0 = 0; P2SEL1 = 0; P2REN = 0; P2OUT = 0; P2DIR = 0xff; P2IFG = 0; P2IE = 0; }
#define GPIO_P3_RESET       { P3SEL0 = 0; P3SEL1 = 0; P3REN = 0; P3OUT = 0; P3DIR = 0xff; P3IFG = 0; P3IE = 0; }
#define GPIO_P4_RESET       { P4SEL0 = 0; P4SEL1 = 0; P4REN = 0; P4OUT = 0; P4DIR = 0xff; P4IFG = 0; P4IE = 0; }
#define GPIO_PJ_RESET       { PJSEL0 = 0; PJSEL1 = 0; PJOUT = 0; PJDIR = 0xff; }

// helper macros related to low-power modes and interrupts
#define ENTER_LPM4          ( __bis_SR_register(LPM4_bits) )        // enter LPM4 immediately
#define ENTER_LPM45         { PMM_disableSVSH(); PMM_regOff(); PMM_lockLPM5(); __bic_SR_register(GIE); __bis_SR_register(LPM4_bits); }
#define LPM4_TO_LPM0_AFTER_ISR  ( __bic_SR_register_on_exit(SCG0 + SCG1 + OSCOFF) )  // switch from LPM4 to LPM1 after the ISR
#define ENTER_LPM4_AFTER_ISR    ( __bis_SR_register_on_exit(LPM4_bits) )    // enter LPM4 after execution of ISR
#define EXIT_LPM_FROM_ISR       ( __bic_SR_register_on_exit(LPM4_bits) )    // exit the LPM from an ISR
#define ENABLE_INTERRUPTS       ( __bis_SR_register(GIE) )            // or use: __enable_interrupts()
#define DISABLE_INTERRUPTS      ( __bic_SR_register(GIE) )            // or use: __disable_interrupts()

// DMA, SPI and UART helper macros
#define SPI_ADDR(inst)            ( (FSMINST_APPL_PROC == inst) ? SPI_A : SPI_C )
#define DMA_ADDR(inst)            ( (FSMINST_APPL_PROC == inst) ? DMA_A : DMA_C )
#define ENABLE_SPI(inst)          ( EUSCI_SPI_ENABLE(SPI_ADDR(inst)) )
#define DISABLE_SPI(inst)         ( EUSCI_SPI_DISABLE(SPI_ADDR(inst)) )                     // Warning: must wait for the busy flag and read the received data before disabling the SPI
#define DMA_REMAINING_BYTES(inst) ( REG_VAL16(DMA_BASE + DMA_ADDR(inst) + 0x001A) )
#define SPI_WAIT_BSY(inst)        { NOP6; NOP6; NOP7; NOP7; } //{ while (EUSCI_SPI_ISBUSY(SPI_ADDR(inst))); }   // IMPORTANT: replace this with __delay_cycles(26); to make it deterministic!
#define UART_WAIT_FOR_TC          { while (!(EUSCI_UART_GETINTERRUPTSTATUS(0xff) & EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG)); }
#define UART_WAIT_BSY             { while (EUSCI_SPI_ISBUSY(EUSCI_A0_BASE)); }  // same register for SPI and UART (EUSCI A module)
#define UART_DISABLE              { UART_WAIT_BSY; EUSCI_UART_DISABLE; GPIO_setAsOutputPinInline(GPIO_PORT_P2, GPIO_PIN0); GPIO_setOutputLowOnPinInline(GPIO_PORT_P2, GPIO_PIN0); }
#define UART_ENABLE               { GPIO_setAsPeripheralModuleFunctionInputPinInline(GPIO_PORT_P2, GPIO_PIN0, GPIO_SECONDARY_MODULE_FUNCTION); EUSCI_UART_ENABLE; }
#define UART_SEND_BYTE(c)         { UART_WAIT_FOR_TC; EUSCI_UART_TRANSMITDATA(c); }
#ifdef DEBUG
  #define CHECK_SPI_RXBUF_OVR_FLAG(write) {\
                        if (write)\
                        {\
                          if (UCA1STATW & UCOE)   /* UCFE bit and other errors are not detected in SPI slave mode */\
                          {\
                            LOG_ERROR("ERROR: SPI A1 RX buffer overrun detected!");\
                          }\
                          if (UCB0STATW & UCOE)\
                          {\
                            LOG_ERROR("ERROR: SPI B0 RX buffer overrun detected!");\
                          }\
                        }\
                      }
#else
  #define CHECK_SPI_RXBUF_OVR_FLAG(write)
#endif // DEBUG

// helper macros for control signaling pins
#define ACK_PORT(inst)            ( (FSMINST_APPL_PROC == inst) ? SPI_A_ACK_PORT : SPI_C_ACK_PORT )
#define ACK_PIN(inst)             ( (FSMINST_APPL_PROC == inst) ? SPI_A_ACK_PIN : SPI_C_ACK_PIN )
#define REQ_PORT(inst)            ( (FSMINST_APPL_PROC == inst) ? SPI_A_REQ_PORT : SPI_C_REQ_PORT )
#define REQ_PIN(inst)             ( (FSMINST_APPL_PROC == inst) ? SPI_A_REQ_PIN : SPI_C_REQ_PIN )
#define MODE_PORT(inst)           ( (FSMINST_APPL_PROC == inst) ? SPI_A_MODE_PORT : SPI_C_MODE_PORT )
#define MODE_PIN(inst)            ( (FSMINST_APPL_PROC == inst) ? SPI_A_MODE_PIN : SPI_C_MODE_PIN )
#define IND_PORT(inst)            ( (FSMINST_APPL_PROC == inst) ? SPI_A_IND_PORT : SPI_C_IND_PORT )
#define IND_PIN(inst)             ( (FSMINST_APPL_PROC == inst) ? SPI_A_IND_PIN : SPI_C_IND_PIN )
#define SPI_ACK_HIGH(inst)        ( GPIO_setOutputHighOnPinInline(ACK_PORT(inst), ACK_PIN(inst)) )
#define SPI_ACK_LOW(inst)         ( GPIO_setOutputLowOnPinInline(ACK_PORT(inst), ACK_PIN(inst)) )
#define SPI_MODE_STATUS(inst)     ( GPIO_getInputPinValueInline(MODE_PORT(inst), MODE_PIN(inst)) )
#define SPI_IND_HIGH(inst)        ( GPIO_setOutputHighOnPinInline(IND_PORT(inst), IND_PIN(inst)) )
#define SPI_IND_LOW(inst)         ( GPIO_setOutputLowOnPinInline(IND_PORT(inst), IND_PIN(inst)) )
#define SPI_REQ_STATUS(inst)      ( GPIO_getInputPinValueInline(REQ_PORT(inst), REQ_PIN(inst)) )
#define SET_REQ_INTERRUPT_EDGE(inst, edge)  ( GPIO_interruptEdgeSelectInline(REQ_PORT(inst), REQ_PIN(inst), (edge)) )
#define CLEAR_REQ_IFG(inst)       ( GPIO_clearInterruptFlagInline(REQ_PORT(inst), REQ_PIN(inst)) )

// misc
#define PUSH_BUTTON_PRESSED       ( GPIO_getInputPinValueInline(PUSH_BUTTON_PORT, PUSH_BUTTON_PIN) == 1 )
#define WAIT(ms)                  ( __delay_cycles((uint32_t)ms * MCLK_CYCLES / 1000) )

#define NOP           ( __no_operation() )
#define NOP2          { __no_operation(); __no_operation(); }
#define NOP3          { __no_operation(); __no_operation(); __no_operation(); }
#define NOP4          { __no_operation(); __no_operation(); __no_operation(); __no_operation(); }
#define NOP5          { __no_operation(); __no_operation(); __no_operation(); __no_operation(); __no_operation(); }
#define NOP6          { __no_operation(); __no_operation(); __no_operation(); __no_operation(); __no_operation(); __no_operation(); }
#define NOP7          { __no_operation(); __no_operation(); __no_operation(); __no_operation(); __no_operation(); __no_operation(); __no_operation(); }



// custom driver library (inline functions or macros)
#ifdef USE_CUSTOM_DRIVERLIB


#define GPIO_GET_BASE_ADDRESS(port)             ( (port) < GPIO_PORT_P3 ? 0x0200 : ((port) < GPIO_PORT_PJ ? 0x0220 : 0x0320) )        // replaces privateGPIOGetBaseAddress()
#define DMA_DISABLETRANSFERS(chSel)             ( REG_VAL16(DMA_BASE + (chSel) + OFS_DMA0CTL) &= ~(DMAEN) )
#define DMA_ENABLETRANSFERS(chSel)              ( REG_VAL16(DMA_BASE + (chSel) + OFS_DMA0CTL) |= DMAEN )
#define DMA_CLEARINTERRUPT(chSel)               ( REG_VAL16(DMA_BASE + (chSel) + OFS_DMA0CTL) &= ~(DMAIFG) )
#define DMA_ENABLEINTERRUPT(chSel)              ( REG_VAL16(DMA_BASE + (chSel) + OFS_DMA0CTL) |= DMAIE )
#define DMA_DISABLEINTERRUPT(chSel)             ( REG_VAL16(DMA_BASE + (chSel) + OFS_DMA0CTL) &= ~(DMAIE) )
#define DMA_GETINTERRUPTSTATUS(chSel)           ( REG_VAL16(DMA_BASE + (chSel) + OFS_DMA0CTL) & DMAIFG )
#define EUSCI_SPI_ENABLE(spi)                   ( REG_VAL16((spi) + OFS_UCAxCTLW0) &= ~(UCSWRST) )                      // replaces EUSCI_A_SPI_enable and EUSCI_B_SPI_enable
#define EUSCI_SPI_DISABLE(spi)                  ( REG_VAL16((spi) + OFS_UCAxCTLW0) |= UCSWRST )                         // replaces EUSCI_A_SPI_disable and EUSCI_B_SPI_disable
#define EUSCI_SPI_ISBUSY(spi)                   ( REG_VAL16((spi) + (EUSCI_B0_BASE == (spi) ? OFS_UCBxSTATW : OFS_UCAxSTATW)) & UCBUSY )  // replaces EUSCI_A_SPI_isBusy and EUSCI_B_SPI_isBusy
#define EUSCI_SPI_TRANSMITDATA(spi, data)       ( REG_VAL16((spi) + OFS_UCAxTXBUF) = (data) )                           // replaces EUSCI_A_SPI_transmitData and EUSCI_B_SPI_transmitData, can be used for both SPIs
#define EUSCI_SPI_GETINTERRUPTSTATUS(spi, mask) ( REG_VAL16((spi) + (EUSCI_B0_BASE == (spi) ? OFS_UCBxIFG : OFS_UCAxIFG)) & (mask) )      // replaces EUSCI_A_SPI_getInterruptStatus and EUSCI_B_SPI_getInterruptStatus
#define EUSCI_SPI_CLEARINTERRUPTFLAG(spi, mask) ( REG_VAL16((spi) + (EUSCI_B0_BASE == (spi) ? OFS_UCBxIFG : OFS_UCAxIFG)) &= ~(mask) )    // replaces EUSCI_A_SPI_clearInterruptFlag and EUSCI_B_SPI_clearInterruptFlag
#define EUSCI_SPI_ENABLEINTERRUPT(spi, mask)    ( REG_VAL16((spi) + (EUSCI_B0_BASE == (spi) ? OFS_UCBxIE : OFS_UCAxIE)) |= (mask) )       // replaces EUSCI_A_SPI_enableInterrupt and EUSCI_B_SPI_enableInterrupt
#define EUSCI_UART_ENABLE                       ( REG_VAL16(EUSCI_A0_BASE + OFS_UCAxCTLW0) &= ~(UCSWRST) )
#define EUSCI_UART_DISABLE                      ( REG_VAL16(EUSCI_A0_BASE + OFS_UCAxCTLW0) |= UCSWRST )
#define EUSCI_UART_GETINTERRUPTSTATUS(mask)     ( REG_VAL16(EUSCI_A0_BASE + OFS_UCAxIFG) & (mask) )
#define EUSCI_UART_TRANSMITDATA(data)           ( REG_VAL16(EUSCI_A0_BASE + OFS_UCAxTXBUF) = (data) )
#define EUSCI_UART_RECEIVEDATA(rcv)             { if (!(REG_VAL16(EUSCI_A0_BASE + OFS_UCAxIE) & UCRXIE)) { while (!(REG_VAL16(EUSCI_A0_BASE + OFS_UCAxIFG) & UCRXIFG)); } (rcv) = REG_VAL16(EUSCI_A0_BASE + OFS_UCAxRXBUF); }
#define EUSCI_SPI_RECEIVEDATA(spi)              ( REG_VAL16((spi) + OFS_UCAxRXBUF) )                              // replaces EUSCI_A_SPI_receiveData and EUSCI_B_SPI_receiveData
#define TIMER_A_GETCAPTURECOMPARECOUNT(t, ccr)  ( REG_VAL16((t) + OFS_TAxR + (ccr)) )
#define TIMER_A_SETCOMPAREVALUE(t, ccr, val)    ( REG_VAL16((t) + (ccr) + OFS_TAxR) = (val) )
#define EUSCI_SPI_RECEIVEBUFFERADDRESS(spi)     ( (spi) + OFS_UCAxRXBUF )       // replaces EUSCI_A_SPI_getReceiveBufferAddress and EUSCI_B_SPI_getReceiveBufferAddress
#define EUSCI_SPI_TRANSMITBUFFERADDRESS(spi)    ( (spi) + OFS_UCAxTXBUF )	    // replaces EUSCI_A_SPI_getTransmitBufferAddress and EUSCI_B_SPI_getTransmitBufferAddress

	

// The whole function is translated by the compiler to a single line of assembly code if the arguments are constants!
#pragma FUNC_ALWAYS_INLINE(GPIO_setOutputHighOnPinInline)
static __inline void GPIO_setOutputHighOnPinInline(const uint8_t selectedPort, const uint16_t selectedPins)
{
  ASSERT((GPIO_PORT_P1 == selectedPort) || (GPIO_PORT_P2 == selectedPort) ||
       (GPIO_PORT_P3 == selectedPort) || (GPIO_PORT_P4 == selectedPort) || (GPIO_PORT_PJ == selectedPort));
  ASSERT(0x00 != (selectedPins & (GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 +
                  GPIO_PIN3 + GPIO_PIN4 + GPIO_PIN5 +
                  GPIO_PIN6 + GPIO_PIN7)));
  uint32_t baseAddress = GPIO_GET_BASE_ADDRESS(selectedPort);
  switch (selectedPort)
  {
  case GPIO_PORT_P1:
  case GPIO_PORT_P3:
    REG_VAL8(baseAddress + OFS_P1OUT) |= (uint8_t)selectedPins;
    break;
  case GPIO_PORT_P2:
  case GPIO_PORT_P4:
    REG_VAL8(baseAddress + OFS_P2OUT) |= (uint8_t)selectedPins;
    break;
  case GPIO_PORT_PJ:
    REG_VAL16(baseAddress + OFS_PAOUT) |= selectedPins;
    break;
  }
}


#pragma FUNC_ALWAYS_INLINE(GPIO_toggleOutputOnPinInline)
static __inline void GPIO_toggleOutputOnPinInline(const uint8_t selectedPort, const uint16_t selectedPins)
{
  ASSERT((GPIO_PORT_P1 == selectedPort) || (GPIO_PORT_P2 == selectedPort) ||
       (GPIO_PORT_P3 == selectedPort) || (GPIO_PORT_P4 == selectedPort) || (GPIO_PORT_PJ == selectedPort));
  ASSERT(0x00 != (selectedPins & (GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 +
                  GPIO_PIN3 + GPIO_PIN4 + GPIO_PIN5 +
                  GPIO_PIN6 + GPIO_PIN7)));
  uint32_t baseAddress = GPIO_GET_BASE_ADDRESS(selectedPort);
  switch (selectedPort)
  {
  case GPIO_PORT_P1:
  case GPIO_PORT_P3:
    REG_VAL8(baseAddress + OFS_P1OUT) ^= (uint8_t)selectedPins;
    break;
  case GPIO_PORT_P2:
  case GPIO_PORT_P4:
    REG_VAL8(baseAddress + OFS_P2OUT) ^= (uint8_t)selectedPins;
    break;
  case GPIO_PORT_PJ:
    REG_VAL16(baseAddress + OFS_PAOUT) ^= selectedPins;
    break;
  }
}


#pragma FUNC_ALWAYS_INLINE(GPIO_setOutputLowOnPinInline)
static __inline void GPIO_setOutputLowOnPinInline(const uint8_t selectedPort, const uint16_t selectedPins)
{
  ASSERT((GPIO_PORT_P1 == selectedPort) || (GPIO_PORT_P2 == selectedPort) ||
       (GPIO_PORT_P3 == selectedPort) || (GPIO_PORT_P4 == selectedPort) || (GPIO_PORT_PJ == selectedPort));
  ASSERT(0x00 != (selectedPins & (GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 +
                  GPIO_PIN3 + GPIO_PIN4 + GPIO_PIN5 +
                  GPIO_PIN6 + GPIO_PIN7)));
  uint32_t baseAddress = GPIO_GET_BASE_ADDRESS(selectedPort);
  switch (selectedPort)
  {
  case GPIO_PORT_P1:
  case GPIO_PORT_P3:
    REG_VAL8(baseAddress + OFS_P1OUT) &= (uint8_t) ~selectedPins;
    break;
  case GPIO_PORT_P2:
  case GPIO_PORT_P4:
    REG_VAL8(baseAddress + OFS_P2OUT) &= (uint8_t) ~selectedPins;
    break;
  case GPIO_PORT_PJ:
    REG_VAL16(baseAddress + OFS_PAOUT) &= ~selectedPins;
    break;
  }
}


#pragma FUNC_ALWAYS_INLINE(GPIO_getInputPinValueInline)
static __inline uint8_t GPIO_getInputPinValueInline(uint8_t selectedPort, uint16_t selectedPins)
{
  ASSERT((GPIO_PORT_P1 == selectedPort) || (GPIO_PORT_P2 == selectedPort) ||
       (GPIO_PORT_P3 == selectedPort) || (GPIO_PORT_P4 == selectedPort) || (GPIO_PORT_PJ == selectedPort));
  ASSERT(0x00 != (selectedPins & (GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 +
                  GPIO_PIN3 + GPIO_PIN4 + GPIO_PIN5 +
                  GPIO_PIN6 + GPIO_PIN7)));
  uint32_t baseAddress = GPIO_GET_BASE_ADDRESS(selectedPort);
  uint16_t inputPinValue = 0;
  switch (selectedPort)
  {
  case GPIO_PORT_P1:
  case GPIO_PORT_P3:
    inputPinValue = REG_VAL8(baseAddress + OFS_P1IN) & ((uint8_t)selectedPins);
    break;
  case GPIO_PORT_P2:
  case GPIO_PORT_P4:
    inputPinValue = REG_VAL8(baseAddress + OFS_P2IN) & ((uint8_t)selectedPins);
    break;
  case GPIO_PORT_PJ:
    inputPinValue = REG_VAL16(baseAddress + OFS_PAIN) & (selectedPins);
    break;
  }
  if (inputPinValue > 0)
  {
    return GPIO_INPUT_PIN_HIGH;
  }

  return GPIO_INPUT_PIN_LOW;
}


#pragma FUNC_ALWAYS_INLINE(GPIO_clearInterruptFlagInline)
static __inline void GPIO_clearInterruptFlagInline(const uint8_t selectedPort, const uint16_t selectedPins)
{
  ASSERT((GPIO_PORT_P1 == selectedPort) || (GPIO_PORT_P2 == selectedPort) ||
       (GPIO_PORT_P3 == selectedPort) || (GPIO_PORT_P4 == selectedPort) || (GPIO_PORT_PJ == selectedPort));
  ASSERT(0x00 != (selectedPins & (GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 +
                  GPIO_PIN3 + GPIO_PIN4 + GPIO_PIN5 +
                  GPIO_PIN6 + GPIO_PIN7)));
  uint32_t baseAddress = GPIO_GET_BASE_ADDRESS(selectedPort);
  switch (selectedPort)
  {
  case GPIO_PORT_P1:
  case GPIO_PORT_P3:
    REG_VAL8(baseAddress + OFS_P1IFG) &= (uint8_t) ~selectedPins;
    break;
  case GPIO_PORT_P2:
  case GPIO_PORT_P4:
    REG_VAL8(baseAddress + OFS_P2IFG) &= (uint8_t) ~selectedPins;
    break;
  case GPIO_PORT_PJ:
    REG_VAL16(baseAddress + OFS_PAIFG) &= ~selectedPins;
    break;
  }
}


#pragma FUNC_ALWAYS_INLINE(GPIO_setAsPeripheralModuleFunctionInputPinInline)
static __inline void GPIO_setAsPeripheralModuleFunctionInputPinInline(uint8_t selectedPort,
                                    uint16_t selectedPins,
                                    uint8_t mode)
{
  ASSERT((GPIO_PORT_P1 == selectedPort) || (GPIO_PORT_P2 == selectedPort) ||
       (GPIO_PORT_P3 == selectedPort) || (GPIO_PORT_P4 == selectedPort) || (GPIO_PORT_PJ == selectedPort));
  ASSERT(0x00 != (selectedPins & (GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 +
                  GPIO_PIN3 + GPIO_PIN4 + GPIO_PIN5 +
                  GPIO_PIN6 + GPIO_PIN7)));
  uint32_t baseAddress = GPIO_GET_BASE_ADDRESS(selectedPort);
  switch (selectedPort)
  {
  case GPIO_PORT_P1:
  case GPIO_PORT_P3:
  {
    switch (mode)
    {
    case GPIO_PRIMARY_MODULE_FUNCTION:
      REG_VAL8(baseAddress + OFS_P1SEL0) |= (uint8_t)selectedPins;
      REG_VAL8(baseAddress + OFS_P1SEL1) &= (uint8_t) ~selectedPins;
      REG_VAL8(baseAddress + OFS_P1DIR) &= (uint8_t) ~selectedPins;
      break;
    case GPIO_SECONDARY_MODULE_FUNCTION:
      REG_VAL8(baseAddress + OFS_P1SEL0) &= (uint8_t) ~selectedPins;
      REG_VAL8(baseAddress + OFS_P1SEL1) |= (uint8_t)selectedPins;
      REG_VAL8(baseAddress + OFS_P1DIR) &= (uint8_t) ~selectedPins;
      break;
    case GPIO_TERNARY_MODULE_FUNCTION:
      REG_VAL8(baseAddress + OFS_P1SEL0) |= (uint8_t)selectedPins;
      REG_VAL8(baseAddress + OFS_P1SEL1) |= (uint8_t)selectedPins;
      REG_VAL8(baseAddress + OFS_P1DIR) &= (uint8_t) ~selectedPins;
    }
    break;
  }
  case GPIO_PORT_P2:
  case GPIO_PORT_P4:
  {
    switch (mode)
    {
    case GPIO_PRIMARY_MODULE_FUNCTION:
      REG_VAL8(baseAddress + OFS_P2SEL0) |= (uint8_t)selectedPins;
      REG_VAL8(baseAddress + OFS_P2SEL1) &= (uint8_t) ~selectedPins;
      REG_VAL8(baseAddress + OFS_P2DIR) &= (uint8_t) ~selectedPins;
      break;
    case GPIO_SECONDARY_MODULE_FUNCTION:
      REG_VAL8(baseAddress + OFS_P2SEL0) &= (uint8_t) ~selectedPins;
      REG_VAL8(baseAddress + OFS_P2SEL1) |= (uint8_t)selectedPins;
      REG_VAL8(baseAddress + OFS_P2DIR) &= (uint8_t) ~selectedPins;
      break;
    case GPIO_TERNARY_MODULE_FUNCTION:
      REG_VAL8(baseAddress + OFS_P2SEL0) |= (uint8_t)selectedPins;
      REG_VAL8(baseAddress + OFS_P2SEL1) |= (uint8_t)selectedPins;
      REG_VAL8(baseAddress + OFS_P2DIR) &= (uint8_t) ~selectedPins;
    }
    break;
  }
  case GPIO_PORT_PJ:
  {
    switch (mode)
    {
    case GPIO_PRIMARY_MODULE_FUNCTION:
      REG_VAL16(baseAddress + OFS_PASEL0) |= selectedPins;
      REG_VAL16(baseAddress + OFS_PASEL1) &= ~selectedPins;
      REG_VAL16(baseAddress + OFS_PADIR) &= ~selectedPins;
      break;
    case GPIO_SECONDARY_MODULE_FUNCTION:
      REG_VAL16(baseAddress + OFS_PASEL0) &= ~selectedPins;
      REG_VAL16(baseAddress + OFS_PASEL1) |= selectedPins;
      REG_VAL16(baseAddress + OFS_PADIR) &= ~selectedPins;
      break;
    case GPIO_TERNARY_MODULE_FUNCTION:
      REG_VAL16(baseAddress + OFS_PASEL0) |= selectedPins;
      REG_VAL16(baseAddress + OFS_PASEL1) |= selectedPins;
      REG_VAL16(baseAddress + OFS_PADIR) &= ~selectedPins;
    }
    break;
  }
  }
}


#pragma FUNC_ALWAYS_INLINE(GPIO_setAsOutputPinInline)
static __inline void GPIO_setAsOutputPinInline(uint8_t selectedPort, uint16_t selectedPins)
{
  ASSERT((GPIO_PORT_P1 == selectedPort) || (GPIO_PORT_P2 == selectedPort) ||
       (GPIO_PORT_P3 == selectedPort) || (GPIO_PORT_P4 == selectedPort) || (GPIO_PORT_PJ == selectedPort));
  ASSERT(0x00 != (selectedPins & (GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 +
                  GPIO_PIN3 + GPIO_PIN4 + GPIO_PIN5 +
                  GPIO_PIN6 + GPIO_PIN7)));
  uint32_t baseAddress = GPIO_GET_BASE_ADDRESS(selectedPort);
  switch (selectedPort)
  {
  case GPIO_PORT_P1:
  case GPIO_PORT_P3:
    REG_VAL8(baseAddress + OFS_P1SEL0) &= (uint8_t) ~selectedPins;
    REG_VAL8(baseAddress + OFS_P1SEL1) &= (uint8_t) ~selectedPins;
    REG_VAL8(baseAddress + OFS_P1DIR) |= (uint8_t)selectedPins;
    break;
  case GPIO_PORT_P2:
  case GPIO_PORT_P4:
    REG_VAL8(baseAddress + OFS_P2SEL0) &= (uint8_t) ~selectedPins;
    REG_VAL8(baseAddress + OFS_P2SEL1) &= (uint8_t) ~selectedPins;
    REG_VAL8(baseAddress + OFS_P2DIR) |= (uint8_t)selectedPins;
    break;
  case GPIO_PORT_PJ:
    REG_VAL16(baseAddress + OFS_PASEL0) &= ~selectedPins;
    REG_VAL16(baseAddress + OFS_PASEL1) &= ~selectedPins;
    REG_VAL16(baseAddress + OFS_PADIR) |= selectedPins;
    break;
  }
}


#pragma FUNC_ALWAYS_INLINE(GPIO_setAsInputPinWithPullDownresistorInline)
static __inline void GPIO_setAsInputPinWithPullDownresistorInline(uint8_t selectedPort, uint16_t selectedPins)
{
  ASSERT((GPIO_PORT_P1 == selectedPort) || (GPIO_PORT_P2 == selectedPort) ||
       (GPIO_PORT_P3 == selectedPort) || (GPIO_PORT_P4 == selectedPort) || (GPIO_PORT_PJ == selectedPort));
  ASSERT(0x00 != (selectedPins & (GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 +
                  GPIO_PIN3 + GPIO_PIN4 + GPIO_PIN5 +
                  GPIO_PIN6 + GPIO_PIN7)));
  uint32_t baseAddress = GPIO_GET_BASE_ADDRESS(selectedPort);
  switch (selectedPort)
  {
  case GPIO_PORT_P1:
  case GPIO_PORT_P3:
    REG_VAL8(baseAddress + OFS_P1SEL0) &= (uint8_t) ~selectedPins;
    REG_VAL8(baseAddress + OFS_P1SEL1) &= (uint8_t) ~selectedPins;
    REG_VAL8(baseAddress + OFS_P1DIR) &= (uint8_t) ~selectedPins;
    REG_VAL8(baseAddress + OFS_P1REN) |= (uint8_t)selectedPins;
    REG_VAL8(baseAddress + OFS_P1OUT) &= (uint8_t) ~selectedPins;
    break;
  case GPIO_PORT_P2:
  case GPIO_PORT_P4:
    REG_VAL8(baseAddress + OFS_P2SEL0) &= (uint8_t) ~selectedPins;
    REG_VAL8(baseAddress + OFS_P2SEL1) &= (uint8_t) ~selectedPins;
    REG_VAL8(baseAddress + OFS_P2DIR) &= (uint8_t) ~selectedPins;
    REG_VAL8(baseAddress + OFS_P2REN) |= (uint8_t)selectedPins;
    REG_VAL8(baseAddress + OFS_P2OUT) &= (uint8_t) ~selectedPins;
    break;
  case GPIO_PORT_PJ:
    REG_VAL16(baseAddress + OFS_PASEL0) &= ~selectedPins;
    REG_VAL16(baseAddress + OFS_PASEL1) &= ~selectedPins;
    REG_VAL16(baseAddress + OFS_PADIR) &= ~selectedPins;
    REG_VAL16(baseAddress + OFS_PAREN) |= selectedPins;
    REG_VAL16(baseAddress + OFS_PAOUT) &= ~selectedPins;
    break;
  }
}


#pragma FUNC_ALWAYS_INLINE(GPIO_setAsInputPinInline)
static __inline void GPIO_setAsInputPinInline(uint8_t selectedPort, uint16_t selectedPins)
{
  ASSERT((GPIO_PORT_P1 == selectedPort) || (GPIO_PORT_P2 == selectedPort) ||
       (GPIO_PORT_P3 == selectedPort) || (GPIO_PORT_P4 == selectedPort) || (GPIO_PORT_PJ == selectedPort));
  ASSERT(0x00 != (selectedPins & (GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 +
                  GPIO_PIN3 + GPIO_PIN4 + GPIO_PIN5 +
                  GPIO_PIN6 + GPIO_PIN7)));

  uint32_t baseAddress = GPIO_GET_BASE_ADDRESS(selectedPort);
  switch (selectedPort)
  {
  case GPIO_PORT_P1:
  case GPIO_PORT_P3:
    REG_VAL8(baseAddress + OFS_P1SEL0) &= (uint8_t) ~selectedPins;
    REG_VAL8(baseAddress + OFS_P1SEL1) &= (uint8_t) ~selectedPins;
    REG_VAL8(baseAddress + OFS_P1DIR) &= (uint8_t) ~selectedPins;
    REG_VAL8(baseAddress + OFS_P1REN) &= (uint8_t) ~selectedPins;
    break;
  case GPIO_PORT_P2:
  case GPIO_PORT_P4:
    REG_VAL8(baseAddress + OFS_P2SEL0) &= (uint8_t) ~selectedPins;
    REG_VAL8(baseAddress + OFS_P2SEL1) &= (uint8_t) ~selectedPins;
    REG_VAL8(baseAddress + OFS_P2DIR) &= (uint8_t) ~selectedPins;
    REG_VAL8(baseAddress + OFS_P2REN) &= (uint8_t) ~selectedPins;
    break;
  case GPIO_PORT_PJ:
    REG_VAL16(baseAddress + OFS_PASEL0) &= ~selectedPins;
    REG_VAL16(baseAddress + OFS_PASEL1) &= ~selectedPins;
    REG_VAL16(baseAddress + OFS_PADIR) &= ~selectedPins;
    REG_VAL16(baseAddress + OFS_PAREN) &= ~selectedPins;
    break;
  }
}


#pragma FUNC_ALWAYS_INLINE(GPIO_interruptEdgeSelectInline)
static __inline void GPIO_interruptEdgeSelectInline(uint8_t selectedPort,
                          uint16_t selectedPins,
                          uint8_t edgeSelect)
{
  ASSERT((GPIO_PORT_P1 == selectedPort) || (GPIO_PORT_P2 == selectedPort) ||
       (GPIO_PORT_P3 == selectedPort) || (GPIO_PORT_P4 == selectedPort) || (GPIO_PORT_PJ == selectedPort));
  ASSERT(0x00 != (selectedPins & (GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 +
                  GPIO_PIN3 + GPIO_PIN4 + GPIO_PIN5 +
                  GPIO_PIN6 + GPIO_PIN7)));
  uint32_t baseAddress = GPIO_GET_BASE_ADDRESS(selectedPort);
  ASSERT((edgeSelect == GPIO_HIGH_TO_LOW_TRANSITION) || (edgeSelect == GPIO_LOW_TO_HIGH_TRANSITION));
  switch (selectedPort)
  {
  case GPIO_PORT_P1:
  case GPIO_PORT_P3:
    if (GPIO_LOW_TO_HIGH_TRANSITION == edgeSelect)
    {
      REG_VAL8(baseAddress + OFS_P1IES) &= (uint8_t) ~selectedPins;
    } else
    {
      REG_VAL8(baseAddress + OFS_P1IES) |= (uint8_t)selectedPins;
    }
    break;
  case GPIO_PORT_P2:
  case GPIO_PORT_P4:
    if (GPIO_LOW_TO_HIGH_TRANSITION == edgeSelect)
    {
      REG_VAL8(baseAddress + OFS_P2IES) &= (uint8_t) ~selectedPins;
    } else
    {
      REG_VAL8(baseAddress + OFS_P2IES) |= (uint8_t)selectedPins;
    }
    break;
  case GPIO_PORT_PJ:
    if (GPIO_LOW_TO_HIGH_TRANSITION == edgeSelect)
    {
      REG_VAL16(baseAddress + OFS_PAIES) &= ~selectedPins;
    } else
    {
      REG_VAL16(baseAddress + OFS_PAIES) |= selectedPins;
    }
    break;
  }
}


#pragma FUNC_ALWAYS_INLINE(GPIO_enableInterruptInline)
static __inline void GPIO_enableInterruptInline(uint8_t selectedPort, uint16_t selectedPins)
{
  ASSERT((GPIO_PORT_P1 == selectedPort) || (GPIO_PORT_P2 == selectedPort) ||
       (GPIO_PORT_P3 == selectedPort) || (GPIO_PORT_P4 == selectedPort) || (GPIO_PORT_PJ == selectedPort));
  ASSERT(0x00 != (selectedPins & (GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 +
                  GPIO_PIN3 + GPIO_PIN4 + GPIO_PIN5 +
                  GPIO_PIN6 + GPIO_PIN7)));
  uint32_t baseAddress  = GPIO_GET_BASE_ADDRESS(selectedPort);
  switch (selectedPort)
  {
  case GPIO_PORT_P1:
  case GPIO_PORT_P3:
    REG_VAL8(baseAddress + OFS_P1IE) |= (uint8_t)selectedPins;
    break;
  case GPIO_PORT_P2:
  case GPIO_PORT_P4:
    REG_VAL8(baseAddress + OFS_P2IE) |= (uint8_t)selectedPins;
    break;
  case GPIO_PORT_PJ:
    REG_VAL16(baseAddress + OFS_PAIE) |= selectedPins;
    break;
  }
}


#pragma FUNC_ALWAYS_INLINE(GPIO_setAsPeripheralModuleFunctionOutputPinInline)
static __inline void GPIO_setAsPeripheralModuleFunctionOutputPinInline(uint8_t selectedPort,
                                     uint16_t selectedPins,
                                     uint8_t mode)
{
  ASSERT((GPIO_PORT_P1 == selectedPort) || (GPIO_PORT_P2 == selectedPort) ||
       (GPIO_PORT_P3 == selectedPort) || (GPIO_PORT_P4 == selectedPort) || (GPIO_PORT_PJ == selectedPort));
  ASSERT(0x00 != (selectedPins & (GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 +
                  GPIO_PIN3 + GPIO_PIN4 + GPIO_PIN5 +
                  GPIO_PIN6 + GPIO_PIN7)));
  uint32_t baseAddress = GPIO_GET_BASE_ADDRESS(selectedPort);
  switch (selectedPort)
  {
  case GPIO_PORT_P1:
  case GPIO_PORT_P3:
  {
    switch (mode)
    {
    case GPIO_PRIMARY_MODULE_FUNCTION:
      REG_VAL8(baseAddress + OFS_P1SEL0) |= (uint8_t)selectedPins;
      REG_VAL8(baseAddress + OFS_P1SEL1) &= (uint8_t) ~selectedPins;
      REG_VAL8(baseAddress + OFS_P1DIR) |= (uint8_t)selectedPins;
      break;
    case GPIO_SECONDARY_MODULE_FUNCTION:
      REG_VAL8(baseAddress + OFS_P1SEL0) &= (uint8_t) ~selectedPins;
      REG_VAL8(baseAddress + OFS_P1SEL1) |= (uint8_t)selectedPins;
      REG_VAL8(baseAddress + OFS_P1DIR) |= (uint8_t)selectedPins;
      break;
    case GPIO_TERNARY_MODULE_FUNCTION:
      REG_VAL8(baseAddress + OFS_P1SEL0) |= (uint8_t)selectedPins;
      REG_VAL8(baseAddress + OFS_P1SEL1) |= (uint8_t)selectedPins;
      REG_VAL8(baseAddress + OFS_P1DIR) |= (uint8_t)selectedPins;
    }
    break;
  }
  case GPIO_PORT_P2:
  case GPIO_PORT_P4:
  {
    switch (mode)
    {
    case GPIO_PRIMARY_MODULE_FUNCTION:
      REG_VAL8(baseAddress + OFS_P2SEL0) |= (uint8_t)selectedPins;
      REG_VAL8(baseAddress + OFS_P2SEL1) &= (uint8_t) ~selectedPins;
      REG_VAL8(baseAddress + OFS_P2DIR) |= (uint8_t)selectedPins;
      break;
    case GPIO_SECONDARY_MODULE_FUNCTION:
      REG_VAL8(baseAddress + OFS_P2SEL0) &= (uint8_t) ~selectedPins;
      REG_VAL8(baseAddress + OFS_P2SEL1) |= (uint8_t)selectedPins;
      REG_VAL8(baseAddress + OFS_P2DIR) |= (uint8_t)selectedPins;
      break;
    case GPIO_TERNARY_MODULE_FUNCTION:
      REG_VAL8(baseAddress + OFS_P2SEL0) |= (uint8_t)selectedPins;
      REG_VAL8(baseAddress + OFS_P2SEL1) |= (uint8_t)selectedPins;
      REG_VAL8(baseAddress + OFS_P2DIR) |= (uint8_t)selectedPins;
    }
    break;
  }
  case GPIO_PORT_PJ:
  {
    switch (mode)
    {
    case GPIO_PRIMARY_MODULE_FUNCTION:
      REG_VAL16(baseAddress + OFS_PASEL0) |= selectedPins;
      REG_VAL16(baseAddress + OFS_PASEL1) &= ~selectedPins;
      REG_VAL16(baseAddress + OFS_PADIR) |= selectedPins;
      break;
    case GPIO_SECONDARY_MODULE_FUNCTION:
      REG_VAL16(baseAddress + OFS_PASEL0) &= ~selectedPins;
      REG_VAL16(baseAddress + OFS_PASEL1) |= selectedPins;
      REG_VAL16(baseAddress + OFS_PADIR) |= selectedPins;
      break;
    case GPIO_TERNARY_MODULE_FUNCTION:
      REG_VAL16(baseAddress + OFS_PASEL0) |= selectedPins;
      REG_VAL16(baseAddress + OFS_PASEL1) |= selectedPins;
      REG_VAL16(baseAddress + OFS_PADIR) |= selectedPins;
    }
    break;
  }
  }
}


#pragma FUNC_ALWAYS_INLINE(GPIO_getInterruptStatusInline)
static __inline uint16_t GPIO_getInterruptStatusInline(uint8_t selectedPort, uint16_t selectedPins)
{
  ASSERT((GPIO_PORT_P1 == selectedPort) || (GPIO_PORT_P2 == selectedPort) ||
       (GPIO_PORT_P3 == selectedPort) || (GPIO_PORT_P4 == selectedPort) || (GPIO_PORT_PJ == selectedPort));
  ASSERT(0x00 != (selectedPins & (GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 +
                  GPIO_PIN3 + GPIO_PIN4 + GPIO_PIN5 +
                  GPIO_PIN6 + GPIO_PIN7)));
  uint32_t baseAddress = GPIO_GET_BASE_ADDRESS(selectedPort);
  uint16_t returnValue = 0x0;
  switch (selectedPort)
  {
  case GPIO_PORT_P1:
  case GPIO_PORT_P3:
    returnValue = (REG_VAL8(baseAddress + OFS_P1IFG) & ((uint8_t)selectedPins));
    break;
  case GPIO_PORT_P2:
  case GPIO_PORT_P4:
    returnValue = (REG_VAL8(baseAddress + OFS_P2IFG) & ((uint8_t)selectedPins));
    break;
  case GPIO_PORT_PJ:
    returnValue = (REG_VAL16(baseAddress + OFS_PAIFG) & selectedPins);
    break;
  }
  return returnValue;
}


// NOTE: this code is optimize for use with DMA channel 0 and 1 only! channel 2 won't work!
#pragma FUNC_ALWAYS_INLINE(DMA_initInline)
static __inline void DMA_initInline(uint8_t channelSelect,
                  uint16_t transferModeSelect,
                  uint16_t transferSize,
                  uint8_t triggerSourceSelect,
                  uint8_t transferUnitSelect,
                  uint8_t triggerTypeSelect)
{
  ASSERT((channelSelect <= DMA_CHANNEL_7) &&
       (transferModeSelect <= DMA_TRANSFER_REPEATED_BURSTBLOCK) &&
       (triggerSourceSelect <= DMA_TRIGGERSOURCE_31) &&
       (transferUnitSelect <= DMA_SIZE_SRCBYTE_DSTBYTE) &&
       (triggerTypeSelect <= DMA_TRIGGER_HIGH));
  // reset and set DMA channel x control register (set transfer mode, unit size and trigger type)
  REG_VAL16(DMA_BASE + channelSelect + OFS_DMA0CTL) = transferModeSelect + transferUnitSelect + triggerTypeSelect;
  REG_VAL16(DMA_BASE + channelSelect + OFS_DMA0SZ) = transferSize;
  // set DMA control 0 register
  if (channelSelect & DMA_CHANNEL_1)
  {
    REG_VAL16(DMA_BASE) &= 0x00FF;   // reset trigger select
    REG_VAL16(DMA_BASE) |= (triggerSourceSelect << 8);
  } else
  {
    REG_VAL16(DMA_BASE) &= 0xFF00;   // reset trigger select
    REG_VAL16(DMA_BASE) |= triggerSourceSelect;
  }
}


#pragma FUNC_ALWAYS_INLINE(DMA_setSrcAddressInline)
static __inline void DMA_setSrcAddressInline(uint8_t channelSelect,
                       uint32_t srcAddress,
                       uint16_t directionSelect)
{
  ASSERT(channelSelect <= DMA_CHANNEL_7);
  ASSERT(directionSelect <= DMA_DIRECTION_INCREMENT);
  // set the source address
  __data16_write_addr((unsigned short)(DMA_BASE + channelSelect + OFS_DMA0SA), srcAddress);
  // reset bits before setting them
  REG_VAL16(DMA_BASE + channelSelect + OFS_DMA0CTL) &= ~(DMASRCINCR_3);
  REG_VAL16(DMA_BASE + channelSelect + OFS_DMA0CTL) |= directionSelect;
}


#pragma FUNC_ALWAYS_INLINE(DMA_setDstAddressInline)
static __inline void DMA_setDstAddressInline(uint8_t channelSelect,
                       uint32_t dstAddress,
                       uint16_t directionSelect)
{
  ASSERT(channelSelect <= DMA_CHANNEL_7);
  ASSERT(directionSelect <= DMA_DIRECTION_INCREMENT);
  // set the destination address
  __data16_write_addr((unsigned short)(DMA_BASE + channelSelect + OFS_DMA0DA), dstAddress);
  // reset bits before setting them
  REG_VAL16(DMA_BASE + channelSelect + OFS_DMA0CTL) &= ~(DMADSTINCR_3);
  REG_VAL16(DMA_BASE + channelSelect + OFS_DMA0CTL) |= (directionSelect << 2);
}


// replaces EUSCI_A_SPI_slaveInit and EUSCI_B_SPI_slaveInit
#pragma FUNC_ALWAYS_INLINE(EUSCI_SPI_slaveInitInline)
static __inline void EUSCI_SPI_slaveInitInline(uint32_t baseAddress,
                      uint16_t msbFirst,
                      uint16_t clockPhase,
                      uint16_t clockPolarity,
                      uint16_t spiMode)
{
  ASSERT((EUSCI_A_SPI_MSB_FIRST == msbFirst) || (EUSCI_A_SPI_LSB_FIRST == msbFirst));
  ASSERT((EUSCI_A_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT == clockPhase) || (EUSCI_A_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT == clockPhase));
  ASSERT((EUSCI_A_SPI_CLOCKPOLARITY_INACTIVITY_HIGH == clockPolarity) || (EUSCI_A_SPI_CLOCKPOLARITY_INACTIVITY_LOW == clockPolarity));
  ASSERT((EUSCI_A_SPI_3PIN == spiMode) || (EUSCI_A_SPI_4PIN_UCxSTE_ACTIVE_HIGH == spiMode) || (EUSCI_A_SPI_4PIN_UCxSTE_ACTIVE_LOW == spiMode));
  // disable USCI module
  REG_VAL16(baseAddress + OFS_UCAxCTLW0) |= UCSWRST;
  // reset OFS_UCAxCTLW0/OFS_UCBxCTLW0 register
  REG_VAL16(baseAddress + OFS_UCAxCTLW0) &= ~(UCMSB + UC7BIT + UCMST + UCCKPL + UCCKPH + UCMODE_3);
  // clock polarity, phase select, msbFirst, SYNC, Mode0
  REG_VAL16(baseAddress + OFS_UCAxCTLW0) |= (clockPhase + clockPolarity + msbFirst + UCSYNC + spiMode);
}


// replaces EUSCI_A_UART_initAdvanceInline
#pragma FUNC_ALWAYS_INLINE(EUSCI_UART_initAdvanceInline)
static __inline void EUSCI_UART_initAdvanceInline(uint8_t selectClockSource,
                          uint16_t clockPrescalar,
                          uint8_t firstModReg,
                          uint8_t secondModReg,
                          uint8_t parity,
                          uint16_t msborLsbFirst,
                          uint16_t numberofStopBits,
                          uint16_t uartMode,
                          uint8_t overSampling)
{
  ASSERT((EUSCI_A_UART_MODE == uartMode) ||
       (EUSCI_A_UART_IDLE_LINE_MULTI_PROCESSOR_MODE == uartMode) ||
       (EUSCI_A_UART_ADDRESS_BIT_MULTI_PROCESSOR_MODE == uartMode) ||
       (EUSCI_A_UART_AUTOMATIC_BAUDRATE_DETECTION_MODE == uartMode));
  ASSERT((EUSCI_A_UART_CLOCKSOURCE_ACLK == selectClockSource) || (EUSCI_A_UART_CLOCKSOURCE_SMCLK == selectClockSource));
  ASSERT((EUSCI_A_UART_MSB_FIRST == msborLsbFirst) || (EUSCI_A_UART_LSB_FIRST == msborLsbFirst));
  ASSERT((EUSCI_A_UART_ONE_STOP_BIT == numberofStopBits) || (EUSCI_A_UART_TWO_STOP_BITS == numberofStopBits));
  ASSERT((EUSCI_A_UART_NO_PARITY == parity) || (EUSCI_A_UART_ODD_PARITY == parity) || (EUSCI_A_UART_EVEN_PARITY == parity));

  // disable the USCI module
  REG_VAL16(EUSCI_A0_BASE + OFS_UCAxCTLW0) |= UCSWRST;
  // clock source select
  REG_VAL16(EUSCI_A0_BASE + OFS_UCAxCTLW0) &= ~UCSSEL_3;
  REG_VAL16(EUSCI_A0_BASE + OFS_UCAxCTLW0) |= selectClockSource;
  // MSB, LSB select
  REG_VAL16(EUSCI_A0_BASE + OFS_UCAxCTLW0) &= ~UCMSB;
  REG_VAL16(EUSCI_A0_BASE + OFS_UCAxCTLW0) |= msborLsbFirst;
  // UCSPB = 0(1 stop bit) OR 1(2 stop bits)
  REG_VAL16(EUSCI_A0_BASE + OFS_UCAxCTLW0) &= ~UCSPB;
  REG_VAL16(EUSCI_A0_BASE + OFS_UCAxCTLW0) |= numberofStopBits;
  // parity
  switch (parity)
  {
  case EUSCI_A_UART_NO_PARITY:
    // no parity
    REG_VAL16(EUSCI_A0_BASE + OFS_UCAxCTLW0) &= ~UCPEN;
    break;
  case EUSCI_A_UART_ODD_PARITY:
    // odd parity
    REG_VAL16(EUSCI_A0_BASE + OFS_UCAxCTLW0) |= UCPEN;
    REG_VAL16(EUSCI_A0_BASE + OFS_UCAxCTLW0) &= ~UCPAR;
    break;
  case EUSCI_A_UART_EVEN_PARITY:
    // even parity
    REG_VAL16(EUSCI_A0_BASE + OFS_UCAxCTLW0) |= UCPEN;
    REG_VAL16(EUSCI_A0_BASE + OFS_UCAxCTLW0) |= UCPAR;
    break;
  }
  // BaudRate control register
  REG_VAL16(EUSCI_A0_BASE + OFS_UCAxBRW ) = clockPrescalar;
  // modulation control register
  REG_VAL16(EUSCI_A0_BASE + OFS_UCAxMCTLW) = ((secondModReg << 8) + (firstModReg << 4) + overSampling );
  // asynchronous mode & 8 bit character select & clear mode
  REG_VAL16(EUSCI_A0_BASE + OFS_UCAxCTLW0) &=  ~(UCSYNC + UC7BIT + UCMODE_3);
  // configure  UART mode.
  REG_VAL16(EUSCI_A0_BASE + OFS_UCAxCTLW0) |= uartMode;
  // reset UCRXIE, UCBRKIE, UCDORM, UCTXADDR, UCTXBRK
  REG_VAL16(EUSCI_A0_BASE + OFS_UCAxCTLW0)  &= ~(UCRXEIE + UCBRKIE + UCDORM + UCTXADDR + UCTXBRK);
}



#else

#define GPIO_GET_BASE_ADDRESS(port)                             privateGPIOGetBaseAddress(port)
#define DMA_DISABLETRANSFERS(chSel)                             DMA_disableTransfers(chSel)
#define DMA_ENABLETRANSFERS(chSel)                              DMA_enableTransfers(chSel)
#define DMA_CLEARINTERRUPT(chSel)                               DMA_clearInterrupt(chSel)
#define DMA_ENABLEINTERRUPT(chSel)                              DMA_enableInterrupt(chSel)
#define DMA_GETINTERRUPTSTATUS(chSel)                           DMA_getInterruptStatus(chSel)
#define EUSCI_SPI_ISBUSY(spi)                                   (spi == EUSCI_B0_BASE ? EUSCI_B_SPI_isBusy(spi) : EUSCI_A_SPI_isBusy(spi))
#define EUSCI_SPI_TRANSMITDATA(spi, data)                       (spi == EUSCI_B0_BASE ? EUSCI_B_SPI_transmitData(spi, data) : EUSCI_A_SPI_transmitData(spi, data))
#define EUSCI_SPI_CLEARINTERRUPTFLAG(spi, mask)                 (spi == EUSCI_B0_BASE ? EUSCI_B_SPI_clearInterruptFlag(spi, mask) : EUSCI_A_SPI_clearInterruptFlag(spi, mask))
#define EUSCI_SPI_ENABLEINTERRUPT(spi, mask)                    (spi == EUSCI_B0_BASE ? EUSCI_B_SPI_enableInterrupt(spi, mask) : EUSCI_A_SPI_enableInterrupt(spi, mask))
#define EUSCI_SPI_GETINTERRUPTSTATUS(spi, mask)                 (spi == EUSCI_B0_BASE ? EUSCI_B_SPI_getInterruptStatus(spi, mask) : EUSCI_A_SPI_getInterruptStatus(spi, mask))
#define EUSCI_UART_ENABLE                                       EUSCI_A_UART_enable(EUSCI_A0_BASE)
#define EUSCI_UART_DISABLE                                      EUSCI_A_UART_disable(EUSCI_A0_BASE)
#define EUSCI_SPI_ENABLE(spi)                                   (spi == EUSCI_B0_BASE ? EUSCI_B_SPI_enable(spi) : EUSCI_A_SPI_enable(spi))
#define EUSCI_SPI_DISABLE(spi)                                  (spi == EUSCI_B0_BASE ? EUSCI_B_SPI_disable(spi) : EUSCI_A_SPI_disable(spi))
#define EUSCI_UART_GETINTERRUPTSTATUS(mask)                     EUSCI_A_UART_getInterruptStatus(EUSCI_A0_BASE, mask)
#define EUSCI_UART_TRANSMITDATA(data)                           EUSCI_A_UART_transmitData(EUSCI_A0_BASE, data)
#define EUSCI_UART_RECEIVEDATA(rcv)                             rcv = EUSCI_A_UART_receiveData(EUSCI_A0_BASE)
#define EUSCI_SPI_RECEIVEDATA(spi)                              (spi == EUSCI_B0_BASE ? EUSCI_B_SPI_receiveData(spi) : EUSCI_A_SPI_receiveData(spi))
#define TIMER_A_GETCAPTURECOMPARECOUNT(t, ccr)                  TIMER_A_getCaptureCompareCount(t, ccr)
#define TIMER_A_SETCOMPAREVALUE(t, ccr, val)                    TIMER_A_setCompareValue(t, ccr, val)
#define EUSCI_SPI_RECEIVEBUFFERADDRESS(spi)                     (spi == EUSCI_B0_BASE ? EUSCI_B_SPI_getReceiveBufferAddress(spi) : EUSCI_A_SPI_getReceiveBufferAddress(spi))
#define EUSCI_SPI_TRANSMITBUFFERADDRESS(spi)                    (spi == EUSCI_B0_BASE ? EUSCI_B_SPI_getTransmitBufferAddress(spi) : EUSCI_A_SPI_getTransmitBufferAddress(spi))

#define GPIO_setOutputHighOnPinInline(port, pin)                GPIO_setOutputHighOnPin(port, pin)
#define GPIO_toggleOutputOnPinInline(port, pin)                 GPIO_toggleOutputOnPin(port, pin)
#define GPIO_setOutputLowOnPinInline(port, pin)                 GPIO_setOutputLowOnPin(port, pin)
#define GPIO_clearInterruptFlagInline(port, pin)                GPIO_clearInterruptFlag(port, pin)
#define GPIO_getInputPinValueInline(port, pin)                  GPIO_getInputPinValue(port, pin)
#define GPIO_setAsPeripheralModuleFunctionInputPinInline(port, pin, mode)   GPIO_setAsPeripheralModuleFunctionInputPin(port, pin, mode)
#define GPIO_setAsOutputPinInline(port, pin)                    GPIO_setAsOutputPin(port, pin)
#define GPIO_setAsInputPinWithPullDownresistorInline(port, pin) GPIO_setAsInputPinWithPullDownresistor(port, pin)
#define GPIO_setAsInputPinInline(port, pin)                     GPIO_setAsInputPin(port, pin)
#define GPIO_interruptEdgeSelectInline(port, pin, edge)         GPIO_interruptEdgeSelect(port, pin, edge)
#define GPIO_enableInterruptInline(port, pin)                   GPIO_enableInterrupt(port, pin)
#define GPIO_setAsPeripheralModuleFunctionOutputPinInline(port, pin, mode)  GPIO_setAsPeripheralModuleFunctionOutputPin(port, pin, mode)
#define GPIO_getInterruptStatusInline(port, pin)                GPIO_getInterruptStatus(port, pin)
#define DMA_initInline(chSel, mode, size, src, unit, trigger)   DMA_init(chSel, mode, size, src, unit, trigger)
#define DMA_setSrcAddressInline(chSel, src, dir)                DMA_setSrcAddress(chSel, src, dir)
#define DMA_setDstAddressInline(chSel, dst, dir)                DMA_setDstAddress(chSel, dst, dir)
#define EUSCI_SPI_slaveInitInline(spi, msb, phase, pol, mode)   (spi == EUSCI_B0_BASE ? EUSCI_B_SPI_slaveInit(spi, msb, phase, pol, mode) : EUSCI_A_SPI_slaveInit(spi, msb, phase, pol, mode))
#define EUSCI_UART_initAdvanceInline(clksrc, pre, mod1, mod2, par, msb, stop, mode, sampl)  EUSCI_A_UART_initAdvance(EUSCI_A0_BASE, clksrc, pre, mod1, mod2, par, msb, stop, mode, sampl)


#endif // USE_CUSTOM_DRIVERLIB

#endif // MSP430DRIVER_H

