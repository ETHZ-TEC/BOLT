/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//*****************************************************************************
//eusci_uart.c - Driver for the UART Module.
//THIS FILE IS INCLUDED FOR BACKWARD COMPATIBILTY. PLEASE DO NOT USE THIS FILE
//AND INCLUDED APIS FOR NEW APPLICATIONS. PLEASE REFER eusci_a_uart.c 
//FOR NEW PROJECTS
//*****************************************************************************

#include "inc/hw_regaccess.h"
#include "assert.h"
#include "eusci_uart.h"
#include "eusci_euartbaudrate.h"
#ifdef  __IAR_SYSTEMS_ICC__
#include "deprecated/IAR/msp430xgeneric.h"
#else
#include "deprecated/CCS/msp430xgeneric.h"
#endif

//*****************************************************************************
//
//! DEPRECATED. Please use EUSCI_UART_initAdvance() to initialize the 
//! EUSCI_UART module
//!
//! Initializes the UART block.
//!
//! \param baseAddress is the base address of the UART module.
//! \param selectClockSource selects Clock source. Valid values are
//!         \b EUSCI_UART_CLOCKSOURCE_SMCLK
//!         \b EUSCI_UART_CLOCKSOURCE_ACLK
//! \param clockSourceFrequency is the frequency of the slected clock source
//! \param desiredUartClock is the desired clock rate for UART communication
//! \param parity is the desired parity. Valid values are
//!        \b EUSCI_UART_NO_PARITY  [Default Value],
//!        \b EUSCI_UART_ODD_PARITY,
//!        \b EUSCI_UART_EVEN_PARITY
//! \param msborLsbFirst controls direction of receive and transmit shift
//!     register. Valid values are
//!        \b EUSCI_UART_MSB_FIRST
//!        \b EUSCI_UART_LSB_FIRST [Default Value]
//! \param numberofStopBits indicates one/two STOP bits
//!      Valid values are
//!        \b EUSCI_UART_ONE_STOP_BIT [Default Value]
//!        \b EUSCI_UART_TWO_STOP_BITS
//! \param uartMode selects the mode of operation
//!      Valid values are
//!        \b EUSCI_UART_MODE  [Default Value],
//!        \b EUSCI_UART_IDLE_LINE_MULTI_PROCESSOR_MODE,
//!        \b EUSCI_UART_ADDRESS_BIT_MULTI_PROCESSOR_MODE,
//!        \b EUSCI_UART_AUTOMATIC_BAUDRATE_DETECTION_MODE
//! \param overSampling indicates low frequency or oversampling baud generation
//!      Valid values are
//!        \b EUSCI_UART_OVERSAMPLING_BAUDRATE_GENERATION
//!        \b EUSCI_UART_LOW_FREQUENCY_BAUDRATE_GENERATION
//!
//! Upon successful initialization of the UART block, this function
//! will have initialized the module, but the UART block still remains
//! disabled and must be enabled with EUSCI_UART_enable()
//!
//! Modified bits are \b UCPEN, \b UCPAR, \b UCMSB, \b UC7BIT, \b UCSPB,
//! \b UCMODEx, \b UCSYNC bits of \b UCAxCTL0 and \b UCSSELx,
//! \b UCSWRST bits of \b UCAxCTL1
//!
//! \return STATUS_SUCCESS or
//!         STATUS_FAIL of the initialization process
//
//*****************************************************************************
unsigned short EUSCI_UART_init ( uint32_t baseAddress,
    uint8_t selectClockSource,
    uint32_t clockSourceFrequency,
    uint32_t desiredUartBaudRate,
    uint8_t parity,
    uint16_t msborLsbFirst,
    uint16_t numberofStopBits,
    uint16_t uartMode,
    uint8_t overSampling
    )
{
    assert(
        (EUSCI_UART_MODE == uartMode) ||
        (EUSCI_UART_IDLE_LINE_MULTI_PROCESSOR_MODE == uartMode) ||
        (EUSCI_UART_ADDRESS_BIT_MULTI_PROCESSOR_MODE == uartMode) ||
        (EUSCI_UART_AUTOMATIC_BAUDRATE_DETECTION_MODE == uartMode)
        );

    assert(
        (EUSCI_UART_CLOCKSOURCE_ACLK == selectClockSource) ||
        (EUSCI_UART_CLOCKSOURCE_SMCLK == selectClockSource)
        );

    assert(
        (EUSCI_UART_MSB_FIRST == msborLsbFirst) ||
        (EUSCI_UART_LSB_FIRST == msborLsbFirst)
        );

    assert(
        (EUSCI_UART_ONE_STOP_BIT == numberofStopBits) ||
        (EUSCI_UART_TWO_STOP_BITS == numberofStopBits)
        );

    assert(
        (EUSCI_UART_NO_PARITY == parity) ||
        (EUSCI_UART_ODD_PARITY == parity) ||
        (EUSCI_UART_EVEN_PARITY == parity)
        );
		
	if(EUSCI_UART_LOW_FREQUENCY_BAUDRATE_GENERATION == overSampling)
	{
		assert ( clockSourceFrequency >= (desiredUartBaudRate * 3));
	
		if( clockSourceFrequency < (3 * desiredUartBaudRate))
			return STATUS_FAIL;
	}
	else
	{
		assert ( clockSourceFrequency >= (desiredUartBaudRate * 6));
	
		if( clockSourceFrequency < (6 * desiredUartBaudRate))
			return STATUS_FAIL;
	}

    uint8_t retVal = STATUS_SUCCESS;
    uint16_t UCAxBRW_value = 0x00;
    uint16_t UCAxMCTL_value = 0x00;

    //Disable the USCI Module
    HWREG16(baseAddress + OFS_UCAxCTLW0) |= UCSWRST;

    //Clock source select
    HWREG16(baseAddress + OFS_UCAxCTLW0) &= ~UCSSEL_3;
    HWREG16(baseAddress + OFS_UCAxCTLW0) |= selectClockSource;

    //MSB, LSB select
    HWREG16(baseAddress + OFS_UCAxCTLW0) &= ~UCMSB;
    HWREG16(baseAddress + OFS_UCAxCTLW0) |= msborLsbFirst;


    //UCSPB = 0(1 stop bit) OR 1(2 stop bits)
    HWREG16(baseAddress + OFS_UCAxCTLW0) &= ~UCSPB;
    HWREG16(baseAddress + OFS_UCAxCTLW0) |= numberofStopBits;


    //Parity
    switch (parity){
        case EUSCI_UART_NO_PARITY:
            //No Parity
            HWREG16(baseAddress + OFS_UCAxCTLW0) &= ~UCPEN;
            break;
        case EUSCI_UART_ODD_PARITY:
            //Odd Parity
            HWREG16(baseAddress + OFS_UCAxCTLW0) |= UCPEN;
            HWREG16(baseAddress + OFS_UCAxCTLW0) &= ~UCPAR;
            break;
        case EUSCI_UART_EVEN_PARITY:
            //Even Parity
            HWREG16(baseAddress + OFS_UCAxCTLW0) |= UCPEN;
            HWREG16(baseAddress + OFS_UCAxCTLW0) |= UCPAR;
            break;
    }

    //Calculate Baud rate divider values for Modulation control registers
    if ( STATUS_FAIL == eUARTBAUDRATE_calculateBaudDividers(clockSourceFrequency,
             desiredUartBaudRate,
             &UCAxBRW_value,
             &UCAxMCTL_value,
             overSampling
             )){
        return ( STATUS_FAIL) ;
    }

    //Modulation Control Registers
    HWREG16(baseAddress + OFS_UCAxBRW ) = UCAxBRW_value;
    HWREG16(baseAddress + OFS_UCAxMCTLW) = UCAxMCTL_value;

    //Asynchronous mode & 8 bit character select & clear mode
    HWREG16(baseAddress + OFS_UCAxCTLW0) &=  ~(UCSYNC +
                                             UC7BIT +
                                             UCMODE_3
                                             );

    //Configure  UART mode.
    HWREG16(baseAddress + OFS_UCAxCTLW0) |= uartMode ;

    //Reset UCRXIE, UCBRKIE, UCDORM, UCTXADDR, UCTXBRK
    HWREG16(baseAddress + OFS_UCAxCTLW0)  &= ~(UCRXEIE + UCBRKIE + UCDORM +
                                             UCTXADDR + UCTXBRK
                                             );

    return ( retVal) ;
}

//*****************************************************************************
//
//! Advanced initialization routine for the UART block. The values to be written
//! into the UCAxBRW and UCAxMCTLW registers should be pre-computed and passed
//! into the initialization function
//!
//! \param baseAddress is the base address of the UART module.
//! \param selectClockSource selects Clock source. Valid values are
//!         \b EUSCI_UART_CLOCKSOURCE_SMCLK
//!         \b EUSCI_UART_CLOCKSOURCE_ACLK
//! \param clockPrescalar is the value to be written into UCBRx bits
//! \param firstModReg  is First modulation stage register setting. This value 
//! 	is a pre-calculated value which can be obtained from the Device User’s 
//!		Guide.This value is written into UCBRFx bits of UCAxMCTLW.
//! \param secondModReg is Second modulation stage register setting. 
//! 	This value is a pre-calculated value which can be obtained from the Device 
//! 	User’s Guide. This value is written into UCBRSx bits of UCAxMCTLW.
//! \param parity is the desired parity. Valid values are
//!        \b EUSCI_UART_NO_PARITY  [Default Value],
//!        \b EUSCI_UART_ODD_PARITY,
//!        \b EUSCI_UART_EVEN_PARITY
//! \param msborLsbFirst controls direction of receive and transmit shift
//!     register. Valid values are
//!        \b EUSCI_UART_MSB_FIRST
//!        \b EUSCI_UART_LSB_FIRST [Default Value]
//! \param numberofStopBits indicates one/two STOP bits
//!      Valid values are
//!        \b EUSCI_UART_ONE_STOP_BIT [Default Value]
//!        \b EUSCI_UART_TWO_STOP_BITS
//! \param uartMode selects the mode of operation
//!      Valid values are
//!        \b EUSCI_UART_MODE  [Default Value],
//!        \b EUSCI_UART_IDLE_LINE_MULTI_PROCESSOR_MODE,
//!        \b EUSCI_UART_ADDRESS_BIT_MULTI_PROCESSOR_MODE,
//!        \b EUSCI_UART_AUTOMATIC_BAUDRATE_DETECTION_MODE
//! \param overSampling indicates low frequency or oversampling baud generation
//!      Valid values are
//!        \b EUSCI_UART_OVERSAMPLING_BAUDRATE_GENERATION
//!        \b EUSCI_UART_LOW_FREQUENCY_BAUDRATE_GENERATION
//!
//! Upon successful initialization of the UART block, this function
//! will have initialized the module, but the UART block still remains
//! disabled and must be enabled with EUSCI_UART_enable()
//!
//! Modified bits are \b UCPEN, \b UCPAR, \b UCMSB, \b UC7BIT, \b UCSPB,
//! \b UCMODEx, \b UCSYNC bits of \b UCAxCTL0 and \b UCSSELx,
//! \b UCSWRST bits of \b UCAxCTL1
//!
//! \return STATUS_SUCCESS or
//!         STATUS_FAIL of the initialization process
//
//*****************************************************************************
unsigned short EUSCI_UART_initAdvance ( uint32_t baseAddress,
    uint8_t selectClockSource,
    uint16_t clockPrescalar,
    uint8_t firstModReg,
    uint8_t secondModReg,
    uint8_t parity,
    uint16_t msborLsbFirst,
    uint16_t numberofStopBits,
    uint16_t uartMode,
    uint8_t overSampling
    )
{
    assert(
        (EUSCI_UART_MODE == uartMode) ||
        (EUSCI_UART_IDLE_LINE_MULTI_PROCESSOR_MODE == uartMode) ||
        (EUSCI_UART_ADDRESS_BIT_MULTI_PROCESSOR_MODE == uartMode) ||
        (EUSCI_UART_AUTOMATIC_BAUDRATE_DETECTION_MODE == uartMode)
        );

    assert(
        (EUSCI_UART_CLOCKSOURCE_ACLK == selectClockSource) ||
        (EUSCI_UART_CLOCKSOURCE_SMCLK == selectClockSource)
        );

    assert(
        (EUSCI_UART_MSB_FIRST == msborLsbFirst) ||
        (EUSCI_UART_LSB_FIRST == msborLsbFirst)
        );

    assert(
        (EUSCI_UART_ONE_STOP_BIT == numberofStopBits) ||
        (EUSCI_UART_TWO_STOP_BITS == numberofStopBits)
        );

    assert(
        (EUSCI_UART_NO_PARITY == parity) ||
        (EUSCI_UART_ODD_PARITY == parity) ||
        (EUSCI_UART_EVEN_PARITY == parity)
        );


    uint8_t retVal = STATUS_SUCCESS;

    //Disable the USCI Module
    HWREG16(baseAddress + OFS_UCAxCTLW0) |= UCSWRST;

    //Clock source select
    HWREG16(baseAddress + OFS_UCAxCTLW0) &= ~UCSSEL_3;
    HWREG16(baseAddress + OFS_UCAxCTLW0) |= selectClockSource;

    //MSB, LSB select
    HWREG16(baseAddress + OFS_UCAxCTLW0) &= ~UCMSB;
    HWREG16(baseAddress + OFS_UCAxCTLW0) |= msborLsbFirst;


    //UCSPB = 0(1 stop bit) OR 1(2 stop bits)
    HWREG16(baseAddress + OFS_UCAxCTLW0) &= ~UCSPB;
    HWREG16(baseAddress + OFS_UCAxCTLW0) |= numberofStopBits;


    //Parity
    switch (parity){
        case EUSCI_UART_NO_PARITY:
            //No Parity
            HWREG16(baseAddress + OFS_UCAxCTLW0) &= ~UCPEN;
            break;
        case EUSCI_UART_ODD_PARITY:
            //Odd Parity
            HWREG16(baseAddress + OFS_UCAxCTLW0) |= UCPEN;
            HWREG16(baseAddress + OFS_UCAxCTLW0) &= ~UCPAR;
            break;
        case EUSCI_UART_EVEN_PARITY:
            //Even Parity
            HWREG16(baseAddress + OFS_UCAxCTLW0) |= UCPEN;
            HWREG16(baseAddress + OFS_UCAxCTLW0) |= UCPAR;
            break;
    }

    //BaudRate Control Register
    HWREG16(baseAddress + OFS_UCAxBRW ) = clockPrescalar;
    //Modulation Control Register
    HWREG16(baseAddress + OFS_UCAxMCTLW) = ((secondModReg <<8) + (firstModReg <<4) + overSampling );

    //Asynchronous mode & 8 bit character select & clear mode
    HWREG16(baseAddress + OFS_UCAxCTLW0) &=  ~(UCSYNC +
                                             UC7BIT +
                                             UCMODE_3
                                             );

    //Configure  UART mode.
    HWREG16(baseAddress + OFS_UCAxCTLW0) |= uartMode ;

    //Reset UCRXIE, UCBRKIE, UCDORM, UCTXADDR, UCTXBRK
    HWREG16(baseAddress + OFS_UCAxCTLW0)  &= ~(UCRXEIE + UCBRKIE + UCDORM +
                                             UCTXADDR + UCTXBRK
                                             );

    return ( retVal) ;
}
//*****************************************************************************
//
//! Transmits a byte from the UART Module.
//!
//! \param baseAddress is the base address of the UART module.
//! \param transmitData data to be transmitted from the UART module
//!
//! This function will place the supplied data into UART trasmit data register
//! to start transmission
//!
//! Modified register is \b UCAxTXBUF
//! \return None.
//
//*****************************************************************************
void EUSCI_UART_transmitData ( uint32_t baseAddress,
    uint8_t transmitData
    )
{
    HWREG16(baseAddress + OFS_UCAxTXBUF) = transmitData;
}

//*****************************************************************************
//
//! Receives a byte that has been sent to the UART Module.
//!
//! \param baseAddress is the base address of the UART module.
//!
//! This function reads a byte of data from the UART receive data Register.
//!
//! Modified register is \b UCAxRXBUF
//!
//! \return Returns the byte received from by the UART module, cast as an
//! uint8_t.
//
//*****************************************************************************
uint8_t EUSCI_UART_receiveData (uint32_t baseAddress)
{
    return ( HWREG16(baseAddress + OFS_UCAxRXBUF)) ;
}

//*****************************************************************************
//
//! Enables individual UART interrupt sources.
//!
//! \param baseAddress is the base address of the UART module.
//! \param mask is the bit mask of the interrupt sources to be enabled.
//!
//! Enables the indicated UART interrupt sources.  The interrupt flag is first
//! and then the corresponfing interrupt is enabled. Only the sources that
//! are enabled can be reflected to the processor interrupt; disabled sources
//! have no effect on the processor.
//! <b>Does not clear interrupt flags.</b>
//!
//! The mask parameter is the logical OR of any of the following:
//! - \b EUSCI_UART_RECEIVE_INTERRUPT -Receive interrupt
//! - \b EUSCI_UART_TRANSMIT_INTERRUPT - Transmit interrupt
//! - \b EUSCI_UART_RECEIVE_ERRONEOUSCHAR_INTERRUPT - Receive erroneous-character
//!                             interrupt enable
//! - \b EUSCI_UART_BREAKCHAR_INTERRUPT - Receive break character interrupt enable
//! - \b EUSCI_UART_STARTBIT_INTERRUPT - Start bit received interrupt enable
//! - \b EUSCI_UART_TRANSMIT_COMPLETE_INTERRUPT - Transmit complete interrupt 
//!                                                 enable
//!
//! Modified register is \b UCAxIFG, \b UCAxIE and \b UCAxCTL1
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_UART_enableInterrupt (uint32_t baseAddress,
    uint8_t mask
    )
{
   	assert(mask & (EUSCI_UART_RECEIVE_INTERRUPT |
   	     EUSCI_UART_TRANSMIT_INTERRUPT |
   	     EUSCI_UART_RECEIVE_ERRONEOUSCHAR_INTERRUPT |
   	     EUSCI_UART_BREAKCHAR_INTERRUPT |
         EUSCI_UART_STARTBIT_INTERRUPT |
         EUSCI_UART_TRANSMIT_COMPLETE_INTERRUPT
         ));
        
    uint8_t locMask;
    
    if(locMask = (mask & (EUSCI_UART_RECEIVE_INTERRUPT |
        EUSCI_UART_TRANSMIT_INTERRUPT |
        EUSCI_UART_STARTBIT_INTERRUPT |
        EUSCI_UART_TRANSMIT_COMPLETE_INTERRUPT
        )))
    {
        HWREG16(baseAddress + OFS_UCAxIE) |= locMask;
    }
    
    if(locMask = (mask & (EUSCI_UART_RECEIVE_ERRONEOUSCHAR_INTERRUPT |
        EUSCI_UART_BREAKCHAR_INTERRUPT
        )))
    {
        HWREG16(baseAddress + OFS_UCAxCTLW0) |= locMask;
    }
}

//*****************************************************************************
//
//! Disables individual UART interrupt sources.
//!
//! \param baseAddress is the base address of the UART module.
//! \param mask is the bit mask of the interrupt sources to be
//! disabled.
//!
//! Disables the indicated UART interrupt sources.  Only the sources that
//! are enabled can be reflected to the processor interrupt; disabled sources
//! have no effect on the processor.
//!
//! The mask parameter is the logical OR of any of the following:
//! - \b EUSCI_UART_RECEIVE_INTERRUPT -Receive interrupt
//! - \b EUSCI_UART_TRANSMIT_INTERRUPT - Transmit interrupt
//! - \b EUSCI_UART_RECEIVE_ERRONEOUSCHAR_INTERRUPT - Receive erroneous-character
//!                             interrupt enable
//! - \b EUSCI_UART_BREAKCHAR_INTERRUPT - Receive break character interrupt enable
//! - \b EUSCI_UART_STARTBIT_INTERRUPT - Start bit received interrupt enable
//! - \b EUSCI_UART_TRANSMIT_COMPLETE_INTERRUPT - Transmit complete interrupt 
//!                                                 enable
//!
//! Modified register is \b UCAxIFG, \b UCAxIE and \b UCAxCTL1
//! \return None.
//
//*****************************************************************************
void EUSCI_UART_disableInterrupt (uint32_t baseAddress,
    uint8_t mask
    )
{
   	assert(mask & (EUSCI_UART_RECEIVE_INTERRUPT |
   	     EUSCI_UART_TRANSMIT_INTERRUPT |
   	     EUSCI_UART_RECEIVE_ERRONEOUSCHAR_INTERRUPT |
   	     EUSCI_UART_BREAKCHAR_INTERRUPT |
         EUSCI_UART_STARTBIT_INTERRUPT |
         EUSCI_UART_TRANSMIT_COMPLETE_INTERRUPT
         ));
    
    uint8_t locMask;
        
    if(locMask = (mask & (EUSCI_UART_RECEIVE_INTERRUPT |
        EUSCI_UART_TRANSMIT_INTERRUPT |
        EUSCI_UART_STARTBIT_INTERRUPT |
        EUSCI_UART_TRANSMIT_COMPLETE_INTERRUPT
        )))
    {
        HWREG16(baseAddress + OFS_UCAxIE) &= ~locMask;
    }
    
    if(locMask = (mask & (EUSCI_UART_RECEIVE_ERRONEOUSCHAR_INTERRUPT |
        EUSCI_UART_BREAKCHAR_INTERRUPT
        )))
    {
        HWREG16(baseAddress + OFS_UCAxCTLW0) &= ~locMask;
    }
}

//*****************************************************************************
//
//! Gets the current UART interrupt status.
//!
//! \param baseAddress is the base address of the UART module.
//! \param mask is the masked interrupt flag status to be returned.
//!
//! This returns the interrupt status for the UART  module based on which
//! flag is passed. mask parameter can be either any of the following
//! selection.
//! - \b EUSCI_UART_RECEIVE_INTERRUPT_FLAG -Receive interrupt flag
//! - \b EUSCI_UART_TRANSMIT_INTERRUPT_FLAG - Transmit interrupt flag
//! - \b EUSCI_UART_STARTBIT_INTERRUPT_FLAG - Start bit received interrupt flag
//! - \b EUSCI_UART_TRANSMIT_COMPLETE_INTERRUPT_FLAG - Transmit complete interrupt
//!                                                    flag
//!
//! Modified register is \b UCAxIFG.
//!
//! \return The current interrupt status, returned as with the respective bits
//! set if the corresponding interrupt flag is set
//
//*****************************************************************************
uint8_t EUSCI_UART_getInterruptStatus (uint32_t baseAddress,
    uint8_t mask)
{
  	assert(mask & (EUSCI_UART_RECEIVE_INTERRUPT_FLAG |
		EUSCI_UART_TRANSMIT_INTERRUPT_FLAG |
        EUSCI_UART_STARTBIT_INTERRUPT_FLAG |
        EUSCI_UART_TRANSMIT_COMPLETE_INTERRUPT_FLAG
		));

    return ( HWREG16(baseAddress + OFS_UCAxIFG) & mask );
}

//*****************************************************************************
//
//! Clears UART interrupt sources.
//!
//! \param baseAddress is the base address of the UART module.
//! \param mask is a bit mask of the interrupt sources to be cleared.
//!
//! The UART interrupt source is cleared, so that it no longer asserts.
//! The highest interrupt flag is automatically cleared when an interrupt vector
//! generator is used.
//!
//! The mask parameter has the same definition as the mask parameter to
//! EUSCI_UART_enableInterrupt().
//!
//! Modified register is \b UCAxIFG
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_UART_clearInterruptFlag (uint32_t baseAddress, uint8_t mask)
{
    //Clear the UART interrupt source.
    HWREG16(baseAddress + OFS_UCAxIFG) &= ~(mask);
}

//*****************************************************************************
//
//! Enables the UART block.
//!
//! \param baseAddress is the base address of the USCI UART module.
//!
//! This will enable operation of the UART block.
//!
//! Modified register is \b UCAxCTL1
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_UART_enable (uint32_t baseAddress)
{
    //Reset the UCSWRST bit to enable the USCI Module
    HWREG16(baseAddress + OFS_UCAxCTLW0) &= ~(UCSWRST);
}

//*****************************************************************************
//
//! Disables the UART block.
//!
//! \param baseAddress is the base address of the USCI UART module.
//!
//! This will disable operation of the UART block.
//!
//! Modified register is \b UCAxCTL1
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_UART_disable (uint32_t baseAddress)
{
    //Set the UCSWRST bit to disable the USCI Module
    HWREG16(baseAddress + OFS_UCAxCTLW0) |= UCSWRST;
}

//*****************************************************************************
//
//! Gets the current UART status flags.
//!
//! \param baseAddress is the base address of the UART module.
//! \param mask is the masked interrupt flag status to be returned.
//!
//! This returns the status for the UART  module based on which
//! flag is passed. mask parameter can be either any of the following
//! selection.
//! - \b EUSCI_UART_LISTEN_ENABLE
//! - \b EUSCI_UART_FRAMING_ERROR
//! - \b EUSCI_UART_OVERRUN_ERROR
//! - \b EUSCI_UART_PARITY_ERROR
//! - \b EUSCI_UART_BREAK_DETECT
//! - \b EUSCI_UART_RECEIVE_ERROR
//! - \b EUSCI_UART_ADDRESS_RECEIVED
//! - \b EUSCI_UART_IDLELINE
//! - \b EUSCI_UART_BUSY
//!
//! Modified register is \b UCAxSTAT
//!
//! \return the masked status flag
//
//*****************************************************************************
uint8_t EUSCI_UART_queryStatusFlags (uint32_t baseAddress,
    uint8_t mask)
{
    assert( 0x00 != mask && (EUSCI_UART_LISTEN_ENABLE +
		EUSCI_UART_FRAMING_ERROR +
		EUSCI_UART_OVERRUN_ERROR +
		EUSCI_UART_PARITY_ERROR +
		EUSCI_UART_BREAK_DETECT +
		EUSCI_UART_RECEIVE_ERROR +
		EUSCI_UART_ADDRESS_RECEIVED +
		EUSCI_UART_IDLELINE +
		EUSCI_UART_BUSY
		));
		
    return ( HWREG16(baseAddress + OFS_UCAxSTATW) & mask );
}

//*****************************************************************************
//
//! Sets the UART module in dormant mode
//!
//! \param baseAddress is the base address of the UART module.
//!
//! Puts USCI in sleep mode
//! Only characters that are preceded by an idle-line or with address bit set
//! UCRXIFG. In UART mode with automatic baud-rate detection, only the
//! combination of a break and synch field sets UCRXIFG.
//!
//! Modified register is \b UCAxCTL1
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_UART_setDormant (uint32_t baseAddress)
{
    HWREG16(baseAddress + OFS_UCAxCTLW0) |= UCDORM;
}

//*****************************************************************************
//
//! Re-enables UART module from dormant mode
//!
//! \param baseAddress is the base address of the UART module.
//!
//! Not dormant. All received characters set UCRXIFG.
//!
//! Modified bits are \b UCDORM of \b UCAxCTL1 register.
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_UART_resetDormant (uint32_t baseAddress)
{
    HWREG16(baseAddress + OFS_UCAxCTLW0) &= ~UCDORM;
}

//*****************************************************************************
//
//! Transmits the next byte to be transmitted marked as address depending on
//! selected multiprocessor mode
//!
//! \param baseAddress is the base address of the UART module.
//! \param transmitAddress is the next byte to be transmitted
//!
//! Modified register is \b UCAxCTL1, \b UCAxTXBUF
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_UART_transmitAddress (uint32_t baseAddress,
    uint8_t transmitAddress)
{
    //Set UCTXADDR bit
    HWREG16(baseAddress + OFS_UCAxCTLW0) |= UCTXADDR;

    //Place next byte to be sent into the transmit buffer
    HWREG16(baseAddress + OFS_UCAxTXBUF) = transmitAddress;
}

//*****************************************************************************
//
//! Transmit break. Transmits a break with the next write to the transmit
//! buffer. In UART mode with automatic baud-rate detection,
//! EUSCI_UART_AUTOMATICBAUDRATE_SYNC(0x55) must be written into UCAxTXBUF to
//! generate the required break/synch fields.
//! Otherwise, DEFAULT_SYNC(0x00) must be written into the transmit buffer.
//! Also ensures module is ready for transmitting the next data
//!
//! \param baseAddress is the base address of the UART module.
//!
//! Modified register is \b UCAxCTL1, \b UCAxTXBUF
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_UART_transmitBreak (uint32_t baseAddress)
{
    //Set UCTXADDR bit
    HWREG16(baseAddress + OFS_UCAxCTLW0) |= UCTXBRK;

    //If current mode is automatic baud-rate detection
    if (EUSCI_UART_AUTOMATIC_BAUDRATE_DETECTION_MODE ==
        (HWREG16(baseAddress + OFS_UCAxCTLW0) &
         EUSCI_UART_AUTOMATIC_BAUDRATE_DETECTION_MODE)){
        HWREG16(baseAddress + OFS_UCAxTXBUF) = EUSCI_UART_AUTOMATICBAUDRATE_SYNC;
    } else   {
        HWREG16(baseAddress + OFS_UCAxTXBUF) = DEFAULT_SYNC;
    }

    //USCI TX buffer ready?
    while (!EUSCI_UART_getInterruptStatus(baseAddress, UCTXIFG)) ;
}

//*****************************************************************************
//
//! Returns the address of the RX Buffer of the UART for the DMA module.
//!
//! \param baseAddress is the base address of the UART module.
//!
//! Returns the address of the UART RX Buffer. This can be used in conjunction
//! with the DMA to store the received data directly to memory.
//!
//! \return None
//
//*****************************************************************************
uint32_t EUSCI_UART_getReceiveBufferAddressForDMA (uint32_t baseAddress)
{
    return ( baseAddress + OFS_UCAxRXBUF );
}

//*****************************************************************************
//
//! Returns the address of the TX Buffer of the UART for the DMA module.
//!
//! \param baseAddress is the base address of the UART module.
//!
//! Returns the address of the UART TX Buffer. This can be used in conjunction
//! with the DMA to obtain transmitted data directly from memory.
//!
//! \return None
//
//*****************************************************************************
uint32_t EUSCI_UART_getTransmitBufferAddressForDMA (uint32_t baseAddress)
{
    return ( baseAddress + OFS_UCAxTXBUF );
}

//*****************************************************************************
//
//! Sets the deglitch time
//!
//! \param baseAddress is the base address of the UART module.
//! \param deglitchTime is the selected deglitch time
//! 	Valid values are 
//! 		- \b EUSCI_UART_DEGLITCH_TIME_2ns
//! 		- \b EUSCI_UART_DEGLITCH_TIME_50ns
//! 		- \b EUSCI_UART_DEGLITCH_TIME_100ns
//! 		- \b EUSCI_UART_DEGLITCH_TIME_200ns
//!		
//!
//! Returns the address of the UART TX Buffer. This can be used in conjunction
//! with the DMA to obtain transmitted data directly from memory.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_UART_selectDeglitchTime(uint32_t baseAddress,
			uint32_t deglitchTime
			)
{
	assert((EUSCI_UART_DEGLITCH_TIME_2ns == deglitchTime) ||
        (EUSCI_UART_DEGLITCH_TIME_50ns == deglitchTime) ||
        (EUSCI_UART_DEGLITCH_TIME_100ns == deglitchTime) ||
        (EUSCI_UART_DEGLITCH_TIME_200ns == deglitchTime)
        );
    
    HWREG16(baseAddress + OFS_UCAxCTLW1) &= ~(UCGLIT1 + UCGLIT0);
    
    HWREG16(baseAddress + OFS_UCAxCTLW1) = deglitchTime;
}
//*****************************************************************************
//
//Close the Doxygen group.
//! @}
//
//*****************************************************************************
