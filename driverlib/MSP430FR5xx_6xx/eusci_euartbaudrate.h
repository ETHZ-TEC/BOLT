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
//THIS FILE IS INCLUDED FOR BACKWARD COMPATIBILTY. PLEASE DO NOT USE THIS FILE
//AND INCLUDED APIS FOR NEW APPLICATIONS. PLEASE REFER 
//eusci_a_uartbaudrate.h FOR NEW PROJECTS
//*****************************************************************************

#ifndef __MSP430WARE_EUARTBAUDRATE_H__
#define __MSP430WARE_EUARTBAUDRATE_H__

//*****************************************************************************
//
//The following are the defines to include the required modules for this
//peripheral in msp430xgeneric.h file
//
//*****************************************************************************
#define __MSP430_HAS_EUSCI_Ax__

//*****************************************************************************
//
//The following value selects low frequency mode eand NOT the oversampling mode
//
//*****************************************************************************
#define eUARTBAUDRATE_LOW_FREQUENCY_BAUDRATE_GENERATION    0x00


typedef struct ERROR_STRUCTURE {
    double max_error;
    double max_positive_error;
    double max_negative_error;
}MAX_ERR;


extern double eUARTBAUDRATE_txTbit (uint16_t mode,
    uint16_t i,
    uint16_t s_mod,
    uint16_t f_mod
    );

extern MAX_ERR eUARTBAUDRATE_txError (uint16_t mode,
    uint16_t s_mod,
    uint16_t f_mod
    );

extern MAX_ERR eUARTBAUDRATE_rxError (uint16_t mode,
    uint16_t s_mod,
    uint16_t f_mod,
    double t_sync
    );

extern unsigned short eUARTBAUDRATE_calculateBaudDividers (
    double brclk_f,
    double baudrate_f,
    uint16_t *UCAxBRW_value,
    uint16_t *UCAxMCTL_value,
    uint8_t overSampling
    );
#endif
