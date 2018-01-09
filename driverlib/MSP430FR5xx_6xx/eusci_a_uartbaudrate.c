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
//
//eUSCI_uart_baudrate.c - Helper for the UART module
//
//*****************************************************************************
#include "inc/hw_regaccess.h"
#include "assert.h"
#include "eusci_a_uartbaudrate.h"
#ifdef  __IAR_SYSTEMS_ICC__
#include "deprecated/IAR/msp430xgeneric.h"
#else
#include "deprecated/CCS/msp430xgeneric.h"
#endif

#define EUSCI_A_UARTBAUDRATE_NO_BITS  1 + 8 + 1 + 1

double eusci_a_uartbaudrate_baudrate;
double eusci_a_uartbaudrate_brclk;

double eusci_a_uartbaudrate_lf_div_f;
double eusci_a_uartbaudrate_osr_div_f;
uint16_t eusci_a_uartbaudrate_lf_div_i, eusci_a_uartbaudrate_osr_div_i;

const unsigned short eusci_a_uartbaudrate_UCSx_mod[256][8] =
{   {0,    0,    0,    0,    0,    0,    0,    0},
    {0,    0,    0,    0,    0,    0,    0,    1},
    {0,    0,    0,    0,    0,    0,    1,    0},
    {0,    0,    0,    0,    0,    0,    1,    1},
    {0,    0,    0,    0,    0,    1,    0,    0},
    {0,    0,    0,    0,    0,    1,    0,    1},
    {0,    0,    0,    0,    0,    1,    1,    0},
    {0,    0,    0,    0,    0,    1,    1,    1},
    {0,    0,    0,    0,    1,    0,    0,    0},
	{0,    0,    0,    0,    1,    0,    0,    1},
	{0,    0,    0,    0,    1,    0,    1,    0},
	{0,    0,    0,    0,    1,    0,    1,    1},
	{0,    0,    0,    0,    1,    1,    0,    0},
	{0,    0,    0,    0,    1,    1,    0,    1},
	{0,    0,    0,    0,    1,    1,    1,    0},
	{0,    0,    0,    0,    1,    1,    1,    1},
	{0,    0,    0,    1,    0,    0,    0,    0},
	{0,    0,    0,    1,    0,    0,    0,    1},
	{0,    0,    0,    1,    0,    0,    1,    0},
	{0,    0,    0,    1,    0,    0,    1,    1},
	{0,    0,    0,    1,    0,    1,    0,    0},
	{0,    0,    0,    1,    0,    1,    0,    1},
	{0,    0,    0,    1,    0,    1,    1,    0},
	{0,    0,    0,    1,    0,    1,    1,    1},
	{0,    0,    0,    1,    1,    0,    0,    0},
	{0,    0,    0,    1,    1,    0,    0,    1},
	{0,    0,    0,    1,    1,    0,    1,    0},
	{0,    0,    0,    1,    1,    0,    1,    1},
	{0,    0,    0,    1,    1,    1,    0,    0},
	{0,    0,    0,    1,    1,    1,    0,    1},
	{0,    0,    0,    1,    1,    1,    1,    0},
	{0,    0,    0,    1,    1,    1,    1,    1},
	{0,    0,    1,    0,    0,    0,    0,    0},
	{0,    0,    1,    0,    0,    0,    0,    1},
	{0,    0,    1,    0,    0,    0,    1,    0},
	{0,    0,    1,    0,    0,    0,    1,    1},
	{0,    0,    1,    0,    0,    1,    0,    0},
	{0,    0,    1,    0,    0,    1,    0,    1},
	{0,    0,    1,    0,    0,    1,    1,    0},
	{0,    0,    1,    0,    0,    1,    1,    1},
	{0,    0,    1,    0,    1,    0,    0,    0},
	{0,    0,    1,    0,    1,    0,    0,    1},
	{0,    0,    1,    0,    1,    0,    1,    0},
	{0,    0,    1,    0,    1,    0,    1,    1},
	{0,    0,    1,    0,    1,    1,    0,    0},
	{0,    0,    1,    0,    1,    1,    0,    1},
	{0,    0,    1,    0,    1,    1,    1,    0},
	{0,    0,    1,    0,    1,    1,    1,    1},
	{0,    0,    1,    1,    0,    0,    0,    0},
	{0,    0,    1,    1,    0,    0,    0,    1},
	{0,    0,    1,    1,    0,    0,    1,    0},
	{0,    0,    1,    1,    0,    0,    1,    1},
	{0,    0,    1,    1,    0,    1,    0,    0},
	{0,    0,    1,    1,    0,    1,    0,    1},
	{0,    0,    1,    1,    0,    1,    1,    0},
	{0,    0,    1,    1,    0,    1,    1,    1},
	{0,    0,    1,    1,    1,    0,    0,    0},
	{0,    0,    1,    1,    1,    0,    0,    1},
	{0,    0,    1,    1,    1,    0,    1,    0},
	{0,    0,    1,    1,    1,    0,    1,    1},
	{0,    0,    1,    1,    1,    1,    0,    0},
	{0,    0,    1,    1,    1,    1,    0,    1},
	{0,    0,    1,    1,    1,    1,    1,    0},
	{0,    0,    1,    1,    1,    1,    1,    1},
	{0,    1,    0,    0,    0,    0,    0,    0},
	{0,    1,    0,    0,    0,    0,    0,    1},
	{0,    1,    0,    0,    0,    0,    1,    0},
	{0,    1,    0,    0,    0,    0,    1,    1},
	{0,    1,    0,    0,    0,    1,    0,    0},
	{0,    1,    0,    0,    0,    1,    0,    1},
	{0,    1,    0,    0,    0,    1,    1,    0},
	{0,    1,    0,    0,    0,    1,    1,    1},
	{0,    1,    0,    0,    1,    0,    0,    0},
	{0,    1,    0,    0,    1,    0,    0,    1},
	{0,    1,    0,    0,    1,    0,    1,    0},
	{0,    1,    0,    0,    1,    0,    1,    1},
	{0,    1,    0,    0,    1,    1,    0,    0},
	{0,    1,    0,    0,    1,    1,    0,    1},
	{0,    1,    0,    0,    1,    1,    1,    0},
	{0,    1,    0,    0,    1,    1,    1,    1},
	{0,    1,    0,    1,    0,    0,    0,    0},
	{0,    1,    0,    1,    0,    0,    0,    1},
	{0,    1,    0,    1,    0,    0,    1,    0},
	{0,    1,    0,    1,    0,    0,    1,    1},
	{0,    1,    0,    1,    0,    1,    0,    0},
	{0,    1,    0,    1,    0,    1,    0,    1},
	{0,    1,    0,    1,    0,    1,    1,    0},
	{0,    1,    0,    1,    0,    1,    1,    1},
	{0,    1,    0,    1,    1,    0,    0,    0},
	{0,    1,    0,    1,    1,    0,    0,    1},
	{0,    1,    0,    1,    1,    0,    1,    0},
	{0,    1,    0,    1,    1,    0,    1,    1},
	{0,    1,    0,    1,    1,    1,    0,    0},
	{0,    1,    0,    1,    1,    1,    0,    1},
	{0,    1,    0,    1,    1,    1,    1,    0},
	{0,    1,    0,    1,    1,    1,    1,    1},
	{0,    1,    1,    0,    0,    0,    0,    0},
	{0,    1,    1,    0,    0,    0,    0,    1},
	{0,    1,    1,    0,    0,    0,    1,    0},
	{0,    1,    1,    0,    0,    0,    1,    1},
	{0,    1,    1,    0,    0,    1,    0,    0},
	{0,    1,    1,    0,    0,    1,    0,    1},
	{0,    1,    1,    0,    0,    1,    1,    0},
	{0,    1,    1,    0,    0,    1,    1,    1},
	{0,    1,    1,    0,    1,    0,    0,    0},
	{0,    1,    1,    0,    1,    0,    0,    1},
	{0,    1,    1,    0,    1,    0,    1,    0},
	{0,    1,    1,    0,    1,    0,    1,    1},
	{0,    1,    1,    0,    1,    1,    0,    0},
	{0,    1,    1,    0,    1,    1,    0,    1},
	{0,    1,    1,    0,    1,    1,    1,    0},
	{0,    1,    1,    0,    1,    1,    1,    1},
	{0,    1,    1,    1,    0,    0,    0,    0},
	{0,    1,    1,    1,    0,    0,    0,    1},
	{0,    1,    1,    1,    0,    0,    1,    0},
	{0,    1,    1,    1,    0,    0,    1,    1},
	{0,    1,    1,    1,    0,    1,    0,    0},
	{0,    1,    1,    1,    0,    1,    0,    1},
	{0,    1,    1,    1,    0,    1,    1,    0},
	{0,    1,    1,    1,    0,    1,    1,    1},
	{0,    1,    1,    1,    1,    0,    0,    0},
	{0,    1,    1,    1,    1,    0,    0,    1},
	{0,    1,    1,    1,    1,    0,    1,    0},
	{0,    1,    1,    1,    1,    0,    1,    1},
	{0,    1,    1,    1,    1,    1,    0,    0},
	{0,    1,    1,    1,    1,    1,    0,    1},
	{0,    1,    1,    1,    1,    1,    1,    0},
	{0,    1,    1,    1,    1,    1,    1,    1},
	{1,    0,    0,    0,    0,    0,    0,    0},
	{1,    0,    0,    0,    0,    0,    0,    1},
	{1,    0,    0,    0,    0,    0,    1,    0},
	{1,    0,    0,    0,    0,    0,    1,    1},
	{1,    0,    0,    0,    0,    1,    0,    0},
	{1,    0,    0,    0,    0,    1,    0,    1},
	{1,    0,    0,    0,    0,    1,    1,    0},
	{1,    0,    0,    0,    0,    1,    1,    1},
	{1,    0,    0,    0,    1,    0,    0,    0},
	{1,    0,    0,    0,    1,    0,    0,    1},
	{1,    0,    0,    0,    1,    0,    1,    0},
	{1,    0,    0,    0,    1,    0,    1,    1},
	{1,    0,    0,    0,    1,    1,    0,    0},
	{1,    0,    0,    0,    1,    1,    0,    1},
	{1,    0,    0,    0,    1,    1,    1,    0},
	{1,    0,    0,    0,    1,    1,    1,    1},
	{1,    0,    0,    1,    0,    0,    0,    0},
	{1,    0,    0,    1,    0,    0,    0,    1},
	{1,    0,    0,    1,    0,    0,    1,    0},
	{1,    0,    0,    1,    0,    0,    1,    1},
	{1,    0,    0,    1,    0,    1,    0,    0},
	{1,    0,    0,    1,    0,    1,    0,    1},
	{1,    0,    0,    1,    0,    1,    1,    0},
	{1,    0,    0,    1,    0,    1,    1,    1},
	{1,    0,    0,    1,    1,    0,    0,    0},
	{1,    0,    0,    1,    1,    0,    0,    1},
	{1,    0,    0,    1,    1,    0,    1,    0},
	{1,    0,    0,    1,    1,    0,    1,    1},
	{1,    0,    0,    1,    1,    1,    0,    0},
	{1,    0,    0,    1,    1,    1,    0,    1},
	{1,    0,    0,    1,    1,    1,    1,    0},
	{1,    0,    0,    1,    1,    1,    1,    1},
	{1,    0,    1,    0,    0,    0,    0,    0},
	{1,    0,    1,    0,    0,    0,    0,    1},
	{1,    0,    1,    0,    0,    0,    1,    0},
	{1,    0,    1,    0,    0,    0,    1,    1},
	{1,    0,    1,    0,    0,    1,    0,    0},
	{1,    0,    1,    0,    0,    1,    0,    1},
	{1,    0,    1,    0,    0,    1,    1,    0},
	{1,    0,    1,    0,    0,    1,    1,    1},
	{1,    0,    1,    0,    1,    0,    0,    0},
	{1,    0,    1,    0,    1,    0,    0,    1},
	{1,    0,    1,    0,    1,    0,    1,    0},
	{1,    0,    1,    0,    1,    0,    1,    1},
	{1,    0,    1,    0,    1,    1,    0,    0},
	{1,    0,    1,    0,    1,    1,    0,    1},
	{1,    0,    1,    0,    1,    1,    1,    0},
	{1,    0,    1,    0,    1,    1,    1,    1},
	{1,    0,    1,    1,    0,    0,    0,    0},
	{1,    0,    1,    1,    0,    0,    0,    1},
	{1,    0,    1,    1,    0,    0,    1,    0},
	{1,    0,    1,    1,    0,    0,    1,    1},
	{1,    0,    1,    1,    0,    1,    0,    0},
	{1,    0,    1,    1,    0,    1,    0,    1},
	{1,    0,    1,    1,    0,    1,    1,    0},
	{1,    0,    1,    1,    0,    1,    1,    1},
	{1,    0,    1,    1,    1,    0,    0,    0},
	{1,    0,    1,    1,    1,    0,    0,    1},
	{1,    0,    1,    1,    1,    0,    1,    0},
	{1,    0,    1,    1,    1,    0,    1,    1},
	{1,    0,    1,    1,    1,    1,    0,    0},
	{1,    0,    1,    1,    1,    1,    0,    1},
	{1,    0,    1,    1,    1,    1,    1,    0},
	{1,    0,    1,    1,    1,    1,    1,    1},
	{1,    1,    0,    0,    0,    0,    0,    0},
	{1,    1,    0,    0,    0,    0,    0,    1},
	{1,    1,    0,    0,    0,    0,    1,    0},
	{1,    1,    0,    0,    0,    0,    1,    1},
	{1,    1,    0,    0,    0,    1,    0,    0},
	{1,    1,    0,    0,    0,    1,    0,    1},
	{1,    1,    0,    0,    0,    1,    1,    0},
	{1,    1,    0,    0,    0,    1,    1,    1},
	{1,    1,    0,    0,    1,    0,    0,    0},
	{1,    1,    0,    0,    1,    0,    0,    1},
	{1,    1,    0,    0,    1,    0,    1,    0},
	{1,    1,    0,    0,    1,    0,    1,    1},
	{1,    1,    0,    0,    1,    1,    0,    0},
	{1,    1,    0,    0,    1,    1,    0,    1},
	{1,    1,    0,    0,    1,    1,    1,    0},
	{1,    1,    0,    0,    1,    1,    1,    1},
	{1,    1,    0,    1,    0,    0,    0,    0},
	{1,    1,    0,    1,    0,    0,    0,    1},
	{1,    1,    0,    1,    0,    0,    1,    0},
	{1,    1,    0,    1,    0,    0,    1,    1},
	{1,    1,    0,    1,    0,    1,    0,    0},
	{1,    1,    0,    1,    0,    1,    0,    1},
	{1,    1,    0,    1,    0,    1,    1,    0},
	{1,    1,    0,    1,    0,    1,    1,    1},
	{1,    1,    0,    1,    1,    0,    0,    0},
	{1,    1,    0,    1,    1,    0,    0,    1},
	{1,    1,    0,    1,    1,    0,    1,    0},
	{1,    1,    0,    1,    1,    0,    1,    1},
	{1,    1,    0,    1,    1,    1,    0,    0},
	{1,    1,    0,    1,    1,    1,    0,    1},
	{1,    1,    0,    1,    1,    1,    1,    0},
	{1,    1,    0,    1,    1,    1,    1,    1},
	{1,    1,    1,    0,    0,    0,    0,    0},
	{1,    1,    1,    0,    0,    0,    0,    1},
	{1,    1,    1,    0,    0,    0,    1,    0},
	{1,    1,    1,    0,    0,    0,    1,    1},
	{1,    1,    1,    0,    0,    1,    0,    0},
	{1,    1,    1,    0,    0,    1,    0,    1},
	{1,    1,    1,    0,    0,    1,    1,    0},
	{1,    1,    1,    0,    0,    1,    1,    1},
	{1,    1,    1,    0,    1,    0,    0,    0},
	{1,    1,    1,    0,    1,    0,    0,    1},
	{1,    1,    1,    0,    1,    0,    1,    0},
	{1,    1,    1,    0,    1,    0,    1,    1},
	{1,    1,    1,    0,    1,    1,    0,    0},
	{1,    1,    1,    0,    1,    1,    0,    1},
	{1,    1,    1,    0,    1,    1,    1,    0},
	{1,    1,    1,    0,    1,    1,    1,    1},
	{1,    1,    1,    1,    0,    0,    0,    0},
	{1,    1,    1,    1,    0,    0,    0,    1},
	{1,    1,    1,    1,    0,    0,    1,    0},
	{1,    1,    1,    1,    0,    0,    1,    1},
	{1,    1,    1,    1,    0,    1,    0,    0},
	{1,    1,    1,    1,    0,    1,    0,    1},
	{1,    1,    1,    1,    0,    1,    1,    0},
	{1,    1,    1,    1,    0,    1,    1,    1},
	{1,    1,    1,    1,    1,    0,    0,    0},
	{1,    1,    1,    1,    1,    0,    0,    1},
	{1,    1,    1,    1,    1,    0,    1,    0},
	{1,    1,    1,    1,    1,    0,    1,    1},
	{1,    1,    1,    1,    1,    1,    0,    0},
	{1,    1,    1,    1,    1,    1,    0,    1},
	{1,    1,    1,    1,    1,    1,    1,    0},
	{1,    1,    1,    1,    1,    1,    1,    1},
};

double EUSCI_A_UARTBAUDRATE_txTbit (uint16_t mode,
    uint16_t i,
    uint16_t s_mod,
    uint16_t f_mod)
{
    switch (mode){
        case 0:
            //Ideal
            return (1 / eusci_a_uartbaudrate_baudrate);
        case 1:
            //Low Frequency Baudrate Generation
            return ((1 / eusci_a_uartbaudrate_brclk) * (eusci_a_uartbaudrate_lf_div_i + eusci_a_uartbaudrate_UCSx_mod[s_mod % 256][i % 8]));
        case 2:
            //Oversampling Baudrate Generation
            return ((1 / eusci_a_uartbaudrate_brclk) *
                  ((16 * eusci_a_uartbaudrate_osr_div_i) + eusci_a_uartbaudrate_UCSx_mod[s_mod % 256][i % 8] + f_mod));
        default:
            //Ideal
            return (1 / eusci_a_uartbaudrate_baudrate);
    }
} //eUARTBAUDRATE_txTbit_ideal

MAX_ERR EUSCI_A_UARTBAUDRATE_txError (uint16_t mode,
    uint16_t s_mod,
    uint16_t f_mod )
{
    uint16_t i = 0;
    double t_ideal = 0;
    double t_usci = 0;
    double bit_error = 0;
    MAX_ERR return_max_error;

    return_max_error.max_error = 0;
    return_max_error.max_negative_error = 0;
    return_max_error.max_positive_error = 0;

    for (i = 0; i < EUSCI_A_UARTBAUDRATE_NO_BITS; i++)
    {
        t_ideal = t_ideal + EUSCI_A_UARTBAUDRATE_txTbit(0, i, 0, 0);
        t_usci = t_usci  + EUSCI_A_UARTBAUDRATE_txTbit(mode, i, s_mod, f_mod);
        bit_error = (t_usci - t_ideal) * eusci_a_uartbaudrate_baudrate * 100;

        if (bit_error < return_max_error.max_negative_error){
            return_max_error.max_negative_error = bit_error;
        }
        if (bit_error > return_max_error.max_positive_error){
            return_max_error.max_positive_error = bit_error;
        }
        if ((bit_error * bit_error) >
            ((return_max_error.max_error) * (return_max_error.max_error))
            ){
            return_max_error.max_error = bit_error;
        }
    } //for i

    return (return_max_error);
} //_eUARTBAUDRATE_txError

MAX_ERR EUSCI_A_UARTBAUDRATE_rxError (uint16_t mode,
    uint16_t s_mod,
    uint16_t f_mod,
    double t_sync )
{
    uint16_t i;
    uint16_t j;
    //uint16_t half_bit_clocks;
    double t_ideal = 0;
    double t_usci = 0;
    double bit_error = 0;
    MAX_ERR return_max_error;

    return_max_error.max_error = 0;
    return_max_error.max_positive_error = 0;
    return_max_error.max_negative_error = 0;

    for (i = 0; i < EUSCI_A_UARTBAUDRATE_NO_BITS; i++)
    {
        t_ideal = (i + 0.5) / eusci_a_uartbaudrate_baudrate;
        t_usci = 0;

        for (j = 0; j < i; j++)
        {
            t_usci = t_usci + EUSCI_A_UARTBAUDRATE_txTbit(mode, j, s_mod, f_mod);
        }
        //Oversampling Baudrate Generation
        if (mode == 2){
            if (f_mod == 15){
                t_usci = t_usci + (1 / eusci_a_uartbaudrate_brclk) * (t_sync +
                                                  (8 * eusci_a_uartbaudrate_osr_div_i)+
                                                  eusci_a_uartbaudrate_UCSx_mod[s_mod % 8][i %8] +
                                                  (uint16_t)(7 + eusci_a_uartbaudrate_UCSx_mod[s_mod % 8][i % 8]));
            } else   {
                t_usci = t_usci +
                         (1 /
                        		 eusci_a_uartbaudrate_brclk) * (t_sync + ( 8 * eusci_a_uartbaudrate_osr_div_i) + eusci_a_uartbaudrate_UCSx_mod[s_mod % 8][i % 8] +
                         (uint16_t)((f_mod + 1) / 2));
            }
        } else   { //Low Frequency Baudrate Generation
            t_usci = t_usci +
                     (1 / eusci_a_uartbaudrate_brclk) * (t_sync + (uint16_t)(eusci_a_uartbaudrate_lf_div_i / 2) +
                                    eusci_a_uartbaudrate_UCSx_mod[s_mod % 8][i % 8]);
        }

        bit_error = (t_usci - t_ideal) * eusci_a_uartbaudrate_baudrate * 100;

        if (bit_error < return_max_error.max_negative_error){
            return_max_error.max_negative_error = bit_error;
        }
        if (bit_error > return_max_error.max_positive_error){
            return_max_error.max_positive_error = bit_error;
        }
        if ((bit_error * bit_error) > ((return_max_error.max_error) *
                                       (return_max_error.max_error))){
            return_max_error.max_error = bit_error;
        }
    } //for i

    return (return_max_error);
} //_eUARTBAUDRATE_rxError

//
//Calculate baud dividers based on target baudrates
//
unsigned short EUSCI_A_UARTBAUDRATE_calculateBaudDividers (double brclk_f,
    double baudrate_f,
    uint16_t *UCAxBRW_value,
    uint16_t *UCAxMCTL_value,
    uint8_t overSampling
    )
{
    unsigned short returnValue = STATUS_SUCCESS;
    uint16_t s_mod, f_mod;
    int opt_s_mod, opt_f_mod;
    MAX_ERR max_error = { 0, 0, 0};
    MAX_ERR abs_max = { 0, 0,0 };

    if ((baudrate_f == 0) && (brclk_f == 0)){
        returnValue = STATUS_FAIL;
        return (returnValue);
    } else   {
    	eusci_a_uartbaudrate_baudrate = baudrate_f;
    	eusci_a_uartbaudrate_brclk = brclk_f;
    }

    eusci_a_uartbaudrate_lf_div_f = (eusci_a_uartbaudrate_brclk) / (eusci_a_uartbaudrate_baudrate);
    eusci_a_uartbaudrate_lf_div_i = (uint16_t)(eusci_a_uartbaudrate_lf_div_f);

    if (eUARTBAUDRATE_LOW_FREQUENCY_BAUDRATE_GENERATION == overSampling){
        opt_s_mod = -1;
        abs_max.max_error = 0;
        abs_max.max_positive_error = 0;
        abs_max.max_negative_error = 0;

        for (s_mod = 0; s_mod < 256; s_mod++)
        {
            max_error = EUSCI_A_UARTBAUDRATE_txError(1, s_mod, 0);

            if (((max_error.max_error * max_error.max_error) <
                 (abs_max.max_error * abs_max.max_error)) ||
                (opt_s_mod == -1)){
                abs_max.max_error = max_error.max_error;
                abs_max.max_positive_error = max_error.max_positive_error;
                abs_max.max_negative_error = max_error.max_negative_error;
                opt_s_mod = s_mod;
            }
        } //for s_mod

        max_error = EUSCI_A_UARTBAUDRATE_txError(1, opt_s_mod, 0);

        //Calculated values
        *UCAxBRW_value = eusci_a_uartbaudrate_lf_div_i;
        *UCAxMCTL_value = (opt_s_mod * 256);
    } else   {
        //OVERSAMPLING
        if (eusci_a_uartbaudrate_lf_div_f >= 16){
        	eusci_a_uartbaudrate_osr_div_f = (eusci_a_uartbaudrate_brclk) / (eusci_a_uartbaudrate_baudrate) / 16;
        	eusci_a_uartbaudrate_osr_div_i = (uint16_t)(eusci_a_uartbaudrate_osr_div_f);

            //not defined STD_CALC
            opt_s_mod = -1;
            abs_max.max_error = 0;
            abs_max.max_positive_error = 0;
            abs_max.max_negative_error = 0;

            for (f_mod = 0; f_mod < (eusci_a_uartbaudrate_osr_div_i <= 1 ? 1 : 16); f_mod++)
            //Loop length depends on "EUSCI_osr_div_i" because FX modulation does not
            //work with a BR divider <=1 because the prescaler is bypassed in
            //this case.
            //EUSCI_osr_div_i<=1 => only f_mod = 0 is valid
            //EUSCI_osr_div_i >1 => f_mod can vary between 0 and 15.
            {
                for (s_mod = 0; s_mod < 256; s_mod++)
                {
                    max_error = EUSCI_A_UARTBAUDRATE_txError(2, s_mod, f_mod);

                    
                      
                    if (((max_error.max_error * max_error.max_error) <
                         (abs_max.max_error * abs_max.max_error)) ||
                        (opt_s_mod == -1)){
                        abs_max.max_error = max_error.max_error;
                        abs_max.max_positive_error =
                            max_error.max_positive_error;
                        abs_max.max_negative_error =
                            max_error.max_negative_error;
                        opt_s_mod = s_mod;
                        opt_f_mod = f_mod;
                    }
                } //for s_mod
            } //for f_mod


            if (eusci_a_uartbaudrate_osr_div_i == 1){
            	eusci_a_uartbaudrate_osr_div_i = 2;
                max_error = EUSCI_A_UARTBAUDRATE_txError(2, 0, 0);

                if ((max_error.max_error * max_error.max_error) <
                    (abs_max.max_error * abs_max.max_error)){
                    abs_max.max_error = max_error.max_error;
                    abs_max.max_positive_error = max_error.max_positive_error;
                    abs_max.max_negative_error = max_error.max_negative_error;
                    opt_s_mod = 0;
                    opt_f_mod = 0;
                } else   {
                	eusci_a_uartbaudrate_osr_div_i = 1;
                }
            }

            max_error = EUSCI_A_UARTBAUDRATE_txError(2, opt_s_mod, opt_f_mod);

            if ((max_error.max_error * max_error.max_error) > (50 * 50)){
                returnValue = STATUS_FAIL;
            } else   {
                *UCAxBRW_value = eusci_a_uartbaudrate_osr_div_i;
                *UCAxMCTL_value = (opt_f_mod * 16) + (opt_s_mod * 256) + UCOS16;

            } //else
        } //if
        else {
            returnValue = STATUS_FAIL;
        } //else
    }
    return (returnValue);
} //EUSCI_A_UARTBAUDRATE_calculateBaudDividers
