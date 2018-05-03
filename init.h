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
 * init.h
 * all initialization
 */

#ifndef INIT_H
#define INIT_H


/* typedefs */

typedef enum
{
  OPMODE_READ = 0,
  OPMODE_WRITE = 1,
  NUM_OF_OP_MODES
} OPERATINGMODE;

typedef enum
{
  MCLK_SPEED_1MHZ = 0,
  MCLK_SPEED_8MHZ,
  MCLK_SPEED_16MHZ,
  NUM_OF_MCLK_SPEEDS
} MCLKSPEED;

typedef enum
{
  UART_RATE_9600 = 0,
  UART_RATE_115200,
  NUM_OF_UART_RATES
} UARTRATE;

typedef enum
{
  SPI_CKPL_0 = 0,       // = EUSCI_A_SPI_CLOCKPOLARITY_INACTIVITY_LOW (0 = no signal)
  SPI_CKPL_1,           // = EUSCI_A_SPI_CLOCKPOLARITY_INACTIVITY_HIGH (1 = no signal)
  NUM_OF_SPI_CKPL
} SPICKPL;              // clock polarity

typedef enum
{
  SPI_CKPH_0 = 0,
  SPI_CKPH_1,
  NUM_OF_SPI_CKPH
} SPICKPH;              // clock phase

typedef enum
{
  EDGE_RISING = 0,      // = GPIO_LOW_TO_HIGH_TRANSITION
  EDGE_FALLING = 1,     // = GPIO_HIGH_TO_LOW_TRANSITION
  NUM_OF_SIGNAL_EDGES
} SIGNALEDGE;



/* function prototypes */

void initMSP430();


#endif // INIT_H
