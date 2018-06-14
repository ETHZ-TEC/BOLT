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
 * main.h
 * all includes and compiler definitions are in here
 */

#ifndef MAIN_H
#define MAIN_H


/* project files (includes) */

#include <stdlib.h>     // standard types
#include "driverlib.h"    // official MSP430 peripheral driver library
#include "config.h"     // all high-level configuration
#include "version.h"
#include "hal.h"      // hardware abstraction layer
#include "utils.h"
#include "init.h"
#include "msp430driver.h"
#include "dma.h"
#include "fifo.h"
#include "state.h"


/* typedefs */

// 11x 4 bytes
typedef struct
{
  uint32_t powerOnCount;
  uint32_t powerLossCount;
  uint32_t crashCount;    // startup in invalid system state, e.g. power loss during write/read operation
  uint32_t invStateCount; // invalid state transition
  uint32_t abortCountA;
  uint32_t abortCountC;
  uint32_t writeCountA;
  uint32_t writeCountC;
  uint32_t readCountA;
  uint32_t readCountC;
  uint32_t fullCountA;
  uint32_t fullCountC;
  uint32_t writeByteCountA;
  uint32_t writeByteCountC;
} SYSSTATS;


extern SYSSTATS systemStats;


#endif // MAIN_H
