/* --COPYRIGHT--,BSD
 * Copyright (c) 2013, Texas Instruments Incorporated
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
// pmm.h - Driver for the PMM Module.
//
//*****************************************************************************

#ifndef __MSP430WARE_PMM_H__
#define __MSP430WARE_PMM_H__

#include "inc/hw_memmap.h"

#ifdef __MSP430_HAS_PMM_FRAM__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// The following are values that can be passed to the mask parameter for
// functions: PMM_clearInterrupt(), and PMM_getInterruptStatus().
//
//*****************************************************************************
#define PMM_PMMBORIFG                                                 PMMBORIFG
#define PMM_PMMRSTIFG                                                 PMMRSTIFG
#define PMM_PMMPORIFG                                                 PMMPORIFG
#define PMM_SVSHIFG                                                     SVSHIFG
#define PMM_SVSLIFG                                                     SVSLIFG
#define PMM_PMMLPM5IFG                                               PMMLPM5IFG
#define PMM_ALL                                                          (0xB7)

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern void PMM_enableLowPowerReset(uint32_t baseAddress);

extern void PMM_disableLowPowerReset(uint32_t baseAddress);

extern void PMM_enableSVSH(uint32_t baseAddress);

extern void PMM_disableSVSH(uint32_t baseAddress);

extern void PMM_regOn(uint32_t baseAddress);

extern void PMM_regOff(uint32_t baseAddress);

extern void PMM_trigPOR(uint32_t baseAddress);

extern void PMM_trigBOR(uint32_t baseAddress);

extern void PMM_clearInterrupt(uint32_t baseAddress,
                               uint16_t mask);

extern uint16_t PMM_getInterruptStatus(uint32_t baseAddress,
                                       uint16_t mask);

extern void PMM_lockLPM5(uint32_t baseAddress);

extern void PMM_unlockLPM5(uint32_t baseAddress);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif
#endif // __MSP430WARE_PMM_H__
