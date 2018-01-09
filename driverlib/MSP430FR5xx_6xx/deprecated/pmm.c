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
// pmm.c - Driver for the pmm Module.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup pmm_api
//! @{
//
//*****************************************************************************

#include "inc/hw_regaccess.h"
#include "inc/hw_memmap.h"

#ifdef DRIVERLIB_LEGACY_MODE

#ifdef __MSP430_HAS_PMM_FRAM__
#include "pmm.h"

#include <assert.h>

//*****************************************************************************
//
//! \brief Enables the low power reset. SVSH does not reset device, but
//! triggers a system NMI
//!
//! \param baseAddress is the base address of the PMM module.
//!
//! Modified bits of \b PMMCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void PMM_enableLowPowerReset (uint32_t baseAddress)
{
    HWREG8(baseAddress + OFS_PMMCTL0_H) = PMMPW_H;
    HWREG8(baseAddress + OFS_PMMCTL0) |= PMMLPRST;
    HWREG8(baseAddress + OFS_PMMCTL0_H) = 0x00;
}

//*****************************************************************************
//
//! \brief Disables the low power reset. SVSH resets device.
//!
//! \param baseAddress is the base address of the PMM module.
//!
//! Modified bits of \b PMMCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void PMM_disableLowPowerReset (uint32_t baseAddress)
{
    HWREG8(baseAddress + OFS_PMMCTL0_H) = PMMPW_H;
    HWREG8(baseAddress + OFS_PMMCTL0) &= ~PMMLPRST;
    HWREG8(baseAddress + OFS_PMMCTL0_H) = 0x00;
}

//*****************************************************************************
//
//! \brief Enables the high-side SVS circuitry
//!
//! \param baseAddress is the base address of the PMM module.
//!
//! Modified bits of \b PMMCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void PMM_enableSVSH (uint32_t baseAddress)
{
    HWREG8(baseAddress + OFS_PMMCTL0_H) = PMMPW_H;
    HWREG8(baseAddress + OFS_PMMCTL0_L) |= SVSHE;
    HWREG8(baseAddress + OFS_PMMCTL0_H) = 0x00;
}

//*****************************************************************************
//
//! \brief Disables the high-side SVS circuitry
//!
//! \param baseAddress is the base address of the PMM module.
//!
//! Modified bits of \b PMMCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void PMM_disableSVSH (uint32_t baseAddress)
{
    HWREG8(baseAddress + OFS_PMMCTL0_H) = PMMPW_H;
    HWREG8(baseAddress + OFS_PMMCTL0_L) &= ~SVSHE;
    HWREG8(baseAddress + OFS_PMMCTL0_H) = 0x00;
}

//*****************************************************************************
//
//! \brief Makes the low-dropout voltage regulator (LDO) remain ON when going
//! into LPM 3/4.
//!
//! \param baseAddress is the base address of the PMM module.
//!
//! Modified bits of \b PMMCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void PMM_regOn (uint32_t baseAddress)
{
    HWREG8(baseAddress + OFS_PMMCTL0_H) = PMMPW_H;
    HWREG8(baseAddress + OFS_PMMCTL0) &= ~PMMREGOFF;
    HWREG8(baseAddress + OFS_PMMCTL0_H) = 0x00;
}

//*****************************************************************************
//
//! \brief Turns OFF the low-dropout voltage regulator (LDO) when going into
//! LPM3/4, thus the system will enter LPM3.5 or LPM4.5 respectively
//!
//! \param baseAddress is the base address of the PMM module.
//!
//! Modified bits of \b PMMCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void PMM_regOff (uint32_t baseAddress)
{
    HWREG8(baseAddress + OFS_PMMCTL0_H) = PMMPW_H;
    HWREG8(baseAddress + OFS_PMMCTL0) |= PMMREGOFF;
    HWREG8(baseAddress + OFS_PMMCTL0_H) = 0x00;
}

//*****************************************************************************
//
//! \brief Calling this function will trigger a software Power On Reset (POR).
//!
//! \param baseAddress is the base address of the PMM module.
//!
//! Modified bits of \b PMMCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void PMM_trigPOR (uint32_t baseAddress)
{
    HWREG8(baseAddress + OFS_PMMCTL0_H) = PMMPW_H;
    HWREG8(baseAddress + OFS_PMMCTL0) |= PMMSWPOR;
    HWREG8(baseAddress + OFS_PMMCTL0_H) = 0x00;
}

//*****************************************************************************
//
//! \brief Calling this function will trigger a software Brown Out Rest (BOR).
//!
//! \param baseAddress is the base address of the PMM module.
//!
//! Modified bits of \b PMMCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void PMM_trigBOR (uint32_t baseAddress)
{
    HWREG8(baseAddress + OFS_PMMCTL0_H) = PMMPW_H;
    HWREG8(baseAddress + OFS_PMMCTL0) |= PMMSWBOR;
    HWREG8(baseAddress + OFS_PMMCTL0_H) = 0x00;
}

//*****************************************************************************
//
//! \brief Clears interrupt flags for the PMM
//!
//! \param baseAddress is the base address of the PMM module.
//! \param mask is the mask for specifying the required flag
//!        Mask value is the logical OR of any of the following:
//!        - \b PMM_PMMBORIFG - Software BOR interrupt
//!        - \b PMM_PMMRSTIFG - RESET pin interrupt
//!        - \b PMM_PMMPORIFG - Software POR interrupt
//!        - \b PMM_SVSHIFG - SVS high side interrupt
//!        - \b PMM_SVSLIFG - SVS low side interrupt
//!        - \b PMM_PMMLPM5IFG - LPM5 indication
//!        - \b PMM_ALL - All interrupts
//!
//! Modified bits of \b PMMCTL0 register and bits of \b PMMIFG register.
//!
//! \return None
//
//*****************************************************************************
void PMM_clearInterrupt (uint32_t baseAddress,
		uint16_t mask)
{
    HWREG8(baseAddress + OFS_PMMCTL0_H) = PMMPW_H;
    HWREG16(baseAddress + OFS_PMMIFG) &= ~mask;
    HWREG8(baseAddress + OFS_PMMCTL0_H) = 0x00;
}

//*****************************************************************************
//
//! \brief Returns interrupt status
//!
//! \param baseAddress is the base address of the PMM module.
//! \param mask is the mask for specifying the required flag
//!        Mask value is the logical OR of any of the following:
//!        - \b PMM_PMMBORIFG - Software BOR interrupt
//!        - \b PMM_PMMRSTIFG - RESET pin interrupt
//!        - \b PMM_PMMPORIFG - Software POR interrupt
//!        - \b PMM_SVSHIFG - SVS high side interrupt
//!        - \b PMM_SVSLIFG - SVS low side interrupt
//!        - \b PMM_PMMLPM5IFG - LPM5 indication
//!        - \b PMM_ALL - All interrupts
//!
//! \return STATUS_SUCCESS (0x01) or STATUS_FAIL (0x00)
//
//*****************************************************************************
uint16_t PMM_getInterruptStatus (uint32_t baseAddress,
    uint16_t mask)
{
    return ( (HWREG16(baseAddress + OFS_PMMIFG)) & mask );
}

//*****************************************************************************
//
//! \brief Lock LPM5
//!
//! Lock I/O pin and other LPMx.5 relevant (e.g. RTC) configurations upon
//! entry/exit to/from LPMx.5. Once power is applied to the device, this bit,
//! once set, can only be cleared by the user or via another power cycle.
//! LPMx.5 configuration remains locked. Pin state is held during LPMx.5 entry
//! and exit. Disable the GPIO power-on default high-impedance mode to activate
//! previously configured port settings.
//!
//! \param baseAddress is the base address of the PMM module.
//!
//! \return None
//
//*****************************************************************************
void PMM_lockLPM5 (uint32_t baseAddress)
{
	HWREG8(baseAddress + OFS_PM5CTL0) |= LOCKLPM5;
}

//*****************************************************************************
//
//! \brief Unlock LPM5
//!
//! LPMx.5 configuration is not locked and defaults to its reset condition.
//! Disable the GPIO power-on default high-impedance mode to activate
//! previously configured port settings.
//!
//! \param baseAddress is the base address of the PMM module.
//!
//! \return None
//
//*****************************************************************************
void PMM_unlockLPM5 (uint32_t baseAddress)
{
	HWREG8(baseAddress + OFS_PM5CTL0) &= ~LOCKLPM5;
}

#endif
#endif
//*****************************************************************************
//
//! Close the doxygen group for pmm_api
//! @}
//
//*****************************************************************************
