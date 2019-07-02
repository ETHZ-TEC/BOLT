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
 * main.c
 * main file
 *
 * Status LED:
 * - blinks once at startup
 * - stays on if an error is detected
 *
 * Notes:
 * - By default, the small memory model is enabled, i.e. only the lower 64k of
 *   the memory are used. To use all available memory (at the cost of some
 *   computational overhead and larger pointers), the large data memory model
 *   can be enabled in the compiler options (advanced/runtime tab). Make sure
 *   all the memory addresses are 32-bit (replace pointer conversions).
 * - recommended compiler optimization: 4 (whole program), speed
 */

#include "main.h"

#pragma DATA_SECTION(systemStats, ".sysmem")
SYSSTATS systemStats = { 0 };


// validates and restores the system state
void validateSysState(void)
{
  if (STATE_IDLE != currentState[FSMINST_APPL_PROC] || STATE_IDLE != currentState[FSMINST_COMM_PROC])
  {
    uint8_t rolledBack = 0;
    LOG_ERROR(concatStrings((int8_t*)"ERROR: System failure detected, startup from invalid state (A: %, C: %)", stateToString[currentState[FSMINST_APPL_PROC]], stateToString[currentState[FSMINST_COMM_PROC]], debugBuffer, DEBUG_BUFFER_SIZE));
    systemStats.crashCount++;
    // roll-back if necessary
    if (queueAtoCCtrl.criticalWrite)
    {
      // the crash occurred during the update procedure of the data structure
      rolledBack = 1;
      queueAtoCCtrl.nextWrite = queueAtoCCtrl.oldValue;
      queueAtoCCtrl.criticalWrite = 0;

    } else if (queueAtoCCtrl.criticalRead)    // note: criticalWrite and criticalRead can never be 1 at the same time
    {
      rolledBack = 1;
      queueAtoCCtrl.nextRead = queueAtoCCtrl.oldValue;
      queueAtoCCtrl.criticalRead = 0;
    }
    if (queueCtoACtrl.criticalWrite)
    {
      // the crash occurred during the update procedure of the data structure
      rolledBack = 1;
      queueCtoACtrl.nextWrite = queueCtoACtrl.oldValue;
      queueCtoACtrl.criticalWrite = 0;

    } else if (queueCtoACtrl.criticalRead)    // note: criticalWrite and criticalRead can never be 1 at the same time
    {
      rolledBack = 1;
      queueCtoACtrl.nextRead = queueCtoACtrl.oldValue;
      queueCtoACtrl.criticalRead = 0;
    }
    if (rolledBack)
    {
      LOG_ERROR((int8_t*)"Queue pointer recovered");
    }
    currentState[FSMINST_APPL_PROC] = STATE_IDLE;
    currentState[FSMINST_COMM_PROC] = STATE_IDLE;
  }
  if (!QUEUE_ISEMPTY(FSMINST_COMM_PROC))
  {
    SPI_IND_HIGH(FSMINST_COMM_PROC);    // set IND line (data to read for comm. proc.)
    LOG_INFO("Queue A to C is not empty (IND = 1)");
  }
  if (!QUEUE_ISEMPTY(FSMINST_APPL_PROC))
  {
    SPI_IND_HIGH(FSMINST_APPL_PROC);    // set IND line (data to read for appl. proc.)
    LOG_INFO("Queue C to A is not empty (IND = 1)");
  }
}


// determines and keeps track of the reset source
void collectStats(void)
{
  uint32_t rstFlag = SYSRSTIV;  // flag is automatically cleared by reading it, therefore store its value

  logStats();

  // when the PMM causes a reset, a value is generated in the system reset interrupt vector generator register (SYSRSTIV), corresponding to the source of the reset
  switch (rstFlag)
  {
  // brownout reset (BOR)
  case SYSRSTIV_BOR:
    printLine((int8_t*)"Reset source: BOR");
    systemStats.powerLossCount++;
    break;
  case SYSRSTIV_RSTNMI:
    printLine((int8_t*)"Reset source: Reset Pin");
    break;
  case SYSRSTIV_DOBOR:
    printLine((int8_t*)"Reset source: Software BOR");
    break;
  case SYSRSTIV_LPM5WU:
    printLine((int8_t*)"Reset source: Wake-up from LPMx.5");
    break;
  case SYSRSTIV_SECYV:
    printLine((int8_t*)"Reset source: Security violation");
    break;
  case SYSRSTIV_SVSHIFG:
    printLine((int8_t*)"Reset source: SVS");
    break;
  case SYSRSTIV_DOPOR:
    printLine((int8_t*)"Reset source: Software POR");
    break;
  case SYSRSTIV_WDTTO:
    printLine((int8_t*)"Reset source: Watchdog timeout");
    break;
  case SYSRSTIV_WDTKEY:
    printLine((int8_t*)"Reset source: Watchdog password violation");
    break;
  case SYSRSTIV_FRCTLPW:
    printLine((int8_t*)"Reset source: FRAM password violation");
    break;
  case SYSRSTIV_UBDIFG:
    printLine((int8_t*)"Reset source: Uncorrectable FRAM bit error");
    break;
  case SYSRSTIV_PERF:
    printLine((int8_t*)"Reset source: Peripheral area fetch");
    break;
  case SYSRSTIV_PMMPW:
    printLine((int8_t*)"Reset source: PMM password violation");
    break;
  case SYSRSTIV_MPUPW:
    printLine((int8_t*)"Reset source: MPU password violation");
    break;
  case SYSRSTIV_CSPW:
    printLine((int8_t*)"Reset source: CS password violation");
    break;
  case SYSRSTIV_MPUSEGPIFG:
    printLine((int8_t*)"Reset source: EncIP memory segment violation");
    break;
  case SYSRSTIV_MPUSEGIIFG:
    printLine((int8_t*)"Reset source: Information memory segment violation");
    break;
  case SYSRSTIV_MPUSEG1IFG:
    printLine((int8_t*)"Reset source: Segment 1 memory violation");
    break;
  case SYSRSTIV_MPUSEG2IFG:
    printLine((int8_t*)"Reset source: Segment 2 memory violation");
    break;
  case SYSRSTIV_MPUSEG3IFG:
    printLine((int8_t*)"Reset source: Segment 3 memory violation");
    break;
  default:
    printLine((int8_t*)"Reset source: Unknown");
    break;
  }
  systemStats.powerOnCount++;
}


int main(void)
{
  // --- INITIALIZATION ---

  initMSP430();
  collectStats();
  validateSysState();  // restore the system state (init state machine)
  initQueues();      // initialize the required data structures

  // blink once
  GPIO_setOutputHighOnPinInline(LED_STATUS_PORT, LED_STATUS_PIN);
  WAIT(20);
  GPIO_setOutputLowOnPinInline(LED_STATUS_PORT, LED_STATUS_PIN);

#ifndef DEBUG
  UART_DISABLE;
#endif
  ENABLE_INTERRUPTS;

  // --- READY, program execution starts here ---

  ENTER_LPM4;   // enter low-power mode 4

  while (1);
}
