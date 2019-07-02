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
 * fifo.c
 * FIFO queues
 *
 * Assumptions:
 * - the queues are FIFO queues with no priority
 * - each queue can hold a (compile-time) fixed max. number of messages
 * - each message in both queues has a (compile-time) fixed maximum length, which must be a multiple of 2
 * - the minimum message length is 2 bytes (transmitting shorter messages might lead to a starvation problem due to port interrupt priorities)
 * - DMA performs all data transfers from the SPI to the queue memory and vice-versa
 * - all queue operations must be executed in an ISR to guarantee atomicity, interrupt nesting must be disabled
 * - operations: add (increase write pointer), remove (increase read pointer), is_full, is_empty
 *
 */

#include "main.h"

// put the following variables into a predefined FRAM section
#pragma DATA_SECTION(queueAtoCCtrl, ".sysmem")
FIFOQueue queueAtoCCtrl = { 0 };                // control structure for queue from application to communication processor
#pragma DATA_SECTION(queueCtoACtrl, ".sysmem")
FIFOQueue queueCtoACtrl = { 0 };                // control structure for queue from communication to application processor
#pragma DATA_SECTION(queueAtoCData, ".boltqueues")
FIFOMessage queueAtoCData[MAX_NUM_OF_MSG_A_TO_C];     // the actual queue data (including the size of the messages)
#pragma DATA_SECTION(queueCtoAData, ".boltqueues")
FIFOMessage queueCtoAData[MAX_NUM_OF_MSG_C_TO_A];


// overwrite the whole queue memory with a custom value
void fillQueueMemory(const uint16_t value)
{
  fillRAM(value, (uint16_t*)queueAtoCData, (MAX_NUM_OF_MSG_A_TO_C * MESSAGE_SIZE_WITH_METADATA) >> 1);
  fillRAM(value, (uint16_t*)queueCtoAData, (MAX_NUM_OF_MSG_C_TO_A * MESSAGE_SIZE_WITH_METADATA) >> 1);
}


// resets the queue pointers
void clearQueues()
{
  queueAtoCCtrl.nextWrite = queueAtoCData;
  queueAtoCCtrl.nextRead  = queueAtoCData;
  queueAtoCCtrl.first     = queueAtoCData;
  queueAtoCCtrl.last      = &queueAtoCData[MAX_NUM_OF_MSG_A_TO_C - 1];
  queueCtoACtrl.nextWrite = queueCtoAData;
  queueCtoACtrl.nextRead  = queueCtoAData;
  queueCtoACtrl.first     = queueCtoAData;
  queueCtoACtrl.last      = &queueCtoAData[MAX_NUM_OF_MSG_C_TO_A - 1];
#ifdef DEBUG
  fillQueueMemory(0x1234);
#endif // DEBUG
  printLine(composeString("FIFO queues cleared (address space: %h - %h)", (uint32_t)queueAtoCCtrl.first, (uint32_t)queueCtoACtrl.last, debugBuffer));
  SPI_IND_LOW(FSMINST_COMM_PROC);
  SPI_IND_LOW(FSMINST_APPL_PROC);
}


// this function ensures that the queue pointers are valid (within the queue memory boundaries) and must be called prior to using the queues
void initQueues()
{
  // make sure the queue is valid when the MCU starts up for the first time after programming
  if (RESET_QUEUES_AT_STARTUP ||
      queueAtoCCtrl.first != queueAtoCData ||
      queueAtoCCtrl.last  != &queueAtoCData[MAX_NUM_OF_MSG_A_TO_C - 1] ||
      queueCtoACtrl.first != queueCtoAData ||
      queueCtoACtrl.last  != &queueCtoAData[MAX_NUM_OF_MSG_C_TO_A - 1])
  {
    clearQueues();

  // check if read and write pointers are within the FIFO memory range and are a multiple of MESSAGE_SIZE_WITH_METADATA
  } else if (queueAtoCCtrl.nextRead > &queueAtoCData[MAX_NUM_OF_MSG_A_TO_C - 1]  ||
             queueAtoCCtrl.nextRead < queueAtoCData  ||
             (((uint16_t)queueAtoCCtrl.nextRead - (uint16_t)queueAtoCData) % MESSAGE_SIZE_WITH_METADATA) != 0  ||
             queueAtoCCtrl.nextWrite > &queueAtoCData[MAX_NUM_OF_MSG_A_TO_C - 1] ||
             queueAtoCCtrl.nextWrite < queueAtoCData ||
             (((uint16_t)queueAtoCCtrl.nextWrite - (uint16_t)queueAtoCData) % MESSAGE_SIZE_WITH_METADATA) != 0 ||
             queueCtoACtrl.nextRead > &queueCtoAData[MAX_NUM_OF_MSG_C_TO_A - 1]  ||
             queueCtoACtrl.nextRead < queueCtoAData  ||
             (((uint16_t)queueCtoACtrl.nextRead - (uint16_t)queueCtoAData) % MESSAGE_SIZE_WITH_METADATA) != 0  ||
             queueCtoACtrl.nextWrite > &queueCtoAData[MAX_NUM_OF_MSG_C_TO_A - 1] ||
             queueCtoACtrl.nextWrite < queueCtoAData ||
             (((uint16_t)queueCtoACtrl.nextWrite - (uint16_t)queueCtoAData) % MESSAGE_SIZE_WITH_METADATA) != 0)
  {
    LOG_ERROR("ERROR: Queues corrupted (invalid read / write pointers)");

    clearQueues();
  }
  LOG_INFO_2("FIFO queues initialized (%u msg/queue, %u B/msg)", MAX_NUM_OF_MSG_A_TO_C, MESSAGE_SIZE);
}


// print out the whole FIFO memory (hex format, one line per 16-bit memory word) over the UART interface with polling
void printQueues()
{
  uint16_t* start = (uint16_t*)queueAtoCCtrl.first;

  LOG_INFO_2("\nCurrent FIFO AC r/w pointers: %h, %h", (uint16_t)queueAtoCCtrl.nextRead, (uint16_t)queueAtoCCtrl.nextWrite);
  LOG_INFO_2("Current FIFO CA r/w pointers: %h, %h", (uint16_t)queueCtoACtrl.nextRead, (uint16_t)queueCtoACtrl.nextWrite);

  LOG_INFO("--- FIFO A to C ---");
  while (start < (uint16_t*)queueAtoCCtrl.last)
  {
    LOG_INFO_2("%h: %h", (uint16_t)start, *start);
    start++;
  }
  LOG_INFO("\n--- FIFO C to A ---");
  start = (uint16_t*)queueCtoACtrl.first;
  while (start < (uint16_t*)queueCtoACtrl.last)
  {
    LOG_INFO_2("%h: %h", (uint16_t)start, *start);
    start++;
  }
}
