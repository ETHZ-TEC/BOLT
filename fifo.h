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
 * fifo.h
 * FIFO queues, v1.2
 * management of two equally sized FIFO queues
 *
 * Assumptions:
 * - the queues are FIFO queues with no priority
 * - each queue can hold a (compile-time) fixed max. number of messages
 * - each message in both queues has a (compile-time) fixed maximum length, which must be a multiple of 2
 * - the minimum message length is 2 bytes (transmitting shorter messages might lead to a starvation problem due to port interrupt priorities)
 * - DMA performs all data transfers from the SPI to the queue memory and vice-versa
 * - all queue operations must be executed in an ISR to guarantee atomicity, interrupt nesting must be disabled
 * - valid operations: add (increase write pointer), remove (increase read pointer), is_full, is_empty
 */

#ifndef FIFO_H
#define FIFO_H


#define QUEUE_ISEMPTY(inst)       ( queueIsEmpty((FSMINST_APPL_PROC == inst) ? &queueCtoACtrl : &queueAtoCCtrl) )
#define QUEUE_ISFULL(inst)        ( queueIsFull((FSMINST_APPL_PROC == inst) ? &queueAtoCCtrl : &queueCtoACtrl) )
#define QUEUE_ADD(inst, msgSize)  ( queueAdd((FSMINST_APPL_PROC == inst) ? &queueAtoCCtrl : &queueCtoACtrl, (msgSize)) )
#define QUEUE_REMOVE(inst)        ( queueRemove((FSMINST_APPL_PROC == inst) ? &queueCtoACtrl : &queueAtoCCtrl) )

#define MESSAGE_SIZE_WITH_METADATA  (MESSAGE_SIZE + 2)  // max. size of 1 message including the length of the message (additonal 2 bytes)


/* typedefs */

typedef struct FIFOMSG
{
  uint8_t   data[MESSAGE_SIZE];    // the message itself (payload)
  uint16_t  size;          // message size in bytes (meta)
} FIFOMessage;

typedef struct FIFO
{
  volatile FIFOMessage* nextRead;   // pointer to the oldest message in the queue
  volatile FIFOMessage* nextWrite;  // pointer to the next write position
  volatile FIFOMessage* oldValue;   // the previous pointer value
  FIFOMessage* first;         // first message in queue, constant (pointer needed for a faster empty/full test)
  FIFOMessage* last;          // last message in queue, constant (pointer needed for a faster empty/full test)
  volatile uint8_t criticalRead;
  volatile uint8_t criticalWrite;
} FIFOQueue;


/* external (global) variables */

extern FIFOQueue queueAtoCCtrl;
extern FIFOQueue queueCtoACtrl;


/* inline functions */

// returns 1 if the queue q is empty and 0 otherwise
#pragma FUNC_ALWAYS_INLINE(queueIsEmpty)        // force inlining
static __inline uint8_t queueIsEmpty(const FIFOQueue* const q)
{
  return q->nextWrite == q->nextRead;
}

// returns 1 if the queue q is full and 0 otherwise
#pragma FUNC_ALWAYS_INLINE(queueIsFull)
static __inline uint8_t queueIsFull(const FIFOQueue* const q)
{
  // two cases because the array is used as a circular buffer:
  // - the usual case where the address of the write pointer is bigger than the address of the read pointer
  // - and a special case where the write pointer is smaller than the read pointer, i.e. the write pointer points to the last element in the queue and the read pointer to the first
  if (q->nextWrite < q->nextRead)
  {
    NOP2; // balance if/else
    return ( ((uint16_t)q->nextWrite + MESSAGE_SIZE_WITH_METADATA) == (uint16_t)q->nextRead );
  } else
  {
    // note: the following statements could be simplified, but execution time balancing requires to write it this way
    if (q->nextRead != q->first)
    {
      NOP5;
      return 0;
    } else if (q->nextWrite != q->last)
    {
      return 0;
    }
    return 1;
  }
}

// increases the write pointer of the queue q and stores the size of the message (note: does NOT check whether the queue is full or msg size is 0!)
#pragma FUNC_ALWAYS_INLINE(queueAdd)
static __inline void queueAdd(FIFOQueue* const q, const uint16_t size)
{
  // this code section must execute without interruption (e.g. due to a power failure), otherwise it could corrupt the content of the memory
  q->oldValue = q->nextWrite;
  q->criticalWrite = 1;
  q->nextWrite->size = size;   // store the size
  if (q->nextWrite == q->last)
  {
    NOP3;
    q->nextWrite = q->first;
  } else
  {
    q->nextWrite = (FIFOMessage*)((uint16_t)q->nextWrite + MESSAGE_SIZE_WITH_METADATA);  // move the write pointer to the next queue entry (if at end of queue, set next pointer to the first entry)
  }
  q->criticalWrite = 0;
}

// increases the read pointer of the queue q (note: does NOT check whether the queue is empty!)
#pragma FUNC_ALWAYS_INLINE(queueRemove)
static __inline void queueRemove(FIFOQueue* const q)
{
  q->oldValue = q->nextRead;
  q->criticalRead = 1;
  if (q->nextRead == q->last)
  {
    NOP3;
    q->nextRead = q->first;
  } else
  {
    q->nextRead = (FIFOMessage*)((uint16_t)q->nextRead + MESSAGE_SIZE_WITH_METADATA);
  }
  q->criticalRead = 0;
}


/* prototypes */

void initQueues();
void printQueues();


#endif // FIFO_H
