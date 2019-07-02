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
 * state.c
 * State machine
 */

#include "main.h"


FSMSTATE processInvalid(const FSMINSTANCE inst);
FSMSTATE processREQHigh(const FSMINSTANCE inst);
FSMSTATE processREQLowRead(const FSMINSTANCE inst);
FSMSTATE processREQLowWrite(const FSMINSTANCE inst);
FSMSTATE processTCRead(const FSMINSTANCE inst);
FSMSTATE processTCWrite(const FSMINSTANCE inst);

#pragma DATA_SECTION(currentState, ".sysmem")
volatile FSMSTATE currentState[NUM_OF_FSM_INSTANCES];

// mapping of state and events to action functions (state transitions)
const STATETRANSITION stateMachine[NUM_OF_EVENTS][NUM_OF_STATES] =
{
  // States:
  // Idle (STATE_IDLE)                 Read (STATE_READ)                    Write (STATE_WRITE)                      // Events:
  { (STATETRANSITION) processREQHigh,  (STATETRANSITION) processInvalid,    (STATETRANSITION) processInvalid     },  // Request pin high (EVENT_REQ_HIGH)
  { (STATETRANSITION) processInvalid,  (STATETRANSITION) processREQLowRead, (STATETRANSITION) processREQLowWrite },  // Request pin low (EVENT_REQ_LOW)
  { (STATETRANSITION) processInvalid,  (STATETRANSITION) processTCRead,     (STATETRANSITION) processInvalid     },  // Transfer Complete (EVENT_TC)
};
const int8_t* stateToString[NUM_OF_STATES] = { (int8_t*)"IDLE", (int8_t*)"READ", (int8_t*)"WRITE" };
const int8_t* eventToString[NUM_OF_EVENTS] = { (int8_t*)"REQ_HIGH", (int8_t*)"REQ_LOW", (int8_t*)"TC" };


#define PROCESS_READ(inst)    {\
  /* --- 1. check FIFO queue --- */\
  if (QUEUE_ISEMPTY(inst))\
  {\
    LOG_INFO((FSMINST_APPL_PROC == inst) ? "WARNING: Queue C to A is empty!" : "WARNING: Queue A to C is empty!");\
    return STATE_IDLE;     /* abort, nothing to read */\
  }\
  /* --- 1. b) change interrupt edge selection */\
  SET_REQ_INTERRUPT_EDGE(inst, EDGE_FALLING);\
  CHECK_REQ_PIN(inst); /* detect an invalid REQ transition */\
  /* --- 2. enable SPI --- */\
  ENABLE_SPI(inst);\
  /* --- 3. get buffer address and set up DMA transfer --- */\
  SETUP_DMA_READ(inst);\
  /* --- 4. set ACK = 1 --- */\
  SPI_ACK_HIGH(inst);\
}

#define PROCESS_WRITE(inst)   {\
  /* --- 1. check FIFO queue --- */\
  if (QUEUE_ISFULL(inst))\
  {\
    (FSMINST_APPL_PROC == inst) ? systemStats.fullCountA++ : systemStats.fullCountC++;\
    LOG_INFO((FSMINST_APPL_PROC == inst) ? "WARNING: Queue A to C is full!" : "WARNING in PORT4_ISR: Queue C to A is full!");\
    return STATE_IDLE;   /* abort, no space in the queue */\
  }\
  /* --- 1. b) change interrupt edge selection and check the REQ pin */\
  SET_REQ_INTERRUPT_EDGE(inst, EDGE_FALLING);\
  CHECK_REQ_PIN(inst); /* detect an invalid REQ transition */\
  /* --- 2. enable SPI --- */\
  ENABLE_SPI(inst);\
  /* --- 3. get buffer address and set up DMA transfer --- */\
  SETUP_DMA_WRITE(inst);\
  /* --- 4. set ACK = 1 --- */\
  SPI_ACK_HIGH(inst);\
}

#define PROCESS_ABORTED(inst)   {\
  /* ABORTED by the processor */\
  /* --- 1. stop the DMA --- */\
  STOP_DMA(inst);\
  /* --- 2. change interrupt edge selection --- */\
  SET_REQ_INTERRUPT_EDGE(inst, EDGE_RISING);\
  CLEAR_REQ_IFG(inst); /* clear the IFG to make sure no interrupt is triggered when switching the selection (see user guide p.310) */\
  /* --- wait for the current operation to complete and clear RX buffer overrun flag (however, a buffer overrun should never occur) --- */\
  /*SPI_WAIT_BSY(inst); -> NOT necessary if SPI is disabled at the end */\
  /* --- 3. set ACK = 0 --- */\
  SPI_ACK_LOW(inst);\
  /* --- 4. stats / debugging --- */\
  LOG_INFO("ABORTED");\
  (FSMINST_APPL_PROC == inst) ? systemStats.abortCountA++ : systemStats.abortCountC++;\
  /* --- 5. disable the SPI --- */\
  EUSCI_SPI_RECEIVEDATA(SPI_ADDR(inst)); /* clear RX buffer */\
  NOP3; /* make sure enough time has passed before disabling the SPI */\
  DISABLE_SPI(inst);\
}

#define PROCESS_COMPLETED(inst, msgSize, mode)  {\
  /* --- 1. stop the DMA --- */\
  STOP_DMA(inst);\
  /* --- 2. change interrupt edge selection --- */\
  SET_REQ_INTERRUPT_EDGE(inst, EDGE_RISING);\
  CLEAR_REQ_IFG(inst); /* clear the IFG to make sure no interrupt is triggered when switching the selection (see user guide p.310) */\
  /* --- wait for the current operation to complete and clear RX buffer overrun flag (however, a buffer overrun should never occur) --- */\
  /*SPI_WAIT_BSY(inst);  -> NOT necessary if SPI is disabled at the end! */\
  if (OPMODE_WRITE == mode) /* which mode, read or write? */\
  {\
    /* --- 3. update data structures (add message to the queue) --- */\
    QUEUE_ADD(inst, msgSize);  /* this was a write operation: add message to queue (= update meta data) */\
    (FSMINST_APPL_PROC == inst) ? systemStats.writeCountA++ : systemStats.writeCountC++;\
    (FSMINST_APPL_PROC == inst) ? (systemStats.writeByteCountA += msgSize) : (systemStats.writeByteCountC += msgSize);\
    /* --- 4. set IND = 1 --- */\
    SPI_IND_HIGH((FSMINST_APPL_PROC == inst) ? FSMINST_COMM_PROC : FSMINST_APPL_PROC);  /* now data is available for the other processor */\
    LOG_INFO_1("%u bytes transfered", msgSize);\
    LOG_VERBOSE((FSMINST_APPL_PROC == inst) ? "Message added to queue A to C" : "Message added to queue C to A");\
  } else  /* a read operation has terminated */\
  {\
    /* --- 3. update data structures (remove the message from the queue) --- */\
    QUEUE_REMOVE(inst);      /* this was a read operation: remove message from queue (= update meta data) */\
    (FSMINST_APPL_PROC == inst) ? systemStats.readCountA++ : systemStats.readCountC++;\
    /* --- 4. set IND = 0 if queue empty --- */\
    if (QUEUE_ISEMPTY(inst))\
    {\
      SPI_IND_LOW(inst);\
    } else {\
      NOP2; /* if/else balancing */\
    }\
  }\
  /* --- 5. set ACK = 0 (note: for a write operation, set ACK = 0 at the end) --- */\
  SPI_ACK_LOW(inst);\
  /* --- 6. disable the SPI --- */\
  CHECK_SPI_RXBUF_OVR_FLAG(mode); /* a read will almost always result in a RX buffer overrun, therefore only check flag when a write was performed */\
  EUSCI_SPI_RECEIVEDATA(SPI_ADDR(inst)); /* clear the receive buffer */\
  DISABLE_SPI(inst);\
  LOG_INFO("COMPLETED");\
}

#define CHECK_REQ_PIN(inst)   {\
  if (0 == SPI_REQ_STATUS(inst))\
  {\
    /* REQ pin just switched back to low -> change interrupt edge again! */\
    SET_REQ_INTERRUPT_EDGE(inst, EDGE_RISING);\
    CLEAR_REQ_IFG(inst);  /* clear the IFG to make sure no interrupt is triggered when switching the selection (see user guide p.310) */\
    LOG_INFO("WARNING: REQ pin low before the end of the ISR");\
    return STATE_IDLE;    /* abort here, back to idle (marked as 'Invalid REQ' in the FSM) */\
  }\
}



// handles the transfer complete transition (for a read operation)
FSMSTATE processTCRead(const FSMINSTANCE inst)
{
#ifdef DEBUG
  uint16_t msgSize = DMA_REMAINING_BYTES(inst) + 1;  // at this point, the counter will be reset to the number of bytes to transmit
  LOG_INFO_1("%u bytes transfered", msgSize);
  LOG_VERBOSE((FSMINST_APPL_PROC == inst) ? "Element removed from queue C to A" : "Element removed from queue A to C");
#endif // DEBUG
  if (FSMINST_APPL_PROC == inst)
  {
    PROCESS_COMPLETED(FSMINST_APPL_PROC, 0, OPMODE_READ);
  } else
  {
    PROCESS_COMPLETED(FSMINST_COMM_PROC, 0, OPMODE_READ);
  }
  return STATE_IDLE;
}


// handle an "invalid" state transition (not necessarily an error condition!)
FSMSTATE processInvalid(const FSMINSTANCE inst)
{
  // just for debugging: see if this transition occurs
  systemStats.invStateCount++;
#ifdef DEBUG
  if (STATE_IDLE != currentState[inst])
  {
    LOG_INFO("WARNING: Invalid state transition detected!");
  }
#endif // DEBUG
  return currentState[inst];   // no change to the state
}


// handles the transition triggered by the REQ pin (toggled from high to low during a read operation)
FSMSTATE processREQLowRead(const FSMINSTANCE inst)
{
  if (FSMINST_APPL_PROC == inst)
  {
    PROCESS_ABORTED(FSMINST_APPL_PROC);
  } else
  {
    PROCESS_ABORTED(FSMINST_COMM_PROC);
  }
  return STATE_IDLE;  // back to idle state
}


// handles the transition triggered by the REQ pin (toggled from high to low during a write operation)
FSMSTATE processREQLowWrite(const FSMINSTANCE inst)
{
  volatile uint16_t msgSize;
  if (FSMINST_APPL_PROC == inst)
  {
    if (DMA_GETINTERRUPTSTATUS(DMA_A))
    {
      NOP6;
      msgSize = MESSAGE_SIZE;
    } else
    {
      msgSize = MESSAGE_SIZE - DMA_REMAINING_BYTES(FSMINST_APPL_PROC);
    }
    if (0 == msgSize)
    {
      NOP3;
      LOG_INFO("WARNING: message size is 0 bytes -> dropped");
      PROCESS_ABORTED(FSMINST_APPL_PROC);
    } else
    {
      NOP2;   // if/else balancing
      LOG_INFO_1("Message received (%u bytes)", msgSize);
      PROCESS_COMPLETED(FSMINST_APPL_PROC, msgSize, OPMODE_WRITE);
    }
  } else
  {
    NOP2;     // if/else balancing
    if (DMA_GETINTERRUPTSTATUS(DMA_C))
    {
      NOP6;
      msgSize = MESSAGE_SIZE;
    } else
    {
      msgSize = MESSAGE_SIZE - DMA_REMAINING_BYTES(FSMINST_COMM_PROC);
    }
    if (0 == msgSize)
    {
      LOG_INFO("WARNING: message size is 0 bytes -> dropped");
      PROCESS_ABORTED(FSMINST_COMM_PROC);
      NOP;
    } else
    {
      // end of message: transfer complete
      LOG_INFO_1("Message received (%u bytes)", msgSize);
      PROCESS_COMPLETED(FSMINST_COMM_PROC, msgSize, OPMODE_WRITE);
      NOP2;
    }
  }
  return STATE_IDLE;  // back to idle state
}


// handles the transition triggered by the REQ pin (toggled from low to high in IDLE state)
FSMSTATE processREQHigh(const FSMINSTANCE inst)
{
  if (FSMINST_APPL_PROC == inst)
  {
    if (OPMODE_WRITE == SPI_MODE_STATUS(FSMINST_APPL_PROC))   // which mode: write or read?
    {
      PROCESS_WRITE(FSMINST_APPL_PROC);
      return STATE_WRITE;
    } else
    {
      NOP;
      PROCESS_READ(FSMINST_APPL_PROC);
      return STATE_READ;
    }
  } else
  {
    if (OPMODE_WRITE == SPI_MODE_STATUS(FSMINST_COMM_PROC))   // which mode: write or read?
    {
      PROCESS_WRITE(FSMINST_COMM_PROC);
      return STATE_WRITE;
    } else
    {
      NOP;
      PROCESS_READ(FSMINST_COMM_PROC);
      return STATE_READ;
    }
  }
}

