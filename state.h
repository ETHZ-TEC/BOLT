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
 * state.h
 * State machine
 * all the state transitions are defined here (inline functions)
 */

#ifndef STATE_H
#define STATE_H


/* typedefs */

typedef enum
{
  STATE_IDLE = 0,   // the states Completed and Aborted are essentially the same state as Idle and are therefore not implemented
  STATE_READ,
  STATE_WRITE,
  NUM_OF_STATES
} FSMSTATE;

typedef enum
{
  FSMINST_APPL_PROC = 0,    // application processor side
  FSMINST_COMM_PROC = 1,    // communication processor side
  NUM_OF_FSM_INSTANCES
} FSMINSTANCE;

typedef enum
{
  EVENT_REQ_HIGH = 0,
  EVENT_REQ_LOW,
  EVENT_TC,      // transfer complete
  NUM_OF_EVENTS
} FSMEVENT;


typedef FSMSTATE (*STATETRANSITION)(const FSMINSTANCE inst);   // function pointer declaration for a state transition


/* global variables */

extern volatile FSMSTATE currentState[NUM_OF_FSM_INSTANCES];
extern const STATETRANSITION stateMachine[NUM_OF_EVENTS][NUM_OF_STATES];
extern const int8_t* stateToString[NUM_OF_STATES];
extern const int8_t* eventToString[NUM_OF_EVENTS];


/* inline functions */

// processes an event to execute a transition of the state machine and receives the new state
#pragma FUNC_ALWAYS_INLINE(processEvent)
static __inline void processEvent(const FSMINSTANCE inst, const FSMEVENT event)
{
  ASSERT((event < NUM_OF_EVENTS) && (inst < NUM_OF_FSM_INSTANCES));
  LOG_INFO(concatStrings((int8_t*)(FSMINST_APPL_PROC == inst ? "Inst A: state %, event %": "Inst C: state %, event %"), stateToString[currentState[inst]], eventToString[event], debugBuffer, DEBUG_BUFFER_SIZE));
  // execute the transition and store the new state
  currentState[inst] = ((STATETRANSITION) stateMachine[event][currentState[inst]])(inst);
}



#endif // STATE_H
