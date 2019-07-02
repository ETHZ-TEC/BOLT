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
 * util.h
 * utility functions
 */

#ifndef UTILS_H
#define UTILS_H


#define DEBUG_BUFFER_SIZE       128
#define ERROR_BUFFER_SIZE       128


#define MIN(x, y)               ( (x) < (y) ? (x) : (y) )
#define MAX(x, y)               ( (x) < (y) ? (y) : (x) )
#define LIMIT(val, min, max)    ( MAX(min, MIN(val, max)) )

#define REG_VAL16(x)            ( *((volatile uint16_t*)((uint16_t)(x))) )    // returns the 16-bit value at the given address
#define REG_VAL8(x)             ( *((volatile uint8_t*)((uint16_t)(x))) )     // returns the 16-bit value at the given address

#define RAND_SEED_ADDR          0x1A30    // address of the unique 128-bit random number seed (1A30 - 1A3F, little endian)

#ifdef DEBUG
  // in order to use these macros, a char buffer named acBuffer must exist in the calling function
  #define LOG_LINE               "----------------------------------------------------------"
  #define LOG_ERROR(msg)                { printLine((int8_t*)(msg)); LED_ERROR_ON; }
  #define LOG_INFO(msg)                 ( printLine((int8_t*)(msg)) )
  #define LOG_INFO_1(msg, arg1)         ( printLine(composeString((int8_t*)(msg), (arg1), 0, debugBuffer)) )
  #define LOG_INFO_2(msg, arg1, arg2)   ( printLine(composeString((int8_t*)(msg), (arg1), (arg2), debugBuffer)) )
  #define ASSERT(expr)                  if (!(expr)) \
                                        { \
                                          printString((int8_t*)"ERROR: Assertion failed "); \
                                          printLine(getFileAndLineString(debugBuffer, (int8_t*)__FILE__, __LINE__)); \
                                          LED_ERROR_ON; \
                                        }
  #ifdef DEBUG_VERBOSE
  #define LOG_VERBOSE(msg)              LOG_INFO(msg)
  #else
  #define LOG_VERBOSE(msg)
  #endif
#else
  extern int8_t lastErrorMsg[ERROR_BUFFER_SIZE];

  #ifdef ERROR_LOGGING_TO_FRAM
    #define LOG_ERROR(msg)       { saveErrorMsg((int8_t*)(msg)); LED_ERROR_ON; }
  #else
    #define LOG_ERROR(msg)       { LED_ERROR_ON; }
  #endif // LOG_ERROR_TO_FRAM

  #define LOG_INFO(msg)
  #define LOG_INFO_1(msg, arg1)
  #define LOG_INFO_2(msg, arg1, arg2)
  #define LOG_VERBOSE(msg)
  #define ASSERT(expr)
#endif // DEBUG


/* typedefs */

typedef enum
{
  OP_FAILED = 0,  // operation failed
  OP_OK   = 1   // operation ok
} OPSTATUS;       // operation status


/* inline functions */

// copies [count] 32-bit words from the source src to the destination dst
#pragma FUNC_ALWAYS_INLINE(memCopy32)
static __inline void memCopy32(uint32_t* src, uint32_t* dst, uint16_t count)
{
  while (count > 0)
  {
    *dst++ = *src++;
    count--;
  }
}

// copies [count] bytes from the source src to the destination dst
#pragma FUNC_ALWAYS_INLINE(memCopy8)
static __inline void memCopy8(uint8_t* src, uint8_t* dst, uint16_t count)
{
  while (count > 0)
  {
    *dst++ = *src++;
    count--;
  }
}


/* globals */

#if defined(DEBUG) || defined(ERROR_LOGGING_TO_FRAM)
  extern int8_t debugBuffer[DEBUG_BUFFER_SIZE];
#endif


/* function prototypes */

void fillRAM(const uint16_t fillValue, uint16_t *startAddress, uint16_t numWords);
void printLine(int8_t* string);
void printString(int8_t* string);
void printAsHexString(uint8_t* string);
int8_t* composeString(int8_t* format, const uint32_t arg1, const uint32_t arg2, int8_t* outBuffer);
int8_t* concatStrings(int8_t* format, const int8_t* arg1, const int8_t* arg2, int8_t* const outBuffer, uint16_t maxLength);
int8_t* getFileAndLineString(int8_t* outBuffer, const int8_t* f, const uint16_t l);
int8_t int32ToStr(uint32_t input, const int8_t sign, int8_t* outBuffer);
int8_t int16ToStr(uint16_t iInput, const int8_t sign, int8_t* outBuffer);
int8_t int32ToHexStr(uint32_t iInput, int8_t* outBuffer);
int8_t int16ToHexStr(uint16_t iInput, int8_t* outBuffer);
void printFRAM(void);
void saveErrorMsg(const int8_t* string);
void eraseErrorMsg(void);
uint16_t getStringLength(const int8_t* str);
int8_t* getEOS(int8_t* str);
int8_t* getSystemInfoEncoded(int8_t* outBuffer);
void logStats(void);


#endif // UTILS_H
