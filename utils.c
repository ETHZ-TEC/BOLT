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
 * util.c
 * utility functions
 */

#include "main.h"


#if defined(DEBUG) || defined(ERROR_LOGGING_TO_FRAM)
  int8_t debugBuffer[DEBUG_BUFFER_SIZE];   // define a globally valid buffer for debugging (UART output)
#endif
#ifndef DEBUG
  #pragma DATA_SECTION(lastErrorMsg, ".sysmem")
  int8_t lastErrorMsg[ERROR_BUFFER_SIZE] = { 0 };
#endif // DEBUG


// clear a memory region in the SRAM or FRAM (copies the word fillValue [numWords] times, beginning at startAddress)
void fillRAM(const uint16_t fillValue,
       uint16_t *startAddress,
       uint16_t numWords)
{
  ASSERT(((uint16_t)startAddress >= FRAM_START && ((uint16_t)startAddress + numWords * 2) <= FRAM_END) ||
       ((uint16_t)startAddress >= SRAM_START && ((uint16_t)startAddress + numWords * 2) <= SRAM_END));
  while (numWords > 0)
  {
    *startAddress++ = fillValue;  // write to FRAM (++ increments the address by 2 after the assignment operation)
    numWords--;
  }
}


// prints a string (zero-terminated char array with a newline appended) to the UART interface with polling
void printLine(int8_t* string)
{
  if (!string)
  {
    return;
  }
  for (; 0 != *string; string++)  // print each character in array to USART1
  {
    UART_SEND_BYTE(*string);    // send 1 byte
  }

  UART_SEND_BYTE('\r');
  UART_SEND_BYTE('\n');      // print a newline and carriage return
}


// prints a string (zero-terminated char array WITHOUT a newline appended) to the UART interface with polling
void printString(int8_t* string)
{
  if (!string)
  {
    return;
  }
  for (; 0 != *string; string++)  // print each character in array to USART1
  {
    UART_SEND_BYTE(*string);    // send 1 byte
  }
}


// prints each character of a zero-terminated char array in hex format to the UART interface with polling (e.g. 65 = 'A' is printed as '0x41 '); a newline is appended at the end
void printAsHexString(uint8_t* string)
{
  uint8_t upper, lower;
  if (!string)
  {
    return;
  }

  for (; 0 != *string; string++)  // print each character in array to USART1
  {
    UART_SEND_BYTE('0');
    UART_SEND_BYTE('x');
    upper = (*string >> 4);
    lower = (*string & 0x0f);
    UART_SEND_BYTE((upper > 9) ? ('a' + upper - 10) : ('0' + upper));
    UART_SEND_BYTE((lower > 9) ? ('a' + lower - 10) : ('0' + lower));
    UART_SEND_BYTE(' ');
  }
  UART_SEND_BYTE('\n');      // print a newline and carriage return
  UART_SEND_BYTE('\r');
}


/*
 * composes a string according to the given format, takes up to 2 arguments (integers)
 * the resulting string will be in outBuffer
 *
 * valid format specifiers:
 *   %s   signed integer (16-bit)
 *   %u   unsigned integer (16-bit)
 *   %S   signed integer (32-bit)
 *   %U   unsigned integer (32-bit)
 *   %h   unsigned integer (16-bit) in hex format
 *   %H   unsigned integer (32-bit) in hex format
 *
 * Note: The user is responsible for allocating enough memory for the output buffer!
 */
int8_t* composeString(int8_t* format, const uint32_t arg1, const uint32_t arg2, int8_t* outBuffer)
{
  int8_t* tmp = outBuffer;
  int8_t currArg = 0;

  if (!outBuffer)
  {
    return 0;
  }
  for (; 0 != *format; format++)
  {
    if (*format == '%' && currArg < 2)
    {
      format++;
      if (*format == '%')
      {
        *tmp = '%';
        tmp++;
      } else
      {
        if (*format == 's' || *format == 'u')  // signed / unsigned integer
        {
          tmp += int16ToStr(currArg ? arg2 : arg1, (*format == 's') > 0, tmp);
        } else if (*format == 'h')
        {
          tmp += int16ToHexStr(currArg ? arg2 : arg1, tmp);
        } else if (*format == 'H')
        {
          tmp += int32ToHexStr(currArg ? arg2 : arg1, tmp);
        } else if (*format == 'S' || *format == 'U')
        {
          tmp += int32ToStr(currArg ? arg2 : arg1, (*format == 'S') > 0, tmp);
        } else
        {
          LOG_INFO("WARNING: Invalid format identifier found in composeString()");
        }
        currArg++;
      }
    } else
    {
      *tmp = *format;
      tmp++;
    }
  }
  *tmp = 0;
  return outBuffer;
}


/*
 * combines two strings (zero-terminated char arrays) into a new string according to the given format
 * the resulting string will be in outBuffer
 *
 * use % as place holders for the strings (arg1 and arg2)
 */
int8_t* concatStrings(int8_t* format, const int8_t* arg1, const int8_t* arg2, int8_t* const outBuffer, uint16_t maxLength)
{
  int8_t* tmp = outBuffer;
  int8_t currArg = 0;

  if (!outBuffer || !arg1 || maxLength < 16)
  {
    return 0;
  }
  maxLength--;   // reserve 1 byte for the 0

  for (; (0 != *format) && (0 != maxLength); format++)
  {
    if ('%' == *format && currArg < 2)
    {
      if (1 == currArg && !arg2)
      {
        LOG_INFO("WARNING: Not enough input arguments provided for concatStrings()");
        break;  // stop decoding
      }
      // copy the string
      while ( (0 != maxLength) && (0 != *(currArg ? arg2 : arg1)) )
      {
        *tmp = (currArg ? *arg2 : *arg1);
        (currArg ? arg2++ : arg1++);
        tmp++;
        maxLength--;
      }
      currArg++;
    } else
    {
      *tmp = *format;
      tmp++;
      maxLength--;
    }
  }

  *tmp = 0;
  return outBuffer;
}


// returns a string containing the given file name and line number (outBuffer must be big enough!)
int8_t* getFileAndLineString(int8_t* outBuffer, const int8_t* f, const uint16_t l)
{
  static const int8_t* format = (const int8_t*)"(file %, line %)";
  int8_t* tmp = outBuffer;
  int8_t currArg = 0;

  if (!outBuffer)
  {
    return 0;
  }

  for (; 0 != *format; format++)
  {
    if ('%' == *format)
    {
      if (0 == currArg)
      {
        // skip dots and slashes at beginning (if present)
        while ('/' == *f || '.' == *f) f++;

        // copy the string
        while (0 != *f)
        {
          *tmp = *f;
          f++;
          tmp++;
        }
      } else
        tmp += int16ToStr(l, 0, tmp);
      currArg++;
    } else
    {
      *tmp = *format;
      tmp++;
    }
  }

  *tmp = 0;
  return outBuffer;
}


// converts a 32-bit integer value to a string (Notes: outBuffer must be at least 16 bytes long! Besides, this function is relatively slow and should only be used for debugging purposes)
int8_t int32ToStr(uint32_t input, const int8_t sign, int8_t* outBuffer)
{
  int8_t acTmp[16];
  uint8_t i = 0, r, neg = 0;

  if (!outBuffer)
  {
    return 0;
  }

  if (input == 0)
  {
    outBuffer[0] = '0';
    outBuffer[1] = 0;
    return 1;
  }
  if (sign && (input & 0x80000000))
  {
    neg = 1;
    input &= ~0x80000000;
    input = 0x80000000 - input;
  }
  while (input > 0)
  {
    r = input % 10;
    acTmp[i++] = r + '0';
    input /= 10;
  }
  // copy to output buffer
  r = 0;
  if (neg)
  {
    outBuffer[r++] = '-';
  }
  while (i != 0)
  {
    outBuffer[r++] = acTmp[--i];
  }
  outBuffer[r] = 0;    // end of string

  return r;
}


// converts a 16-bit integer value to a string (Notes: outBuffer must be at least 16 bytes long! Besides, this function is relatively slow and should only be used for debugging purposes)
int8_t int16ToStr(uint16_t input, const int8_t sign, int8_t* outBuffer)
{
  int8_t acTmp[8];
  uint8_t i = 0, r, neg = 0;

  if (!outBuffer)
  {
    return 0;
  }
  if (!input)
  {
    outBuffer[0] = '0';
    outBuffer[1] = 0;
    return 1;
  }
  if (sign && (input & 0x8000))
  {
    neg = 1;
    input &= ~0x8000;
    input = 0x8000 - input;
  }
  while (input)
  {
    r = input % 10;
    acTmp[i++] = r + '0';
    input /= 10;
  }
  // copy to output buffer
  r = 0;
  if (neg)
  {
    outBuffer[r++] = '-';
  }
  while (i != 0)
  {
    outBuffer[r++] = acTmp[--i];
  }
  outBuffer[r] = 0;    // end of string

  return r;
}


// converts a 32-bit integer value to a string in hex format (Notes: outBuffer must be at least 11 bytes long!)
int8_t int32ToHexStr(uint32_t input, int8_t* outBuffer)
{
  uint8_t i, r;

  if (!outBuffer)
  {
    return 0;
  }

  //outBuffer[0] = '0';     // don't add '0x' at the beginning
  //outBuffer[1] = 'x';
  outBuffer[8] = 0;     // end of string (it's pos. 10 with '0x' at the beginning)

  for (i = 8; i != 0; i--)
  {
    r = (input & 0xf);
    if (r < 10)
    {
      outBuffer[i - 1] = r + '0';     // it's +1 with '0x' at the beginning
    } else
    {
      outBuffer[i - 1] = r - 10 + 'a';  // it's +1 with '0x' at the beginning
    }
    input >>= 4;  // divide by 16
  }
  return 8;   // 10 with '0x' at the beginning
}


// converts a 16-bit integer value to a string in hex format (Notes: outBuffer must be at least 7 bytes long!)
int8_t int16ToHexStr(uint16_t input, int8_t* outBuffer)
{
  uint8_t i, r;

  if (!outBuffer)
  {
    return 0;
  }

  //outBuffer[0] = '0';     // don't add '0x' at the beginning
  //outBuffer[1] = 'x';
  outBuffer[4] = 0;     // end of string (it's pos. 6 with '0x' at the beginning)

  for (i = 4; i != 0; i--)
  {
    r = (input & 0xf);
    if (r < 10)
    {
      outBuffer[i - 1] = r + '0';     // it's +1 with '0x' at the beginning
    } else
    {
      outBuffer[i - 1] = r - 10 + 'a';  // it's +1 with '0x' at the beginning
    }
    input >>= 4;  // divide by 16
  }
  return 4;   // 6 with '0x' at the beginning
}


// prints out the whole memory content over the UART interface with polling (in hex format, one line per 16-bit memory word)
void printFRAM()
{
  // read application start address:
  uint16_t *startAddr = (uint16_t*)0x4400;

  while ((uint16_t)startAddr != 0)
  {
    printLine(composeString("%h: %h", (uint32_t)startAddr, *startAddr, debugBuffer));
    startAddr++;
  }
}


#ifndef DEBUG
// stores the given string in the FRAM (does not overwrite an existing error message)
void saveErrorMsg(const int8_t* string)
{
  int8_t* dst = lastErrorMsg;
  uint16_t count = ERROR_BUFFER_SIZE;

  if (!string || *lastErrorMsg != 0x00)
  {
    return;
  }
  while (*string && count)
  {
    *dst++ = *string++;    // copy byte by byte
    count--;
  }
  lastErrorMsg[ERROR_BUFFER_SIZE - 1] = 0;   // make sure it is a zero-terminated string
}


// clears the previous error message (if any)
void eraseErrorMsg()
{
  int8_t* dst = lastErrorMsg;
  uint16_t count = ERROR_BUFFER_SIZE;

  while (count)
  {
    *dst++ = 0x00;
    count--;
  }
}
#endif // DEBUG


// note: str must be zero-terminated!
uint16_t getStringLength(const int8_t* str)
{
  const int8_t* start = str;
  while (0 != *str)
  {
    str++;
  }
  return (uint16_t)str - (uint16_t)start;
}


// returns the address of the zero-byte of a string
int8_t* getEOS(int8_t* str)
{
  while (0 != *str)
  {
    str++;
  }
  return str;
}


// returns an encoded string containing the system information (version numbers and statistics); Note: outBuffer must be at least 284 bytes long!
int8_t* getSystemInfoEncoded(int8_t* outBuffer)
{
  // Note: This function is slow and for debugging purpose only!
  composeString((int8_t*)"mc=" MCU_DESC "&fw=%u&cv=%u", CODE_VS, COMPILER_VS, outBuffer);
  composeString((int8_t*)"&po=%U&pl=%U", systemStats.powerOnCount, systemStats.powerLossCount, getEOS(outBuffer));
  composeString((int8_t*)"&cc=%U&ms=%U", systemStats.crashCount, MESSAGE_SIZE, getEOS(outBuffer));
  composeString((int8_t*)"&qa=%U&qc=%U", MAX_NUM_OF_MSG_A_TO_C, MAX_NUM_OF_MSG_C_TO_A, getEOS(outBuffer));
  composeString((int8_t*)"&fa=%U&fc=%U", systemStats.fullCountA, systemStats.fullCountC, getEOS(outBuffer));
  composeString((int8_t*)"&ra=%U&rc=%U", systemStats.readCountA, systemStats.readCountC, getEOS(outBuffer));
  composeString((int8_t*)"&wa=%U&wc=%U", systemStats.writeCountA, systemStats.writeCountC, getEOS(outBuffer));
  composeString((int8_t*)"&aa=%U&ac=%U", systemStats.abortCountA, systemStats.abortCountC, getEOS(outBuffer));
  composeString((int8_t*)"&ba=%U&bc=%U", systemStats.writeByteCountA, systemStats.writeByteCountC, getEOS(outBuffer));
  composeString((int8_t*)"&rs=%h%h", *(uint16_t*)RAND_SEED_ADDR, *(uint16_t*)(RAND_SEED_ADDR + 2), getEOS(outBuffer));
  composeString((int8_t*)"%h%h", *(uint16_t*)(RAND_SEED_ADDR + 4), *(uint16_t*)(RAND_SEED_ADDR + 6), getEOS(outBuffer));
  composeString((int8_t*)"%h%h", *(uint16_t*)(RAND_SEED_ADDR + 8), *(uint16_t*)(RAND_SEED_ADDR + 10), getEOS(outBuffer));
  composeString((int8_t*)"%h%h", *(uint16_t*)(RAND_SEED_ADDR + 12), *(uint16_t*)(RAND_SEED_ADDR + 14), getEOS(outBuffer));

  return outBuffer;
}


// print out the statistics and some information over UART
void logStats()
{
  initUART();
#if defined(ASYNC_INT_DEV_BOARD_V2)

  if (*lastErrorMsg == 0x00)
  {
    printLine((int8_t*)"\r\n\r\nError log: No error found");
  } else
  {
    uint8_t count = 0;
    printLine((int8_t*)"\r\n\r\nLast error message:");
    printLine((int8_t*)lastErrorMsg);
    while (PUSH_BUTTON_PRESSED)     // long-press (~2s) to delete the error message
    {
      if (count > 200)
      {
        eraseErrorMsg();
        printLine((int8_t*)"Error message erased");
        break;
      }
      WAIT(10);
      count++;
    }
    LED_ERROR_OFF;
    while (PUSH_BUTTON_PRESSED);
  }
#else
  // print out system information
  printLine((int8_t*)"\r\nBOLT - (c) 2015, ETH Zurich");
  printLine((int8_t*)"\r\nMCU: " MCU_DESC);
  printLine(composeString((int8_t*)("Firmware version: %u (" COMPILE_DATE ")\r\nCompiler version: %u"), CODE_VS, COMPILER_VS, debugBuffer));
  printLine(composeString((int8_t*)"Power-on count: %U\r\nPower failure count: %U", systemStats.powerOnCount, systemStats.powerLossCount, debugBuffer));
  printLine(composeString((int8_t*)"Crash count: %U\r\nMessage size: %u", systemStats.crashCount, MESSAGE_SIZE, debugBuffer));
  printLine(composeString((int8_t*)"Max. number of messages: %u A, %u C", MAX_NUM_OF_MSG_A_TO_C, MAX_NUM_OF_MSG_C_TO_A, debugBuffer));
  printLine(composeString((int8_t*)"Queue full count: %U A, %U C", systemStats.fullCountA, systemStats.fullCountC, debugBuffer));
  printLine(composeString((int8_t*)"Read count: %U A, %U C", systemStats.readCountA, systemStats.readCountC, debugBuffer));
  printLine(composeString((int8_t*)"Write count: %U A, %U C", systemStats.writeCountA, systemStats.writeCountC, debugBuffer));
  printLine(composeString((int8_t*)"Abort count: %U A, %U C", systemStats.abortCountA, systemStats.abortCountC, debugBuffer));
  printLine(composeString((int8_t*)"Byte count (write): %U A, %U C", systemStats.writeByteCountA, systemStats.writeByteCountC, debugBuffer));
  // print random seed (could be used as an ID)
  printString(composeString((int8_t*)"Random seed: %h-%h", *(uint16_t*)RAND_SEED_ADDR, *(uint16_t*)(RAND_SEED_ADDR + 2), debugBuffer));
  printString(composeString((int8_t*)"-%h-%h", *(uint16_t*)(RAND_SEED_ADDR + 4), *(uint16_t*)(RAND_SEED_ADDR + 6), debugBuffer));
  printString(composeString((int8_t*)"-%h-%h", *(uint16_t*)(RAND_SEED_ADDR + 8), *(uint16_t*)(RAND_SEED_ADDR + 10), debugBuffer));
  printLine(composeString((int8_t*)"-%h-%h", *(uint16_t*)(RAND_SEED_ADDR + 12), *(uint16_t*)(RAND_SEED_ADDR + 14), debugBuffer));

  #ifndef DEBUG
  if (*lastErrorMsg == 0x00)
  {
    printLine((int8_t*)"Error log: No error found");
  } else
  {
    printLine((int8_t*)"Last error message:");
    printLine((int8_t*)lastErrorMsg);
  }
  #endif // DEBUG
#endif
}
