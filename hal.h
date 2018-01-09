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
 * hal.h
 * hardware abstraction layer for the MSP430FR5969
 *
 * definitions in this file are compatible with the Dual Processor Platform
 *
 * Note: when the master clock is set to 8 MHz, the max. supported SPI clock speed is 4 MHz.
 */

#ifndef HAL_H
#define HAL_H


#define DPP2_BOLT                                // uncomment to compile for the version 2 of the DPP


#define MCLK_SPEED              MCLK_SPEED_8MHZ  // system / main clock speed
#define MCLK_CYCLES             8000000          // don't forget to adjust this setting when changing the value for MCLK_SPEED!
#define UART_BAUDRATE           UART_RATE_115200 // baud rate for the UART interface

#define FRAM_START              0x4400           // start address of the FRAM (note: the first few kB are used for the program code!)
#define FRAM_END                0xff7f           // last valid address (note: 0xff80 - 0xffff is reserved for the interrupt vectors)
#define SRAM_START              0x1C00           // start address of the SRAM
#define SRAM_END                0x23ff           // last valid address in the SRAM

#define RAND_SEED_ADDR          0x1A30           // address of the unique 128-bit random number seed (1A30 - 1A3F, little endian)

// LEDs on Dev. Board v2.0: 1.5, 1.4, 4.5 and 2.7  (on Dev. Board v1.0: 3.5, 3.7, 4.5 and 2.7)

#ifdef DPP2_BOLT
  #define LED_1_PORT            GPIO_PORT_P2     // Debug LED
  #define LED_1_PIN             GPIO_PIN7
  #define LED_ERROR_PORT        LED_1_PORT       // LED to turn on when an error occurs
  #define LED_ERROR_PIN         LED_1_PIN        // LED to turn on when an error occurs
#else
  #define PUSH_BUTTON_PORT      GPIO_PORT_P1     // Debug Switch
  #define PUSH_BUTTON_PIN       GPIO_PIN0
  #define LED_1_PORT            GPIO_PORT_P2     // Debug LED
  #define LED_1_PIN             GPIO_PIN7
  #define LED_ERROR_PORT        LED_1_PORT       // LED to turn on when an error occurs
  #define LED_ERROR_PIN         LED_1_PIN        // LED to turn on when an error occurs
  #define SPI_A_FUTUREUSE_PORT  GPIO_PORT_P1
  #define SPI_A_FUTUREUSE_PIN   GPIO_PIN1
#endif // BOLT_V10

#define LED_STATUS_PORT         LED_1_PORT       // define a status LED
#define LED_STATUS_PIN          LED_1_PIN

#define SPI_A                   EUSCI_B0_BASE    // SPI interface shared with the application processor (SPI B0 is assigned to MCU_A in the schematics)
#define SPI_C                   EUSCI_A1_BASE    // SPI interface shared with the communication processor (default: EUSCI_A1_BASE, must be EUSCI_A1_BASE or EUSCI_B0_BASE)
#define DMA_A                   DMA_CHANNEL_0    // DMA channel used for the interface shared with the application processor
#define DMA_C                   DMA_CHANNEL_1    // DMA channel used for the interface shared with the communication processor

#define SPI_CKPL                SPI_CKPL_0       // SPI clock polarity (must be of enum type SPICKPL)
#define SPI_CKPH                SPI_CKPH_0       // SPI clock phase (must be of enum type SPICKPH, see note below)

/*
 * Note on the SPI phase and polarity:
 * For CPOL = 1 and ...
 * ... CPHA = 0, data is captured on the clock's rising edge and changed (latched) on the falling edge.
 * ... CPHA = 1, data is captured on the clock's falling edge and changed (latched) on the rising edge.
 * For CPOL = 1 and ...
 * ... CPHA = 0, data is captured on the clock's falling edge and changed (latched) on the rising edge.
 * ... CPHA = 1, data is captured on the clock's rising edge and changed (latched) on the falling edge.
 */

#define PORT_APPL_PROC          PORT3_VECTOR
#define PORT_COMM_PROC          PORT4_VECTOR

// Application processor pins (Note: the pins A_IND, A_MODE, A_REQ and A_ACK must be on the same port)
#define SPI_A_IND_PORT          GPIO_PORT_P3
#define SPI_A_IND_PIN           GPIO_PIN0
#define SPI_A_MODE_PORT         GPIO_PORT_P3
#define SPI_A_MODE_PIN          GPIO_PIN1
#define SPI_A_REQ_PORT          GPIO_PORT_P3
#define SPI_A_REQ_PIN           GPIO_PIN2
#define SPI_A_ACK_PORT          GPIO_PORT_P3
#define SPI_A_ACK_PIN           GPIO_PIN3

// Communication processor pins (Note: the pins C_IND, C_MODE, C_REQ and C_ACK must be on the same port)
#define SPI_C_FUTUREUSE_PORT    GPIO_PORT_P1
#define SPI_C_FUTUREUSE_PIN     GPIO_PIN2
#define SPI_C_IND_PORT          GPIO_PORT_P4
#define SPI_C_IND_PIN           GPIO_PIN0
#define SPI_C_MODE_PORT         GPIO_PORT_P4
#define SPI_C_MODE_PIN          GPIO_PIN1
#define SPI_C_REQ_PORT          GPIO_PORT_P4
#define SPI_C_REQ_PIN           GPIO_PIN2
#define SPI_C_ACK_PORT          GPIO_PORT_P4
#define SPI_C_ACK_PIN           GPIO_PIN3


#endif // HAL_H

