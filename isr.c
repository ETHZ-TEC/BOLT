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
 * isr.c
 * interrupt service routines
 *
 * Note:
 * - port 3 interrupts have higher priority than port 4 interrupts
 * - DMA TC interrupt has higher priority than port interrupts
 */

#include "main.h"


/*
 * check the current FSM state and adjust the low-power mode accordingly
 */
#define ADJUST_LPM(inst)  {\
                if (STATE_IDLE != currentState[inst])\
                {\
                  LPM4_TO_LPM0_AFTER_ISR;  /* make sure the system stays in LPM0 after the ISR */\
                  NOP7; /* if/else balancing */\
                } else if (STATE_IDLE == currentState[(inst == FSMINST_APPL_PROC ? FSMINST_COMM_PROC : FSMINST_APPL_PROC)])\
                {\
                  ENTER_LPM4_AFTER_ISR;  /* the whole system is in idle state -> enter LPM4 */\
                } else\
                {\
                  NOP5; /* if/else balancing */\
                }\
              }

#define PORT_A_ISR_CODE   {\
                GPIO_clearInterruptFlagInline(SPI_A_REQ_PORT, SPI_A_REQ_PIN);   /* must be cleared at the beginning! */\
                /* more efficient implementation with the following if/else*/\
                if (GPIO_getInputPinValueInline(SPI_A_REQ_PORT, SPI_A_REQ_PIN))\
                {\
                  processEvent(FSMINST_APPL_PROC, EVENT_REQ_HIGH);\
                  NOP2; /* balancing betw. if/else */\
                } else\
                {\
                  processEvent(FSMINST_APPL_PROC, EVENT_REQ_LOW);\
                }\
              }

#define PORT_C_ISR_CODE   {\
                GPIO_clearInterruptFlagInline(SPI_C_REQ_PORT, SPI_C_REQ_PIN);   /* must be cleared at the beginning! */\
                /* more efficient implementation with the following if/else*/\
                if (GPIO_getInputPinValueInline(SPI_C_REQ_PORT, SPI_C_REQ_PIN))\
                {\
                  processEvent(FSMINST_COMM_PROC, EVENT_REQ_HIGH);\
                  NOP2; /* balancing betw. if/else */\
                } else\
                {\
                  processEvent(FSMINST_COMM_PROC, EVENT_REQ_LOW);\
                }\
              }


// DMA ISR, handles the TC (transfer complete) interrupt for both DMA channels
#pragma vector=DMA_VECTOR
__interrupt void DMA_ISR(void)
{
  LOG_VERBOSE("DMA TC interrupt");

  if (DMA_GETINTERRUPTSTATUS(DMA_A))    // or use: (DMAIV == 0x02)
  {
    processEvent(FSMINST_APPL_PROC, EVENT_TC);
    DMA_CLEARINTERRUPT(DMA_A);
    NOP2;
  } else  // no need to check whether the IFG is set
  {
    processEvent(FSMINST_COMM_PROC, EVENT_TC);
    DMA_CLEARINTERRUPT(DMA_C);
  }

  // enter LPM4 if the whole system is in idle
  if (STATE_IDLE == currentState[FSMINST_APPL_PROC])
  {
    if (STATE_IDLE == currentState[FSMINST_COMM_PROC])
    {
      ENTER_LPM4_AFTER_ISR;
    } else
    {
      NOP5;   // if/else balancing
    }
  } else
  {
    NOP7;   // if/else balancing
    NOP5;   // if/else balancing
  }
}


#pragma vector=PORT_COMM_PROC
__interrupt void PORT_C_ISR(void)
{
  LOG_VERBOSE("Interrupt on port 4");
#ifdef DEBUG
  if (GPIO_getInterruptStatusInline(SPI_C_REQ_PORT, SPI_C_REQ_PIN))
  {
#endif // DEBUG
    PORT_C_ISR_CODE;

    ADJUST_LPM(FSMINST_COMM_PROC);
#ifdef DEBUG
  } else
  {
    LOG_ERROR("ERROR: Unexpected interrupt on port 4!");
  }
#endif // DEBUG
}


// handles interrupts on port PORT_APPL_PROC (for REQ pin on interface shared with the application processor)
#pragma vector=PORT_APPL_PROC
__interrupt void PORT_A_ISR(void)
{
  LOG_VERBOSE("Interrupt on port 3");
#ifdef DEBUG
  if (GPIO_getInterruptStatusInline(SPI_A_REQ_PORT, SPI_A_REQ_PIN))
  {
#endif // DEBUG
    PORT_A_ISR_CODE;

    ADJUST_LPM(FSMINST_APPL_PROC);
#ifdef DEBUG
  } else
  {
    LOG_ERROR("ERROR: Unexpected interrupt on port 3!");
  }
#endif // DEBUG
}


// handles system NMI (non-maskable interrupts)
#pragma vector=SYSNMI_VECTOR
__interrupt void SYSNMI_ISR(void)
{
  switch(__even_in_range(SYSSNIV, SYSSNIV_CBDIFG))
  {
  case SYSSNIV_UBDIFG:    // FRAM Uncorrectable bit Error
    LOG_ERROR("ERROR: uncorrectable bit in FRAM");
    break;
  case SYSSNIV_MPUSEGPIFG:  // enc. IP mem segm. violation
    LOG_ERROR("ERROR: enc. IP memory segment violation");
    break;
  case SYSSNIV_MPUSEGIIFG:  // information mem segment violation
    LOG_ERROR("ERROR: information memory segment violation");
    break;
  // MPU interrupt
  case SYSSNIV_MPUSEG1IFG:  // segment 1 mem violation
  case SYSSNIV_MPUSEG2IFG:  // segment 2 mem violation
  case SYSSNIV_MPUSEG3IFG:  // segment 3 mem violation
    LOG_ERROR("ERROR: memory access violation");
    break;
  case SYSSNIV_VMAIFG:    // vacant mem access (invalid mem. address)
    LOG_ERROR("ERROR: vacant memory access");
    break;
  case SYSSNIV_JMBINIFG:    // mailbox input / output (JTAG)
  case SYSSNIV_JMBOUTIFG:
    LOG_ERROR("ERROR: mailbox input / output SYSNMI");
    break;
  case SYSSNIV_CBDIFG:    // Correctable FRAM bit error detection -> maybe make some stats
    LOG_ERROR("ERROR: correctable FRAM bit error detected");
    break;
  default:
    LOG_ERROR("ERROR: non-maskable system interrupt occurred");
    break;
  }
}


// handles UNMI (non-maskable user interrupts)
#pragma vector=UNMI_VECTOR
__interrupt void UNMI_ISR(void)
{
  switch(__even_in_range(SYSUNIV, SYSUNIV_OFIFG))
  {
  case SYSUNIV_NMIIFG:
    LOG_ERROR("ERROR: NMI pin");
    break;
  case SYSUNIV_OFIFG:
    LOG_ERROR("ERROR: oscillator fault");
    break;
  default:
    LOG_ERROR("ERROR: Non-maskable user interrupt occurred");
    break;
  }
}


// dummy ISRs

/*
#pragma vector=USCI_A1_VECTOR         // SPI A1 interrupts
__interrupt void USCI_A1_ISR(void) { LED_ERROR_TOGGLE; }

#pragma vector=USCI_B0_VECTOR         // SPI B0 interrupts
__interrupt void USCI_B0_ISR(void) { LED_ERROR_TOGGLE; }

#pragma vector=AES256_VECTOR
__interrupt void AES256_ISR(void) { LED_ERROR_TOGGLE; }

#pragma vector=RTC_VECTOR
__interrupt void RTC_ISR(void) { LED_ERROR_TOGGLE; }

#pragma vector=TIMER3_A1_VECTOR
__interrupt void TIMER3_A1_ISR(void) { LED_ERROR_TOGGLE; }

#pragma vector=TIMER3_A0_VECTOR
__interrupt void TIMER3_A0_ISR(void) { LED_ERROR_TOGGLE; }

#pragma vector=PORT2_VECTOR
__interrupt void PORT2_ISR(void) { LED_ERROR_TOGGLE; }

#pragma vector=TIMER2_A1_VECTOR
__interrupt void TIMER2_A1_ISR(void) { LED_ERROR_TOGGLE; }

#pragma vector=TIMER2_A0_VECTOR
__interrupt void TIMER2_A0_ISR(void) { LED_ERROR_TOGGLE; }

#pragma vector=TIMER1_A1_VECTOR
__interrupt void TIMER1_A1_ISR(void) { LED_ERROR_TOGGLE; }

#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_A0_ISR(void) { LED_ERROR_TOGGLE; }

#pragma vector=TIMER0_A1_VECTOR
__interrupt void TIMER0_A1_ISR(void) { LED_ERROR_TOGGLE; }

#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void) { LED_ERROR_TOGGLE; }

#pragma vector=ADC12_VECTOR
__interrupt void ADC12_ISR(void) { LED_ERROR_TOGGLE; }

#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void) { LED_ERROR_TOGGLE; }

#pragma vector=WDT_VECTOR
__interrupt void WDT_ISR(void) { LED_ERROR_TOGGLE; }

#pragma vector=TIMER0_B1_VECTOR
__interrupt void TIMER0_B1_ISR(void) { LED_ERROR_TOGGLE; }

#pragma vector=TIMER0_B0_VECTOR
__interrupt void TIMER0_B0_ISR(void) { LED_ERROR_TOGGLE; }

#pragma vector=COMP_E_VECTOR
__interrupt void COMP_E_ISR(void) { LED_ERROR_TOGGLE; }
*/
