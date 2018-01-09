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
 * dma.h
 * DMA configuration
 */

#ifndef DMA_H
#define DMA_H


/* definitions and macros */


#define SETUP_DMA_READ(inst)    ( SETUP_DMA(DMA_ADDR(inst), OPMODE_READ, (const uint8_t*)((FSMINST_APPL_PROC == inst) ? queueCtoACtrl.nextRead->data : queueAtoCCtrl.nextRead->data), (FSMINST_APPL_PROC == inst) ? queueCtoACtrl.nextRead->size : queueAtoCCtrl.nextRead->size) )
#define SETUP_DMA_WRITE(inst)   ( SETUP_DMA(DMA_ADDR(inst), OPMODE_WRITE, (const uint8_t*)((FSMINST_APPL_PROC == inst) ? queueAtoCCtrl.nextWrite->data : queueCtoACtrl.nextWrite->data), MESSAGE_SIZE) )
#define STOP_DMA(inst)          { DMA_DISABLETRANSFERS(DMA_ADDR(inst)); DMA_DISABLEINTERRUPT(DMA_ADDR(inst));  DMA_CLEARINTERRUPT(DMA_ADDR(inst)); }


/* must use macro (compiler does not inline this function otherwise) */

#define SETUP_DMA(DMAChannel, mode, memAddress, size)   \
{\
  /* trigger sources: 0 = manual start by calling DMA_startTransfer, 16 = UCA1RXIFG, 17 = UCA1TXIFG, 18 = UCB0RXIFG, 19 = UCB0TXIFG, see datasheet p.28 for more details */\
  /* channel priorities: if two triggers occur at the same time, channel 0 has always a higher priority than channel 1 */\
\
  /*DMA_DISABLETRANSFERS(DMAChannel); -> may be skipped because STOP_DMA() is called after each transfer!*/\
\
  /* read or write? (write means a processor wants to write data into a queue = async interface receives data) */\
  if (OPMODE_WRITE == mode)\
  {\
    DMA_initInline(DMAChannel,              /* channel */\
             DMA_TRANSFER_SINGLE,           /* transfer mode (repeated means enable flag is not cleared after the transfer, but trigger is still needed) */\
             MESSAGE_SIZE,                  /* transfer size (set interrupt flag after every x transfers), use DMA_setTransferSize to change the size later on */\
             (EUSCI_A1_BASE == (DMAChannel == DMA_A ? SPI_A : SPI_C) ? DMA_TRIGGERSOURCE_16 : DMA_TRIGGERSOURCE_18),    /* trigger source select; RX: trigger DMA transfer on receive buffer not empty */\
             DMA_SIZE_SRCBYTE_DSTBYTE,      /* transfer unit (byte-to-byte) */\
             DMA_TRIGGER_RISINGEDGE);       /* trigger type select (trigger upon rising edge of trigger source signal) */\
    DMA_setSrcAddressInline(DMAChannel,\
                EUSCI_SPI_RECEIVEBUFFERADDRESS((DMAChannel == DMA_A ? SPI_A : SPI_C)),   /* source address (SPI RX register) */\
                DMA_DIRECTION_UNCHANGED);   /* always use the same source address */\
    DMA_setDstAddressInline(DMAChannel,\
                (uint32_t)memAddress,       /* destination address */\
                DMA_DIRECTION_INCREMENT);   /* increment destination address after each transfer */\
    LOG_INFO_1("DMA configured for write-to-queue operation (address: %h)", (uint32_t)memAddress);\
    EUSCI_SPI_TRANSMITDATA((DMAChannel == DMA_A ? SPI_A : SPI_C), 0x00);   /* put a zero into the transmit buffer (not really necessary, but looks better) */\
    /*DMA_DISABLEINTERRUPT(DMAChannel); -> we don't need interrupts in write mode -> may be skipped, STOP_DMA always disables interrupts */\
  } else  /* READ (transmit data from async interface to the application or communication processor) */\
  {\
    ASSERT(size <= MESSAGE_SIZE);\
    DMA_initInline(DMAChannel,\
             DMA_TRANSFER_SINGLE,\
             size,    /* usually size - 1, but then the TC interrupt fires too early, i.e. when the last byte has been copied into the shift register and is about to be transferred! */\
             (EUSCI_A1_BASE == (DMAChannel == DMA_A ? SPI_A : SPI_C) ? DMA_TRIGGERSOURCE_17 : DMA_TRIGGERSOURCE_19),   /* TX: trigger DMA transfer on transmit buffer empty */\
             DMA_SIZE_SRCBYTE_DSTBYTE,\
             DMA_TRIGGER_RISINGEDGE);\
    DMA_setSrcAddressInline(DMAChannel,\
                (uint32_t)memAddress + 1,\
                DMA_DIRECTION_INCREMENT);\
    DMA_setDstAddressInline(DMAChannel,\
                EUSCI_SPI_TRANSMITBUFFERADDRESS((DMAChannel == DMA_A ? SPI_A : SPI_C)),\
                DMA_DIRECTION_UNCHANGED);\
\
    /* write the 1st byte into the transmit buffer... (necessary in slave mode!) */\
    EUSCI_SPI_TRANSMITDATA((DMAChannel == DMA_A ? SPI_A : SPI_C), *memAddress);\
    LOG_INFO_2("DMA configured for read-from-queue operation (%u bytes, address: %h)", size, (uint32_t)memAddress);\
    DMA_CLEARINTERRUPT(DMAChannel);     /* clear the interrupt flag */\
    DMA_ENABLEINTERRUPT(DMAChannel);    /* use DMA_getInterruptStatus() to get the trigger */\
  }\
  DMA_ENABLETRANSFERS(DMAChannel);      /* from now on, DMA transfers can be started by the above specified trigger, use DMA_disableTransfers() to disable the trigger */\
}


#endif // DMA_H
