/***********************************************************************************************************************
MMBasic

Console.c

Implements the routines to handle all console communications.

Copyright 2011 - 2021 Geoff Graham.  All Rights Reserved.
Adaption to S32K1 family and the ft-RPI-sa hardware: Copyright 2022 Robert Lippmann

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holders nor the names of its contributors
   may be used to endorse or promote products derived from this software
   without specific prior written permission.

4. The name MMBasic be used when referring to the interpreter in any
   documentation and promotional material and the original copyright message
   be displayed  on the console at startup (additional copyright messages may
   be added).

5. All advertising materials mentioning features or use of this software must
   display the following acknowledgement: This product includes software
   developed by Geoff Graham and Peter Mather.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

************************************************************************************************************************/

#include "Version.h"

char ConsoleBuf[CONSOLE_BUF_SIZE];
volatile int ConsoleBufHead = 0;
volatile int ConsoleBufTail = 0;

// initialize the UART1 serial port
void initConsole(void) {

       unsigned int SBR;
       char nbr[7];

    if(Option.Baudrate>115200)
    	Option.Baudrate=115200;
    SBR=48000000/(16*Option.Baudrate);
	if((IP_PCC->PCCn[PCC_LPUART1_INDEX] & PCC_PCCn_CGC_MASK) == 0) {
		IP_PCC->PCCn[PCC_LPUART1_INDEX] = PCC_PCCn_PCS(3);        // PCS = 3: Select FIRCDIV2 = 48MHz
		IP_PCC->PCCn[PCC_LPUART1_INDEX] |= PCC_PCCn_CGC_MASK;    // Enable clock for LPUART1
	} else {
		IntToStr(nbr, Option.Baudrate, 10);         // write baudrate number to string
		nbr[6]=0;
		MMPrintString("\r\nChanged Baudrate to ");
		MMPrintString(nbr);
		MMPrintString(" Baud.\r\nChange terminal baudrate and\r\npress any key\r\n");
		IP_LPUART1->CTRL = 0;                                                                     // disable all to re-configure UART
    }

    IP_LPUART1->BAUD = LPUART_BAUD_OSR(15)|LPUART_BAUD_SBR(SBR);		// configure Baudrate @ 16 times oversampling
    IP_LPUART1->FIFO = LPUART_FIFO_TXFE(1)|LPUART_FIFO_RXFE(1);		// enable 4-byte RX/TX-Fifo
    IP_LPUART1->CTRL = LPUART_CTRL_TE(1)|LPUART_CTRL_RE(1)				// enable receiver & transmitter interrupt from LPUART1
                    | LPUART_CTRL_RIE(1);

    // enable UART1 IRQ
	S32_NVIC->ISER[(LPUART1_RxTx_IRQn>>5)] |= (uint32_t)(1U << ((uint32_t)(LPUART1_RxTx_IRQn) & (uint32_t)0x1FU));

	// this implements the emergency reset routine
    uSec(100000);                                                   // wait for 100ms
    if(getConsole() == '!') {                                       // did we get a !
        uSec(150000);                                               // if so, wait a further 150ms and check that we receive at least four more
        if(getConsole() == '!' && getConsole() == '!' && getConsole() == '!' && getConsole() == '!') {
            int i;
            for(i=0;i<10;i++)
            {
            	uSec(200000);                                          // now wait for 200 msec (2s in total)
            }
            for(i = 0; i < 25; i++)
                if(getConsole() != '!')                             // and check that we get at least 25 consecutive !'s
                    //goto exit_test;
                	break;
            ResetAllFlash();                                        // must be a reset request
            MMPrintString("\r\nMMBasic reset completed\r\n");
            // wait for the user to stop sending characters
            do {
                while(getConsole() != -1);
                uSec(100000);
            } while(getConsole() != -1);
        }
    }
}

// LPUART 1 interrupt handler
void LPUART1_RxTx_IRQHandler(void) {

    while(IP_LPUART1->STAT & LPUART_STAT_RDRF_MASK) {                  // while there is data to read
        if(IP_LPUART1->STAT  & (0b111<<17)) {                    	 	// first check for errors (OR, NF, FE)
            (void)IP_LPUART1->DATA;                                 	// and if there was an error throw away the char
            IP_LPUART1->STAT = 0b111<<17;                              // clear the error on the UART
            continue;                                               // and try the next char
        }
        ConsoleBuf[ConsoleBufHead]  = IP_LPUART1->DATA;       			// store the byte in the ring buffer (and clear IRQ flag)
        if(BreakKey && ConsoleBuf[ConsoleBufHead] == BreakKey) {    // if the user wants to stop the progran
            MMAbort = true;                                         // set the flag for the interpreter to see
            ConsoleBufHead = ConsoleBufTail;                        // empty the buffer
            break;
        }
        ConsoleBufHead = (ConsoleBufHead + 1) % CONSOLE_BUF_SIZE;   // advance the head of the queue
        if(ConsoleBufHead == ConsoleBufTail) {                      // if the buffer has overflowed
            ConsoleBufTail = (ConsoleBufTail + 1) % CONSOLE_BUF_SIZE; // throw away the oldest char
        }
    }
}

// get a keystroke from the console.  Will wait forever for input
// if the char is a cr then replace it with a newline (lf)
int MMgetchar(void) {
    int c;
    static char prevchar = 0;

    loopback:
    do {
        c = MMInkey();
    } while(c == -1);
    if(c == '\n' && prevchar == '\r') {
        prevchar = 0;
        goto loopback;
    }
    prevchar = c;
    if(c == '\n') c = '\r';
    return c;
}



// send a character to the Console serial port
void putConsole(int c) {
    while(!(IP_LPUART1->STAT & LPUART_STAT_TDRE_MASK));       // Wait till the UART transmitter is free.
    IP_LPUART1->DATA = c;                                     // Write data into TXD
}


// returns the number of character waiting in the console input queue
int kbhitConsole(void) {
    int i;
    IP_LPUART1->CTRL &= ~LPUART_CTRL_RIE(1);		// disable RDRF IRQ to avoid race condition
    i = ConsoleBufHead - ConsoleBufTail;
    IP_LPUART1->CTRL |= LPUART_CTRL_RIE(1);		// enable RDRF IRQ
    if(i < 0) i += CONSOLE_BUF_SIZE;
    return i;
}



// get a char from the UART1 serial port (the console)
// will return immediately with -1 if there is no character waiting
int getConsole(void) {
    char c;
    CheckAbort();
    if(ConsoleBufHead == ConsoleBufTail) return -1;
    c = ConsoleBuf[ConsoleBufTail];
    ConsoleBufTail = (ConsoleBufTail + 1) % CONSOLE_BUF_SIZE;       // advance the head of the queue
    return c;
}



void CheckAbort(void) {
    if(MMAbort) {
        WDTimer = 0;                                                // turn off the watchdog timer
        longjmp(mark, 1);                                           // jump back to the input prompt
    }
}




