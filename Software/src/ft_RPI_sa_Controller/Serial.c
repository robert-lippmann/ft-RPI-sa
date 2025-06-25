/***********************************************************************************************************************
MMBasic

Serial.c

Handles the serial I/O  commands and functions in MMBasic..

Copyright 2011 - 2021 Geoff Graham.  All Rights Reserved.

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


// variables for com1
int com1 = 0;                                                       // true if COM1 is enabled
int com1_buf_size;                                                  // size of the buffer used to receive chars
int com1_baud = 0;                                                  // determines the baud rate
char *com1_interrupt;                                               // pointer to the interrupt routine
int com1_ilevel;                                                    // number nbr of chars in the buffer for an interrupt
unsigned char *com1Rx_buf;                                          // pointer to the buffer for received characters
volatile int com1Rx_head, com1Rx_tail;                              // head and tail of the ring buffer for com1
unsigned char *com1Tx_buf;                                          // pointer to the buffer for transmitted characters
volatile int com1Tx_head, com1Tx_tail;                              // head and tail of the ring buffer for com1

#define COM1_9B       0b001                                         // 9 bit data enabled
#define COM1_DE       0b010                                         // RS485 enable flag in use
char com1_mode;                                                     // keeps track of the settings for com1
unsigned char com1_bit9 = 0;                                        // used to track the 9th bit

// variables for com2
int com2 = 0;                                                       // true if COM2 is enabled
int com2_buf_size;                                                  // size of the buffer used to receive chars
int com2_baud = 0;                                                  // determines the baud rate
char *com2_interrupt;                                               // pointer to the interrupt routine
int com2_ilevel;                                                    // number nbr of chars in the buffer for an interrupt

int com2Rx_cnt;                                                     // used to count the timer ticks
int com2Rx_start_cnt;                                               // the starting count for com2Rx_cnt
unsigned char *com2Rx_buf;                                          // pointer to the buffer for received characters
volatile int com2Rx_head, com2Rx_tail;                              // head and tail of the ring buffer for com2 Rx

int com2Tx_cnt;                                                     // used to count the timer ticks
int com2Tx_start_cnt;                                               // the starting count for com2Tx_cnt
unsigned char *com2Tx_buf;                                          // pointer to the buffer for transmitted characters
volatile int com2Tx_head, com2Tx_tail;                              // head and tail of the ring buffer for com2 Tx

volatile int com_interrupt;                                         // MMBasic interrupt flags for all serial ports

/***************************************************************************************************
Initialise the serial function including the timer and interrupts.
****************************************************************************************************/
void SerialOpen(char *spec) {
	error("COM not yet implemented");
/*
	int baud, i, inv, oc, s2, de, b9, bufsize, ilevel;
    char *interrupt;

    getargs(&spec, 13, ":,");                                       // this is a macro and must be the first executable stmt
    if(argc != 2 && (argc & 0x01) == 0) error("COM specification");

    de = b9 = inv = oc = s2 = false;
    for(i = 0; i < 4; i++) {
      if(str_equal(argv[argc - 1], "OC")) { oc = true; argc -= 2; }     // get the open collector option
      if(str_equal(argv[argc - 1], "DE")) { de = true; argc -= 2; }     // get the two stop bit option
      if(str_equal(argv[argc - 1], "9BIT")) { b9 = true; argc -= 2; }   // get the two stop bit option
      if(str_equal(argv[argc - 1], "S2")) { s2 = true; argc -= 2; }     // get the two stop bit option
      //if(str_equal(argv[argc - 1], "INV")) { inv = true; argc -= 2; }   // get the invert option
    }

    if(argc < 1 || argc > 8) error("COM specification");

    if(argc >= 3 && *argv[2]) {
        baud = getinteger(argv[2]);                                 // get the baud rate as a number
        if(spec[3] == '1' && baud > BusSpeed/17) error("Baud rate too high");
        if(spec[3] == '2' && baud > BusSpeed/2000) error("Baud rate too high");
    } else
        baud = COM_DEFAULT_BAUD_RATE;

    if(argc >= 5 && *argv[4])
        bufsize = getinteger(argv[4]);                              // get the buffer size as a number
    else
        bufsize = COM_DEFAULT_BUF_SIZE;

    if(argc >= 7 && *argv[6]) {
        InterruptUsed = true;
        interrupt = GetIntAddress(argv[6]);                         // get the interrupt location
        if(argc >= 9 && *argv[8]) {
            if(*argv[8] == '=')                                     // if we are to interrupt on a character
                ilevel = -getint(argv[8] + 1, 1, 255);              // get the interrupt character
            else
                ilevel = getint(argv[8], 1, bufsize);               // otherwise get the interrupt buffer size
        } else
            ilevel = 1;                                             // default interrupt level
    } else {
        interrupt = NULL;
        ilevel = 0;
    }


    if(spec[3] == '1') {
    ///////////////////////////////// this is COM1 ////////////////////////////////////
        int cfg1, cfg2;

        if(com1) error("Already open");
        CheckPin(COM1_RX_PIN, CP_CHECKALL);
        CheckPin(COM1_TX_PIN, CP_CHECKALL);

        com1_buf_size = bufsize;                                    // extracted from the comspec above
        com1_interrupt = interrupt;
        com1_ilevel   = ilevel;
        com_interrupt &= (~(1 << 1));                               // clear the interrupt flag

        // setup for receive
        com1Rx_buf = GetMemory(com1_buf_size);                      // setup the buffer
        com1Rx_head = com1Rx_tail = 0;
        ExtCfg(COM1_RX_PIN, EXT_DIG_IN, 0);                         // pin 15 is Rx (an input)
        ExtCfg(COM1_RX_PIN, EXT_COM_RESERVED, 0);                   // reserve the pin for com use
        PinSetBit(COM1_RX_PIN, inv ? CNPDSET:CNPUSET);              // set a pulldown/pullup on Rx so that the input does not MMFLOAT

        COM1_RX_PPS_OPEN;

        // setup for transmit
        com1Tx_buf = GetMemory(TX_BUFFER_SIZE);                     // setup the buffer
        com1Tx_head = com1Tx_tail = 0;
        ExtCfg(COM1_TX_PIN, EXT_NOT_CONFIG, 0);
        ExtSet(COM1_TX_PIN, 1);                                     // start with the Tx pin high (ie, idle)
        ExtCfg(COM1_TX_PIN, oc ? EXT_OC_OUT : EXT_DIG_OUT, 0);      // set the Tx pin as an output
        ExtCfg(COM1_TX_PIN, EXT_COM_RESERVED, 0);                   // reserve the pin for com use
        COM1_TX_PPS_OPEN;

        com1_bit9 = com1_mode = 0;

        cfg1 = 0;                                                   // default is Tx/Rx only, no invert and no RTS/CTS

        if(de) {
            CheckPin(COM1_EN_PIN, CP_CHECKALL);
            ExtCfg(COM1_EN_PIN, oc ? EXT_OC_OUT : EXT_DIG_OUT, 0);  // set the RTS pin as an output
            ExtCfg(COM1_EN_PIN, EXT_COM_RESERVED, 0);               // reserve the pin for com use
            COM1_DE_PPS_OPEN;
            com1_mode |= COM1_DE;
            cfg1 |= UART_SUPPORT_IEEE_485;
        }
        if(inv) cfg1 |= (UART_INVERT_RECEIVE_POLARITY | UART_INVERT_TRANSMIT_POLARITY);

        cfg2 = 0;                                                   // default is 8 bits, no parity and 1 stop bit
        if(s2) cfg2 |= UART_STOP_BITS_2;
        if(b9) {
            cfg2 |= UART_DATA_SIZE_9_BITS;
            com1_mode |= COM1_9B;
        }

        // setup the UART
        UARTConfigure(UART2, cfg1);
        UARTSetFifoMode(UART2, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
        UARTSetLineControl(UART2, cfg2);
        UARTSetDataRate(UART2, BusSpeed, (com1_baud = baud));
        UARTEnable(UART2, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

        // Configure UART2 RX Interrupt (the Tx Interrupt is enabled in SerialPutchar())
        INTSetVectorPriority(INT_VECTOR_UART(UART2), INT_PRIORITY_LEVEL_3);
        INTSetVectorSubPriority(INT_VECTOR_UART(UART2), INT_SUB_PRIORITY_LEVEL_0);
        INTEnable(INT_SOURCE_UART_RX(UART2), INT_ENABLED);

        com1 = true;

    }
*/
#ifdef COM2
    if (spec[3] == '2') {
    ///////////////////////////////// this is COM2 ////////////////////////////////////
        if(com2) error("Already open");
        CheckPin(COM2_RX_PIN, CP_CHECKALL);
        CheckPin(COM2_TX_PIN, CP_CHECKALL);

        if(de || b9 || inv || s2) error("COM specification");
        com2_buf_size = bufsize;                                    // extracted from the comspec above
        com2_interrupt = interrupt;
        com2_ilevel   = ilevel;
        com_interrupt &= (~(1 << 2));                               // clear the interrupt flag

        // setup for receive
        com2Rx_start_cnt = (27 + 1);
        com2Rx_buf = GetMemory(com2_buf_size);                      // setup the buffer
        com2Rx_head = com2Rx_tail = 0;
        ExtCfg(COM2_RX_PIN, EXT_DIG_IN, 0);                         // pin is Rx (an input)
        ExtCfg(COM2_RX_PIN, EXT_COM_RESERVED, 0);                   // reserve the pin for com use
        PinSetBit(COM2_RX_PIN, CNPUSET);                            // turn on the pullup for Rx

        // setup for transmit
        com2Tx_start_cnt = 30;                                      // for normal serial
        com2Tx_buf = GetMemory(TX_BUFFER_SIZE);                     // setup the buffer
        com2Tx_head = com2Tx_tail = 0;
        ExtCfg(COM2_TX_PIN, EXT_NOT_CONFIG, 0);
        ExtSet(COM2_TX_PIN, 1);                                     // start with the Tx pin high (ie, idle)
        ExtCfg(COM2_TX_PIN, oc ? EXT_OC_OUT : EXT_DIG_OUT, 0);      // set the Tx pin as an output
        ExtCfg(COM2_TX_PIN, EXT_COM_RESERVED, 0);                   // reserve the pin for com use

        // if timer 5 is off then set it up
        if(T5CON == 0) {
            PR5 = ((BusSpeed/(com2_baud = baud)) / 3) - 1;          // ticks at 3 times the baud rate
            T5CON = 0x8000;                                         // T4 on, prescaler 1:1
            mT5SetIntPriority(6);                                   // a high priority
            mT5ClearIntFlag();                                      // clear interrupt flag
           mT5IntEnable(1);                                         // enable interrupt
       }  

       com2 = true;
    }
#endif
}




/***************************************************************************************************
Close a serial port.
****************************************************************************************************/
void SerialClose(int comnbr) {
/*
  if(comnbr == 1 && com1) {
        INTEnable(INT_SOURCE_UART_RX(UART2), INT_DISABLED);
        INTEnable(INT_SOURCE_UART_TX(UART2), INT_DISABLED);
        UARTEnable(UART2, UART_ENABLE_FLAGS(UART_DISABLE | UART_PERIPHERAL));
        COM1_TX_PPS_CLOSE;
        if(com1_mode & COM1_DE) {
            COM1_DE_PPS_CLOSE;
            ExtCfg(COM1_EN_PIN, EXT_NOT_CONFIG, 0);
        }
      com1 = false;
      com1_interrupt = NULL;
      PinSetBit(COM1_RX_PIN, CNPUCLR);                              // clear the pullup or pulldown on Rx
      PinSetBit(COM1_RX_PIN, CNPDCLR);
      ExtCfg(COM1_RX_PIN, EXT_NOT_CONFIG, 0);
      ExtCfg(COM1_TX_PIN, EXT_NOT_CONFIG, 0);
      FreeMemory(com1Rx_buf);
      FreeMemory(com1Tx_buf);

  }

#ifdef COM2
    if(comnbr == 2 && com2) {
      mT5IntEnable(0);
      T5CON = 0;
      com2 = false;
      com2_interrupt = NULL;
      PinSetBit(COM1_RX_PIN, CNPUCLR);                              // clear the pullup on Rx
      ExtCfg(COM2_RX_PIN, EXT_NOT_CONFIG, 0);
      ExtCfg(COM2_TX_PIN, EXT_NOT_CONFIG, 0);
      FreeMemory(com2Rx_buf);
      FreeMemory(com2Tx_buf);
  }
#endif
      com_interrupt &= (~(1 << comnbr));                            // clear the interrupt flag
*/
}



/***************************************************************************************************
Add a character to the serial output buffer.
****************************************************************************************************/
unsigned char SerialPutchar(int comnbr, unsigned char c)
{
/*
	if(comnbr == 1) {

      while(com1Tx_tail == ((com1Tx_head + 1) % TX_BUFFER_SIZE))    // wait if the buffer is full
          if(MMAbort) {                                             // allow the user to abort a hung serial port
              com1Tx_tail = com1Tx_head = 0;                        // clear the buffer
              longjmp(mark, 1);                                     // and abort
          }
      com1Tx_buf[com1Tx_head] = c;                                  // add the char
      com1Tx_head = (com1Tx_head + 1) % TX_BUFFER_SIZE;             // advance the head of the queue
        INTEnable(INT_SOURCE_UART_TX(UART2), INT_ENABLED);          // enable Tx interrupt in case it was off
  }
    else if(comnbr == 2) {
      while(com2Tx_tail == ((com2Tx_head + 1) % TX_BUFFER_SIZE))    // wait if the buffer is full
          if(MMAbort) {                                             // allow the user to abort a hung serial port
              com2Tx_tail = com2Tx_head = 0;                        // clear the buffer
              longjmp(mark, 1);                                     // and abort
          }
      com2Tx_buf[com2Tx_head] = c;                                  // add the char
      com2Tx_head = (com2Tx_head + 1) % TX_BUFFER_SIZE;             // advance the head of the queue
  }
  */
	  return c;
}



/***************************************************************************************************
Get the status the serial receive buffer.
Returns the number of characters waiting in the buffer
****************************************************************************************************/
int SerialRxStatus(int comnbr) {

	int i = 0;
/*
  if(comnbr == 1) {
      INTEnable(INT_SOURCE_UART_RX(UART2), INT_DISABLED);
      i = com1Rx_head - com1Rx_tail;
      INTEnable(INT_SOURCE_UART_RX(UART2), INT_ENABLED);
      if(i < 0) i += com1_buf_size;
  } else if(comnbr == 2) {
      mT5IntEnable(0);
      i = com2Rx_head - com2Rx_tail;
      mT5IntEnable(1);
      if(i < 0) i += com2_buf_size;
  }
  */
  return i;

}



/***************************************************************************************************
Get the status the serial transmit buffer.
Returns the number of characters waiting in the buffer
****************************************************************************************************/
int SerialTxStatus(int comnbr) {
  int i = 0;
/*
  if(comnbr == 1) {
      i = com1Tx_head - com1Tx_tail;
      if(i < 0) i += TX_BUFFER_SIZE;
        if(i == 0 && !UARTTransmissionHasCompleted(UART2)) i++;     // count a byte in the process of being transmitted
  }
    else if(comnbr == 2) {
      i = com2Tx_head - com2Tx_tail;
      if(i < 0) i += TX_BUFFER_SIZE;
        if(i == 0 && com2Tx_cnt != 0) i++;                          // count a byte in the process of being transmitted
  }
*/
  return i;
}



/***************************************************************************************************
Get a character from the serial receive buffer.
Note that this is returned as an integer and -1 means that there are no characters available
****************************************************************************************************/
// int MIPS16 SerialGetchar(int comnbr) {
int SerialGetchar(int comnbr) {
  int c;
    c = -1;                                                         // -1 is no data
/*  if(comnbr == 1) {
        INTEnable(INT_SOURCE_UART_RX(UART2), INT_DISABLED);
      if(com1Rx_head != com1Rx_tail) {                              // if the queue has something in it
          c = com1Rx_buf[com1Rx_tail];                              // get the char
          com1Rx_tail = (com1Rx_tail + 1) % com1_buf_size;          // and remove from the buffer
      }
        INTEnable(INT_SOURCE_UART_RX(UART2), INT_ENABLED);
  }
    else if(comnbr == 2) {
//        dp("in");
      mT5IntEnable(0);
      if(com2Rx_head != com2Rx_tail) {                              // if the queue has something in it
          c = com2Rx_buf[com2Rx_tail];                              // get the char
          com2Rx_tail = (com2Rx_tail + 1) % com2_buf_size;          // and remove from the buffer
      }
      mT5IntEnable(1);
//        dp("out");
  }
*/
    return c;
}


#ifdef COM2
/****************************************************************************************************************
Timer 5 interrupt processor - This fires at 3 times the baud rate
com1_cnt is used to count the timer ticks while receiving a byte and the bit stream is sampled on every third tick

This code is optimised for speed.  For example, individual integers are used for parameters (eg, com1Rx_cnt)
*****************************************************************************************************************/
//void __ISR( _TIMER_5_VECTOR, ipl6) T5Interrupt( void) {
void T5Interrupt( void) {
  static int com2Rx_byte;                                           // build the received byte here
  static int com2Tx_byte;                                           // hold the transmit byte here

  if(com2) {
      ///////////////////////////////// this is COM2 ////////////////////////////////////
      // this is the receive character routine
      // first check if we are waiting for the start bit (ie, com2Rx_cnt is zero)
      // if so, initialise.  Otherwise we must be receiving a character so clock it in
      if(com2Rx_cnt == 0) {
          if(!PORTAbits.RA3) {
              com2Rx_cnt = com2Rx_start_cnt;                        // got the leading edge of a start bit so initialise
              com2Rx_byte = 0;
          }
      } else {
          // we are in the process of receiving a byte
          com2Rx_cnt--;
          if(com2Rx_cnt % 3 == 0) {                                 // is this the right clock tick?
              if(com2Rx_cnt == 0) {
                  // this should be the stop bit
                  // check that the stop bit is high and that the start bit was low
                  // if both are correct add the byte to the queue
                  if(PORTAbits.RA3 && (com2Rx_byte & 1) == 0) {
                      com2Rx_byte >>= 1;                                    // shift out the start bit (not needed now)
                      com2Rx_buf[com2Rx_head] = com2Rx_byte;                // store the byte in the ring buffer
                      com2Rx_head = (com2Rx_head + 1) % com2_buf_size;      // advance the head of the queue
                      if(com2Rx_head == com2Rx_tail)                        // if the buffer has overflowed throw away the oldest char
                          com2Rx_tail = (com2Rx_tail + 1) % com2_buf_size;
                      if(com2_ilevel != 0) {                                 // do we need to check for an interrupt?
                          if(com2_ilevel < 0) {                              // if we must interrupt on a certain char received...
                              if(com2Rx_byte == -com2_ilevel) com_interrupt |= (1 << 2);
                          } else {                                           // if we must interrupt on the nbr of bytes in the buffer...
                              int i;
                              i = com2Rx_head - com2Rx_tail;
                              if(i < 0) i += com2_buf_size;
                              if(i >= com2_ilevel) com_interrupt |= (1 << 2);
                          }
                      } 
                  }
              } else {
                  // we are receiving the start bit or a data bit - regardless, shift it into our byte
                  com2Rx_byte >>= 1;                                // shift up the data 1 bit
                  if(PORTAbits.RA3) com2Rx_byte |= 0b100000000;     // and insert our new bit
              }
          }
      }

      // this is the transmit character routine
      if(com2Tx_cnt == 0) {                                         // if we are not transmitting a character
          if(com2Tx_head != com2Tx_tail) {                          // if there is something in the buffer
              com2Tx_byte = ((com2Tx_buf[com2Tx_tail] << 1) | 0b111000000000);        // get the char and build the bits to send
              com2Tx_tail = (com2Tx_tail + 1) % TX_BUFFER_SIZE;     // advance the tail of the queue
              com2Tx_cnt = com2Tx_start_cnt;                        // and arm the transmission
          }
      } else {
          // we must be in the process of sending a character
          com2Tx_cnt--;
          if(com2Tx_cnt % 3 == 0) {                                 // is this the right baud rate count?
              if(com2Tx_byte & 1)
                  LATASET = (1 << 2);                               // output the bit hi - this is atomic and is interrupt proof
              else
                  LATACLR = (1 << 2);
              com2Tx_byte >>= 1;                                    // and get the next bit ready
          }
      }
  }

    // Clear the interrupt flag
    mT5ClearIntFlag();
}
#endif


/*
// UART 2 interrupt handler
//void __ISR(_UART2_VECTOR, ipl3) IntUART2Handler(void) {
void IntUART2Handler(void) {
	unsigned short cc;

    if(INTGetFlag(INT_SOURCE_UART_RX(UART2))) {                     // Is this an RX interrupt?
        while(UARTReceivedDataIsAvailable(UART2)) {                 // while there is data to read
            if(UARTGetLineStatus(UART2) & 0b1110) {                 // first check for errors
                UARTGetDataByte(UART2);                             // and if there was an error throw away the char
                U2STACLR = 0b1110;                                  // clear the error on the UART
                continue;                                           // and try the next char
            }
            cc = UARTGetData(UART2).__data;
            if(com1_mode & COM1_9B) {                               // if we are using 9 bits
                com1Rx_buf[com1Rx_head] = (cc >> 8) + '0';          // store the 9th bit in the ring buffer
                com1Rx_head = (com1Rx_head + 1) % com1_buf_size;    // advance the head of the queue
                if(com1Rx_head == com1Rx_tail) {                    // if the buffer has overflowed
                    com1Rx_tail = (com1Rx_tail + 1) % com1_buf_size;// throw away the oldest char
                }
            }
            com1Rx_buf[com1Rx_head] = cc;                           // store the data byte in the ring buffer
            com1Rx_head = (com1Rx_head + 1) % com1_buf_size;        // advance the head of the queue
            if(com1Rx_head == com1Rx_tail) {                        // if the buffer has overflowed
                com1Rx_tail = (com1Rx_tail + 1) % com1_buf_size;    // throw away the oldest char
            }
            if(com1_ilevel != 0) {                                  // do we need to check for an interrupt?
                if(com1_ilevel < 0) {                               // if we must interrupt on a certain char received...
                    if(cc == -com1_ilevel) com_interrupt |= (1 << 1);
                } else {                                            // if we must interrupt on the nbr of bytes in the buffer...
                    int i;
                    i = com1Rx_head - com1Rx_tail;
                    if(i < 0) i += com1_buf_size;
                    if(i >= com1_ilevel) com_interrupt |= (1 << 1);
                }
            } 
        }
        INTClearFlag(INT_SOURCE_UART_RX(UART2));                    // Clear the RX interrupt Flag
    }

    if(INTGetFlag(INT_SOURCE_UART_TX(UART2))) {                     // Is this an Tx interrupt?
        while(UARTTransmitterIsReady(UART2) && com1Tx_tail != com1Tx_head) { // while Tx is free and there is data to send
            cc = com1Tx_buf[com1Tx_tail];
            com1Tx_tail = (com1Tx_tail + 1) % TX_BUFFER_SIZE;       // advance the tail of the queue
            if(com1_mode & COM1_9B) {                               // if we are using 9 bits
                if(com1_bit9 == 0) {
                    com1_bit9 = cc;
                    continue;
                } else {
                    cc |= (com1_bit9 == '1') << 8;
                    com1_bit9 = 0;
                }
            }
            INTDisableInterrupts();                                 // see Errata #10
            UARTSendData(UART2, (UART_DATA)cc);                     // send the byte
            INTEnableInterrupts();
        }
        if(com1Tx_tail == com1Tx_head)                              // if there is nothing left to send
            INTEnable(INT_SOURCE_UART_TX(UART2), INT_DISABLED);     // disable the interrupt
        INTClearFlag(INT_SOURCE_UART_TX(UART2));                    // Clear the Tx interrupt Flag
    }
}
*/

