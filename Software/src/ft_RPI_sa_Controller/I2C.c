/************************************************************************************************************************
fischertechnik-Raspberry Pi-Stand-Alone Controller

I2C.c

Routines to handle I2C access.

Copyright 2011 Gerard Sexton
Copyright 2011-2023 Geoff Graham and  Peter Mather.
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

*******************************************************************************/

#include "Version.h"


#define I2C_10BitAddr_Mask            0x78

// Declare functions
void i2cEnable(char *p);
void i2cDisable(char *p);
void i2cSend(char *p);
void i2cReceive(char *p);
void i2c_disable(void);
void i2c_enable(int bps);
void i2c_masterCommand(int timer);

volatile unsigned int I2C_State;                                    // the state of the master state machine
volatile unsigned int I2C_Status;                                   // status flags
unsigned int I2C_Timer;                                             // master timeout counter
char *I2C_IntLine;                                                  // pointer to the master interrupt line number

static MMFLOAT *I2C_Rcvbuf_Float;                                   // pointer to the master receive buffer for a MMFLOAT
static long long int *I2C_Rcvbuf_Int;                               // pointer to the master receive buffer for an integer
static char *I2C_Rcvbuf_String;                                     // pointer to the master receive buffer for a string
static unsigned int I2C_Addr;                                       // I2C device address
static unsigned int I2C_Prev_Addr = 0xffff;                         // previous I2C device address
static unsigned int I2C_Timeout;                                    // master timeout value
static volatile unsigned int I2C_Sendlen;                           // length of the master send buffer
static volatile unsigned int I2C_Rcvlen;                            // length of the master receive buffer
static volatile unsigned int I2C_Send_Index;                        // current index into I2C_Send_Buffer
static volatile unsigned int I2C_Rcv_Head;                          // Receive buffer head
static volatile unsigned int I2C_Rcv_Tail;                          // Receive buffer tail
static char *I2C_Send_Buffer = NULL;                                // I2C send buffer

static unsigned int I2C_Slave_Mask;                                 // slave address mask
static unsigned int I2C_Slave_Addr;                                 // slave address
static volatile unsigned int I2C_Slave_Sendlen;                     // length of the slave send buffer

// defines for MM.I2C
#define I2C_MMI2C_NoAck           1
#define I2C_MMI2C_Timeout         2
#define I2C_MMI2C_GeneralCall     4
static int mmI2Cvalue;                                              // value of MM.I2C

/*******************************************************************************************
                            I2C related commands in MMBasic
                              ===============================
These are the functions responsible for executing the I2C related commands in MMBasic
They are supported by utility functions that are grouped at the end of this file

********************************************************************************************/


void cmd_i2c(void) {
    char *p;

    if((p = checkstring(cmdline, "OPEN")) != NULL)
        i2cEnable(p);
    else if((p = checkstring(cmdline, "CLOSE")) != NULL)
        i2cDisable(p);
    else if((p = checkstring(cmdline, "WRITE")) != NULL)
        i2cSend(p);
    else if((p = checkstring(cmdline, "READ")) != NULL)
        i2cReceive(p);
    else
        error("Unknown command");
}



/****************************************************************************************************************************
Real Time Clock routines
*****************************************************************************************************************************/

// universal function to send/receive data to/from the RTC
// addr is the I2C address WITHOUT the read/write bit
int DoI2C(int addr) {
    I2C_Addr = addr;                                                // address of the device
    i2c_masterCommand(100);
    while (!(I2C_Status & I2C_Status_Completed)) { CheckAbort(); }
    I2C_Status &= ~I2C_Status_Completed;
    return !mmI2Cvalue;
}


char CvtToBCD(char *p, int min, int max) {
    long long int t;
    t = getint(p, min, max) % 100;
    return ((t / 10) << 4) | (t % 10);
}


char CvtCharsToBCD(char *p, int min, int max) {
    int t;
    t = (p[0] - '0') * 10 + (p[1] - '0');
//    dp("|%c|  |%c|  %d   %d   %d", p[0], p[1], t, min, max);
    if(!isdigit((int)p[0]) || !isdigit((int)p[1]) || t < min || t > max) error("Date/time format");
    return ((t / 10) << 4) | (t % 10);
}


static int CloseI2C = false;

void SetupI2C(void) {
    if(I2C_Send_Buffer == NULL) {
        I2C_Timeout = 5;
        i2c_enable(100);                                            // initialise the I2C interface
        //PinSetBit(P_I2C_SDA, CNPUSET);
        //PinSetBit(P_I2C_SCL, CNPUSET);
        CloseI2C =  true;
    } else
        CloseI2C = false;
}


void RtcGetTime(void) {
    char buff[7];                                                   // Received data is stored here

    SetupI2C();
    I2C_Sendlen = 1;                                                // send one byte
    I2C_Rcvlen = 0;
    *I2C_Send_Buffer = 3;                                           // the first register to read
    if(!DoI2C(RTC_Addr)) goto error_exit;
    I2C_Rcvbuf_String = buff;                                       // we want a string of bytes
    I2C_Rcvbuf_Float = NULL;
    I2C_Rcvbuf_Int = NULL;
    I2C_Rcvlen = 7;                                                 // get 7 bytes
    I2C_Sendlen = 0;
    if(!DoI2C(RTC_Addr)) goto error_exit;

    second = ((buff[0] & 0x7f) >> 4) * 10 + (buff[0] & 0x0f);
    minute = ((buff[1] & 0x7f) >> 4) * 10 + (buff[1] & 0x0f);
    hour = ((buff[2] & 0x3f) >> 4) * 10 + (buff[2] & 0x0f);
    day = ((buff[3] & 0x3f) >> 4) * 10 + (buff[3] & 0x0f);
    month = ((buff[5] & 0x1f) >> 4) * 10 + (buff[5] & 0x0f);
    year = (buff[6] >> 4) * 10 + (buff[6] & 0x0f) + 2000;
    if(CloseI2C) i2c_disable();
    return;

error_exit:
	if(CloseI2C) i2c_disable();
    if(CurrentLinePtr) error("RTC not responding");
    MMPrintString("RTC not responding");
    MMPrintString("\r\n");
}

void cmd_rtc(void) {
    char buff[8], AlrmEnable, ValueToWrite;                                                   // Received data is stored here
    short RamAddress;
    char *p;
    void *ptr = NULL;

    if(checkstring(cmdline, "GETTIME")) {
        RtcGetTime();
        return;
    }

    SetupI2C();                                                  // RtcGetTime() does not need this but the rest of this function does

    if((p = checkstring(cmdline, "SETTIME")) != NULL) {
		getargs(&p, 11, ",");
		// multiple arguments - data should be in the original yy, mm, dd, etc format
		if(argc != 11) error("Argument count");
		I2C_Send_Buffer[1] = CvtToBCD(argv[10], 0, 59);         // seconds
		I2C_Send_Buffer[2] = CvtToBCD(argv[8], 0, 59);          // minutes
		I2C_Send_Buffer[3] = CvtToBCD(argv[6], 0, 23);          // hour
		I2C_Send_Buffer[4] = CvtToBCD(argv[4], 1, 31);          // day
		I2C_Send_Buffer[6] = CvtToBCD(argv[2], 1, 12);          // month
		I2C_Send_Buffer[7] = CvtToBCD(argv[0], 0, 2099);           // year
		I2C_Send_Buffer[0] = 3;                                     // address to write to
		I2C_Send_Buffer[5] = 0;                                                                // weekdays not used
		I2C_Rcvlen = 0;
		I2C_Sendlen = 8;                                            // send 7 bytes
		if(!DoI2C(RTC_Addr)) error("RTC not responding");
		RtcGetTime();
    } else if((p = checkstring(cmdline, "SETALARM")) != NULL) {
		// multiple arguments - data should be in the original dd, hrs, min, sec format
        if(checkstring(p, "OFF") != NULL)                                // switch alarm off?
                     I2C_Send_Buffer[1] = 0x00;                                  // clear alarm enable bit
        else
		{
			getargs(&p, 9, ",");
			if(argc != 9) error("Argument count");
			AlrmEnable = getint(argv[8],1,15);
			I2C_Send_Buffer[1] = CvtToBCD(argv[6], 0, 59)|(0x80-((AlrmEnable&0x1)<<7));          // seconds
			I2C_Send_Buffer[2] = CvtToBCD(argv[4], 0, 59)|(0x80-((AlrmEnable&0x2)<<6));          // minutes
			I2C_Send_Buffer[3] = CvtToBCD(argv[2], 0, 23)|(0x80-((AlrmEnable&0x4)<<5));          // hour
			I2C_Send_Buffer[4] = CvtToBCD(argv[0], 1, 31)|(0x80-((AlrmEnable&0x8)<<4));          // day
			I2C_Send_Buffer[0] = 0x0a;                              // address to write to
			I2C_Send_Buffer[5] = 0x80;                                                          // weekdays not used
			I2C_Rcvlen = 0;
			I2C_Sendlen = 6;                                        // send 6 bytes (address + data)
			if(!DoI2C(RTC_Addr)) error("RTC not responding");
			I2C_Send_Buffer[1] = 0x02;                                                // set AIE flag to get an interrupt from RTC
		}
		I2C_Send_Buffer[0] = 0x01;                              // address to write to
        I2C_Rcvlen = 0;
        I2C_Sendlen = 2;                                        // send 2 bytes (address + data)
        if(!DoI2C(RTC_Addr)) error("RTC not responding");
    } else if((p = checkstring(cmdline, "CLEARALARM")) != NULL) {
        I2C_Send_Buffer[0] = 0x01;                              // address to write to
        I2C_Send_Buffer[1] = 0x02;                                                      // clear alarm flag
        I2C_Rcvlen = 0;
        I2C_Sendlen = 2;                                            // send 6 bytes (address + data)
        if(!DoI2C(RTC_Addr)) error("RTC not responding");
    } else if((p = checkstring(cmdline, "GETREG")) != NULL) {
        getargs(&p, 3, ",");
        if(argc != 3) error("Argument count");
        I2C_Sendlen = 1;                                            // send one byte
        I2C_Rcvlen = 0;
        *I2C_Send_Buffer = getint(argv[0], 0, 0x1d);                 // the register to read
        ptr = findvar(argv[2], V_FIND);
        if(vartbl[VarIndex].type & T_CONST) error("Cannot change a constant");
        if(vartbl[VarIndex].type & T_STR)  error("Invalid variable");

        if(!DoI2C(RTC_Addr)) error("RTC not responding");
        I2C_Rcvbuf_String = buff;                                   // we want a string of bytes
        I2C_Rcvbuf_Float = NULL;
        I2C_Rcvbuf_Int = NULL;
        I2C_Rcvlen = 1;                                             // get 1 byte
        I2C_Sendlen = 0;
        if(!DoI2C(RTC_Addr)) error("RTC not responding1");
        if(vartbl[VarIndex].type & T_NBR)
            *(MMFLOAT *)ptr = buff[0];
        else
            *(long long int *)ptr = buff[0];
    } else if((p = checkstring(cmdline, "SETREG")) != NULL) {
        getargs(&p, 3, ",");
        if(argc != 3) error("Argument count");
        I2C_Rcvlen = 0;
        I2C_Send_Buffer[0] = getint(argv[0], 0, 255);               // set the register pointer
        I2C_Send_Buffer[1] = getint(argv[2], 0, 255);               // and the data to be written
       I2C_Sendlen = 2;                                            // send 2 bytes
        if(!DoI2C(RTC_Addr)) error("RTC not responding");
    } else if((p = checkstring(cmdline, "GETRAM")) != NULL) {
        getargs(&p, 3, ",");
        if(argc != 3) error("Argument count");
        RamAddress = getint(argv[0],0,511);
        ptr = findvar(argv[2], V_FIND);
        if(vartbl[VarIndex].type & T_CONST) error("Cannot change a constant");
        if(vartbl[VarIndex].type & T_STR)  error("Invalid variable");

        I2C_Sendlen = 3;                            	             // send RAM address to read from
        I2C_Rcvlen = 0;
        I2C_Send_Buffer[0] = 0x1a;					                 // the register to write to
        I2C_Send_Buffer[1] = RamAddress/256;		                 // set upper address
        I2C_Send_Buffer[2] = RamAddress%256;		                 // set lower address
        if(!DoI2C(RTC_Addr)) error("RTC not responding");
        I2C_Sendlen = 1;                            	             // send RAM address to read from
        I2C_Rcvlen = 1;
        *I2C_Send_Buffer = 0x1d;					                 // the register to read from
        I2C_Rcvbuf_String = buff;                                   // we want a string of bytes
        I2C_Rcvbuf_Float = NULL;
        I2C_Rcvbuf_Int = NULL;
        if(!DoI2C(RTC_Addr)) error("RTC not responding1");
        if(vartbl[VarIndex].type & T_NBR)
            *(MMFLOAT *)ptr = buff[0];								// read the second data byte which is the RAM content
        else
            *(long long int *)ptr = buff[0];
    } else if((p = checkstring(cmdline, "SETRAM")) != NULL) {
        getargs(&p, 3, ",");
        if(argc != 3) error("Argument count");
        RamAddress = getint(argv[0],0,511);
        ValueToWrite = getint(argv[2],0,255);

        I2C_Sendlen = 3;                            	             // send RAM address to read from
        I2C_Rcvlen = 0;
        I2C_Send_Buffer[0] = 0x1a;					                 // the register to write to
        I2C_Send_Buffer[1] = RamAddress/256;		                 // set upper address
        I2C_Send_Buffer[2] = RamAddress%256;		                 // set lower address
        if(!DoI2C(RTC_Addr)) error("RTC not responding");
        I2C_Sendlen = 2;                            	             // send RAM address to read from
        I2C_Rcvlen = 0;
        I2C_Send_Buffer[0] = 0x1c;					                 // the register to read from
        I2C_Send_Buffer[1] = ValueToWrite;			                 // the data to write
        I2C_Rcvbuf_String = buff;                                   // we want a string of bytes
        I2C_Rcvbuf_Float = NULL;
        I2C_Rcvbuf_Int = NULL;
        if(!DoI2C(RTC_Addr)) error("RTC not responding1");
    } else
        error("Unknown command");

    if(CloseI2C) i2c_disable();
}

/**************************************************************************************************/



// enable the I2C1 module - master mode
void i2cEnable(char *p) {
    int speed, timeout;
    if(I2C_Status & I2C_Status_Enabled) error("I2C is already open");
    getargs(&p, 5, ",");
    if(!(argc == 2 || argc == 3)) error("Argument count");
    speed = getint(argv[0], 10, 400);
    timeout = getinteger(argv[2]);
    if(timeout < 0 || (timeout > 0 && timeout < 100)) error("Number out of bounds" );
    I2C_Timeout = timeout;
    if(I2C_Status & I2C_Status_InProgress) error("I2C is busy");
    i2c_enable(speed);
}


// disable the I2C1 module - master mode
void i2cDisable(char *p) {
    checkend(p);
    i2c_disable();
}


// send data to an I2C slave - master mode
void i2cSend(char *p) {
    int addr, i2c_options, sendlen, i;
    void *ptr = NULL;
    unsigned char *cptr = NULL;

    getargs(&p, 99, ",");
    if(!(argc & 0x01) || (argc < 7)) error("Argument count");
    if(!(I2C_Status & I2C_Status_Master)) error("Not enabled");
    if(I2C_Status & I2C_Status_InProgress) error("I2C is busy");
    addr = getinteger(argv[0]);
    if (addr==PT_Addr || addr==HB_Address_HB1 || addr==HB_Address_HB2) error("Device not allowed to be accessed directly");
    i2c_options = getint(argv[2], 0, 3);
    if(i2c_options & 0x01) I2C_Status |= I2C_Status_BusHold;
    else I2C_Status &= ~I2C_Status_BusHold;
    if(i2c_options & 0x02) {
        I2C_Status |= I2C_Status_10BitAddr;
        if(addr < 0x0000 || addr > 0x03ff) error("Invalid address");
    } else {
        I2C_Status &= ~I2C_Status_10BitAddr;
        if(addr < 0x00 || (addr > 0x00 && addr > 0x77)) error("Invalid address");
    }
    if(I2C_Status & I2C_Status_Slave && ((addr | I2C_Slave_Mask) == (I2C_Slave_Addr | I2C_Slave_Mask)))
        error("Invalid address");
    I2C_Addr = addr;
    sendlen = getint(argv[4], 1, 255);

    if(sendlen == 1 || argc > 7) {      // numeric expressions for data
        if(sendlen != ((argc - 5) >> 1)) error("Argument count");
        for (i = 0; i < sendlen; i++) {
            I2C_Send_Buffer[i] = getinteger(argv[i + i + 6]);
        }
    } else {        // an array of MMFLOAT, integer or a string
        ptr = findvar(argv[6], V_EMPTY_OK | V_NOFIND_ERR);
        if((vartbl[VarIndex].type & T_STR) && vartbl[VarIndex].dims[0] == 0) {      // string
            cptr = (unsigned char *)ptr;
            cptr++;                                                                 // skip the length byte in a MMBasic string
            for (i = 0; i < sendlen; i++) {
                I2C_Send_Buffer[i] = (int)(*(cptr + i));
            }
        } else if((vartbl[VarIndex].type & T_NBR) && vartbl[VarIndex].dims[0] > 0 && vartbl[VarIndex].dims[1] == 0) {   // numeric array
            if( (((MMFLOAT *)ptr - vartbl[VarIndex].val.fa) + sendlen) > (vartbl[VarIndex].dims[0] + 1 - OptionBase) ) {
                error("Insufficient data");
            } else {
                for (i = 0; i < sendlen; i++) {
                    I2C_Send_Buffer[i] = (int)(*((MMFLOAT *)ptr + i));
                }
            }
        } else if((vartbl[VarIndex].type & T_INT) && vartbl[VarIndex].dims[0] > 0 && vartbl[VarIndex].dims[1] == 0) {   // integer array
            if( (((long long int *)ptr - vartbl[VarIndex].val.ia) + sendlen) > (vartbl[VarIndex].dims[0] + 1 - OptionBase) ) {
                error("Insufficient data");
            } else {
                for (i = 0; i < sendlen; i++) {
                    I2C_Send_Buffer[i] = (int)(*((long long int *)ptr + i));
                }
            }
        } else error("Invalid variable");
    }
    I2C_Sendlen = sendlen;
    I2C_Rcvlen = 0;

    i2c_masterCommand(5);
    while (!(I2C_Status & I2C_Status_Completed)) { CheckAbort(); }
    I2C_Status &= ~I2C_Status_Completed;
}


// receive data from an I2C slave - master mode
void i2cReceive(char *p) {
    int addr, i2c_options, rcvlen;
    void *ptr = NULL;

    getargs(&p, 7, ",");
    if(argc != 7) error("Argument count");
    if(!(I2C_Status & I2C_Status_Master)) error("Not enabled");
    if(I2C_Status & I2C_Status_InProgress) error("I2C is busy");
    addr = getinteger(argv[0]);
    i2c_options = getint(argv[2], 0, 3);
    if(i2c_options & 0x01) I2C_Status |= I2C_Status_BusHold;
    else I2C_Status &= ~I2C_Status_BusHold;
    if(i2c_options & 0x02) {
        I2C_Status |= I2C_Status_10BitAddr;
        if(addr < 0x0000 || addr > 0x03ff) error("Invalid address");
    } else {
        I2C_Status &= ~I2C_Status_10BitAddr;
        if(addr < 0x01 || (addr > 0x00 && addr > 0x77)) error("Invalid address");
    }
    if(I2C_Status & I2C_Status_Slave && ((addr | I2C_Slave_Mask) == (I2C_Slave_Addr | I2C_Slave_Mask)))
        error("Invalid address");
    I2C_Addr = addr;
    rcvlen = getint(argv[4], 1, 255);

    ptr = findvar(argv[6], V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
    if(vartbl[VarIndex].type & T_CONST) error("Cannot change a constant");
    if(vartbl[VarIndex].type & T_NBR) {
          if(vartbl[VarIndex].dims[1] != 0) error("Invalid variable");
          if(vartbl[VarIndex].dims[0] <= 0) {   // Not an array
              if(rcvlen != 1) error("Invalid variable");
          } else {      // An array
              if( (((MMFLOAT *)ptr - vartbl[VarIndex].val.fa) + rcvlen) > (vartbl[VarIndex].dims[0] + 1 - OptionBase) )
                  error("Insufficient space in array");
          }
          I2C_Rcvbuf_Float = (MMFLOAT*)ptr;
      } else if(vartbl[VarIndex].type & T_INT) {
          if(vartbl[VarIndex].dims[1] != 0) error("Invalid variable");
          if(vartbl[VarIndex].dims[0] <= 0) {   // Not an array
              if(rcvlen != 1) error("Invalid variable");
          } else {      // An array
              if( (((long long int *)ptr - vartbl[VarIndex].val.ia) + rcvlen) > (vartbl[VarIndex].dims[0] + 1 - OptionBase) )
                  error("Insufficient space in array");
          }
          I2C_Rcvbuf_Int = (long long int *)ptr;
          I2C_Rcvbuf_Float = NULL;
      } else if(vartbl[VarIndex].type & T_STR) {
          if(vartbl[VarIndex].dims[0] != 0) error("Invalid variable");
          *(char *)ptr = rcvlen;
          I2C_Rcvbuf_String = (char *)ptr + 1;
          I2C_Rcvbuf_Float = NULL;
          I2C_Rcvbuf_Int = NULL;
      } else
          error("Invalid variable");
    I2C_Rcvlen = rcvlen;

    I2C_Sendlen = 0;

    i2c_masterCommand(5);
    while (!(I2C_Status & I2C_Status_Completed)) { CheckAbort(); }
    I2C_Status &= ~I2C_Status_Completed;
}

void fun_mmi2c(void) {
    iret = mmI2Cvalue;
    targ = T_INT;
}

/**************************************************************************************************
I2C0 interrupt.
Used to process the I2C requests
***************************************************************************************************/
void LPI2C0_Master_IRQHandler(void) {

    if(I2C_Status & I2C_Status_Timeout) { // Timeout interrupt
    	// This part will be entered only if IRQ has been triggered in the timers.c module to signal a timeout
    	// Therefore, we need to clear this manual set IRQ also manually in the end
        I2C_Status &= ~(I2C_Status_Timeout | I2C_Status_NoAck | I2C_Status_InProgress | I2C_Status_BusOwned |  I2C_Status_BusHold |
                                        I2C_Status_10BitAddr | I2C_Status_Send | I2C_Status_Receive | I2C_Status_MasterCmd);
    	IP_LPI2C0->MSR = I2C_ERROR_MASK;			// clear all potential error flags
    	IP_LPI2C0->MCR |= LPI2C_MCR_RTF_MASK|LPI2C_MCR_RRF_MASK;	// reset FIFOs
        IP_LPI2C0->MIER &= ~LPI2C_MIER_TDIE_MASK;					// Disable transmit data interrupt to stop I2C traffic
        if(!(I2C_Status & I2C_Status_Disable)) {
            I2C_State = I2C_State_Idle;
            I2C_Send_Index = 0;
            I2C_Prev_Addr = 0xffff;
            mmI2Cvalue = I2C_MMI2C_Timeout;
            I2C_Status |= I2C_Status_Completed;
        }
        S32_NVIC->ICPR[(LPI2C0_Master_IRQn>>5)] = (uint32_t)(1U << ((uint32_t)(LPI2C0_Master_IRQn) & (uint32_t)0x1FU)); // clear interrupt
        // previous command need to clear interrupt in the NVIC as this has been set manually (not by I2C module) in the
        // timers.c part to indicate a timeout on this module and request IRQ w/o module being involved
    }

    if(IP_LPI2C0->MSR & LPI2C_MSR_ALF_MASK) {                          // Arbitration lost/Bus collision detected
    	IP_LPI2C0->MSR = LPI2C_MSR_ALF_MASK;
        IP_LPI2C0->MIER &= ~LPI2C_MIER_TDIE_MASK;					// Disable transmit data interrupt to stop I2C traffic
        I2C_Status &= ~(I2C_Status_InProgress | I2C_Status_BusOwned | I2C_Status_Send | I2C_Status_Receive);
        I2C_State = I2C_State_Idle;
        I2C_Prev_Addr = 0xffff;
        if(I2C_Status & I2C_Status_Disable) {
            I2C_Status |= I2C_Status_Completed;
        } else {
            i2c_masterCommand(0);
        }
    }

    // Unexpected NACK received - device not responing
	if(IP_LPI2C0->MSR & LPI2C_MSR_NDF_MASK) {
		I2C_State = I2C_State_Stop;
	}

	switch (I2C_State) {                                        // master state machine
		case I2C_State_Start:                                   // 1
			if(I2C_Status & I2C_Status_Send) {                  // send to I2C device
				I2C_Status |= I2C_Status_BusOwned;
				if(I2C_Status & I2C_Status_10BitAddr) {         // 10 bit addressing
					I2C_State = I2C_State_10Bit;
					IP_LPI2C0->MTDR = ((I2C_10BitAddr_Mask + (I2C_Addr >> 8)) << 1)|LPI2C_MTDR_CMD(0x4); // Generate (rep) START and transmit address
				} else {    // 7 bit addressing
					I2C_Prev_Addr = 0xffff;
					I2C_State = I2C_State_Send;
					IP_LPI2C0->MTDR = (I2C_Addr << 1)|LPI2C_MTDR_CMD(0x4); // Generate (rep) START and transmit address
				}
			} else {                                            // receive from I2C device
				if(I2C_Status & I2C_Status_10BitAddr) {         // 10 bit addressing
					if(I2C_Status & I2C_Status_BusOwned && I2C_Addr == I2C_Prev_Addr) {
						I2C_State = I2C_State_RcvAddr;
						IP_LPI2C0->MTDR = (((I2C_10BitAddr_Mask + (I2C_Addr >> 8)) << 1) + 1)|LPI2C_MTDR_CMD(0x4); // Generate (rep) START and transmit address
					} else {
						I2C_Status |= I2C_Status_BusOwned;
						I2C_State = I2C_State_10Bit;
						IP_LPI2C0->MTDR = ((I2C_10BitAddr_Mask + (I2C_Addr >> 8)) << 1)|LPI2C_MTDR_CMD(0x4);		// Generate (rep) START and transmit address
					}
				} else {    // 7 bit addressing
					I2C_Status |= I2C_Status_BusOwned;
					I2C_State = I2C_State_RcvAddr;
					IP_LPI2C0->MTDR = ((I2C_Addr << 1) + 1)|LPI2C_MTDR_CMD(0x4);									// Generate (rep) START and transmit address
				}
			}
			break;

		case I2C_State_10Bit:   // 2
				I2C_Prev_Addr = I2C_Addr;
				if(I2C_Status & I2C_Status_Send) {
					I2C_State = I2C_State_Send;
					IP_LPI2C0->MTDR = (I2C_Addr & 0xff)|LPI2C_MTDR_CMD(0x0); // transmit
				} else {
					I2C_State = I2C_State_10BitRcv;
					IP_LPI2C0->MTDR = (I2C_Addr & 0xff)|LPI2C_MTDR_CMD(0x0); // transmit
				}
			break;

		case I2C_State_10BitRcv:   // 3
				I2C_State = I2C_State_Start;
				// repeated START will be covered in I2C_State_Start
				//I2C1CONSET = _I2C1CON_RSEN_MASK;
			break;

		case I2C_State_RcvAddr:   // 4
				I2C_State = I2C_State_Receive;
				IP_LPI2C0->MIER &= ~LPI2C_MIER_TDIE_MASK;	// Disable transmit data interrupt as we receive here
				IP_LPI2C0->MTDR = (I2C_Rcvlen-1)|LPI2C_MTDR_CMD(0x01);	// request delta to be received
				IP_LPI2C0->MIER &= ~LPI2C_MIER_TDIE_MASK;		// Disable transmit data interrupt as we receive now

			break;

		case I2C_State_Send:        // 5
				if(I2C_Send_Index < I2C_Sendlen) {
						IP_LPI2C0->MTDR = I2C_Send_Buffer[I2C_Send_Index++]|LPI2C_MTDR_CMD(0x0); // transmit
				} else {
					I2C_Status &= ~I2C_Status_Send;
					I2C_Send_Index = 0;
					if(I2C_Status & I2C_Status_Receive) {
						I2C_State = I2C_State_Start;
					} else {
						if(I2C_Status & I2C_Status_BusHold) {
							I2C_State = I2C_State_Idle;
							I2C_Timer = 0;
							I2C_Status &= ~(I2C_Status_NoAck | I2C_Status_Timeout | I2C_Status_InProgress | I2C_Status_Send | I2C_Status_Receive);
							I2C_Status |= I2C_Status_Completed;
						} else {
							I2C_State = I2C_State_Stop;
							IP_LPI2C0->MTDR = LPI2C_MTDR_CMD(0x2);	// generate STOP condition
						}
					}
			}
			break;

		case I2C_State_Receive:   // 6
			  if(I2C_Rcvbuf_Float != NULL)
				  I2C_Rcvbuf_Float[I2C_Send_Index++] = (IP_LPI2C0->MRDR)&0xff;	// read received data
			  else if(I2C_Rcvbuf_Int != NULL)
				  I2C_Rcvbuf_Int[I2C_Send_Index++] = (IP_LPI2C0->MRDR)&0xff;
			  else
				  I2C_Rcvbuf_String[I2C_Send_Index++] = (IP_LPI2C0->MRDR)&0xff;

			if(I2C_Send_Index < I2C_Rcvlen)
				I2C_State = I2C_State_Receive;
			else
			{
				if(I2C_Status & I2C_Status_BusHold) {
					I2C_State = I2C_State_Idle;
					I2C_Timer = 0;
					I2C_Status &= ~(I2C_Status_NoAck | I2C_Status_Timeout | I2C_Status_InProgress | I2C_Status_Send | I2C_Status_Receive);
					I2C_Status |= I2C_Status_Completed;
				} else {
					I2C_State = I2C_State_Stop;
					IP_LPI2C0->MTDR = LPI2C_MTDR_CMD(0x2);	// generate STOP condition
				}
			}
			break;
		case I2C_State_Stop:        // 8
			I2C_Timer = 0;
			I2C_Send_Index = 0;
			I2C_Slave_Sendlen = 0;
			I2C_Rcv_Tail = I2C_Rcv_Head;
			I2C_State = I2C_State_Idle;
			if(IP_LPI2C0->MSR & LPI2C_MSR_NDF_MASK) {
				mmI2Cvalue = I2C_MMI2C_NoAck;
			}
			I2C_Status &= ~(I2C_Status_NoAck | I2C_Status_Timeout | I2C_Status_BusOwned | I2C_Status_InProgress |
											I2C_Status_10BitAddr | I2C_Status_Send | I2C_Status_Receive | I2C_Status_Slave_Send |
											I2C_Status_Slave_Receive | I2C_Status_Slave_Send_Rdy | I2C_Status_Slave_Receive_Rdy | I2C_Status_Slave_Receive_Full);
			I2C_Prev_Addr = 0xffff;
			I2C_Status |= I2C_Status_Completed;

			IP_LPI2C0->MIER &= ~LPI2C_MIER_TDIE_MASK;					// Disable transmit data interrupt to stop I2C traffic
			IP_LPI2C0->MSR = I2C_ERROR_MASK;							// clear all potential error flags
			IP_LPI2C0->MCR |= LPI2C_MCR_RTF_MASK|LPI2C_MCR_RRF_MASK;	// reset FIFOs

			break;
	}
}

/**************************************************************************************************
Enable the I2C1 module - master mode
***************************************************************************************************/
void i2c_enable(int bps) {
    int CLOCKHI, CLOCKLO, PRESCALER;

    PRESCALER=0;
    do {
        CLOCKLO = (0x0b * (400/bps) + (400/bps-1)*2)/(1<<PRESCALER);
        CLOCKHI = (0x05 * (400/bps) + (400/bps-1)*2)/(1<<PRESCALER);
        PRESCALER++;
    }
    while (CLOCKLO>63 || CLOCKHI>63);
    // Initialization of I2C
    IP_PCC->PCCn[PCC_LPI2C0_INDEX] = 0;                    // Disable clock to change PCS
    IP_PCC->PCCn[PCC_LPI2C0_INDEX] |= PCC_PCCn_PCS(1);        // PCS = 3: Select SOSCDIV2 (8MHz)
    IP_PCC->PCCn[PCC_LPI2C0_INDEX] |= PCC_PCCn_CGC_MASK;    // Enable clock LPI2C0

    IP_LPI2C0->MCR = LPI2C_MCR_RST_MASK;
    IP_LPI2C0->MCR = 0;
    IP_LPI2C0->MCCR0 = LPI2C_MCCR0_DATAVD(0x02)|LPI2C_MCCR0_SETHOLD(0x04)|LPI2C_MCCR0_CLKHI(CLOCKHI)|LPI2C_MCCR0_CLKLO(CLOCKLO);

    IP_LPI2C0->MFCR = LPI2C_MFCR_RXWATER(0)|LPI2C_MFCR_TXWATER(0);
    IP_LPI2C0->MCFGR1 = LPI2C_MCFGR1_PRESCALE(PRESCALER-1)|LPI2C_MCFGR1_IGNACK(0);
    IP_LPI2C0->MCFGR2 = LPI2C_MCFGR2_FILTSDA(0x00)|LPI2C_MCFGR2_FILTSCL(0x00)|LPI2C_MCFGR2_BUSIDLE(0);

	if(I2C_Send_Buffer == NULL) I2C_Send_Buffer = GetMemory(255);
    if(!(I2C_Status & I2C_Status_Master)) {
    	IP_LPI2C0->MSR = I2C_ERROR_MASK;			// clear all potential error flags
    	IP_LPI2C0->MIER = LPI2C_MIER_RDIE_MASK|LPI2C_MIER_ALIE_MASK|LPI2C_MIER_SDIE_MASK;	// generate interrupts
    			// once we have data received, we loose arbitration(bus collision) or STOP has been received
        I2C_Status |= I2C_Status_Master;
    }
    if(!(I2C_Status & I2C_Status_Enabled)) {
        I2C_Status |= I2C_Status_Enabled;
    	IP_LPI2C0->MCR = LPI2C_MCR_MEN_MASK|LPI2C_MCR_RRF(1)|LPI2C_MCR_RTF(1);	// enable LPI2C0 and reset FIFOs
    }
    I2C_Rcv_Tail = I2C_Rcv_Head = 0;                                // reset receive buffer pointers
    I2C_Send_Index = 0;                                             // current index into I2C_Send_Buffer
}


/**************************************************************************************************
Disable the I2C1 module - master mode
***************************************************************************************************/
void i2c_disable() {
    FreeMemory(I2C_Send_Buffer);
    I2C_Send_Buffer = NULL;

    I2C_Status |= I2C_Status_Disable;
    I2C_Status &= ~(I2C_Status_Interrupt | I2C_Status_MasterCmd);
    I2C_IntLine = NULL;
    I2C_Timer = 0;
    if((I2C_Status & I2C_Status_BusOwned) && !(I2C_Status & I2C_Status_InProgress)) { // send stop if required
        I2C_State = I2C_State_Stop;
        IP_LPI2C0->MTDR = LPI2C_MTDR_CMD(0x2);		// Generate STOP condition (cmd=010b)
        while (!(I2C_Status & I2C_Status_Completed)) {}
    }
    I2C_Status = 0;			                                        // clear master status flags
    I2C_Rcvbuf_String = NULL;                                       // pointer to the master receive buffer
    I2C_Rcvbuf_Float = NULL;
    I2C_Rcvbuf_Int = NULL;
    I2C_Sendlen = 0;                                                // length of the master send buffer
    I2C_Rcvlen = 0;                                                 // length of the master receive buffer
    I2C_Addr = 0;                                                   // I2C device address
    I2C_Prev_Addr = 0xffff;                                         // previous I2C address
    I2C_Timeout = 0;                                                // master timeout value
    I2C_State = 0;                                                  // the state of the master state machine
    I2C_Send_Index = 0;                                             // current index into I2C_Send_Buffer
    if (IP_PCC->PCCn[PCC_LPI2C0_INDEX] & PCC_PCCn_CGC_MASK)			// Check whether clock for I2C is enabled
    {
    	IP_LPI2C0->MCR = LPI2C_MCR_RST_MASK;								// reset LPI2C0 completely
    	IP_LPI2C0->MCR = 0;												// disable LPI2C0 completely
    	IP_PCC->PCCn[PCC_LPI2C0_INDEX] = 0;								// Disable I2C clock
    }
    I2C_Rcv_Tail = I2C_Rcv_Head = 0;                                // reset receive buffer pointers
}

/**************************************************************************************************
Send and/or Receive data - master mode
***************************************************************************************************/
void i2c_masterCommand(int timer)
{
    if(I2C_Sendlen) I2C_Status |= I2C_Status_Send;
    if(I2C_Rcvlen) I2C_Status |= I2C_Status_Receive;
    I2C_Status &= ~(I2C_Status_Completed | I2C_Status_NoAck | I2C_Status_Timeout | I2C_Status_MasterCmd |
                                    I2C_Status_Slave_Receive | I2C_Status_Slave_Send | I2C_Status_Slave_Send_Rdy | I2C_Status_Slave_Receive_Rdy);
    I2C_Status |= I2C_Status_InProgress;
    I2C_Send_Index = 0;
    mmI2Cvalue = 0;
    if(timer) I2C_Timer = timer;
    I2C_State = I2C_State_Start;
	IP_LPI2C0->MSR = I2C_ERROR_MASK;							// clear all potential error flags

    // Enable transmit data interrupt (this is by default an active IRQ and therefore the thing get started!)
    IP_LPI2C0->MIER |= LPI2C_MIER_TDIE_MASK;
}

int I2C_HB_Write(int address, int reg, int value) {
    I2C_Sendlen = 2;                                                // send one byte
    I2C_Rcvlen = 0;
    *I2C_Send_Buffer = reg;                              // the first register to write
    *(I2C_Send_Buffer+1) = value;
    if(!DoI2C(address)) return ERROR;
    return SUCCESS;
}

void I2C_SetHBridge(int Device, int Mode)
{
	SetupI2C();
	if(I2C_HB_Write(Device, HB_IC1_CON_REG, Mode)) {
		if(CurrentLinePtr) error("H-Bridge not responding");
		MMPrintString("H-Bridge not responding");
		MMPrintString("\r\n");
	}
	if(CloseI2C) i2c_disable();										// all done on I2C, so close it
}

void I2C_SetEInput(int reg, int value)
{
	SetupI2C();
	if(I2C_HB_Write(PT_Addr, reg, value)) {
		if(CurrentLinePtr) error("Port device not responding");
		MMPrintString("Port device not responding");
		MMPrintString("\r\n");
	}
	if(CloseI2C) i2c_disable();										// all done on I2C, so close it
}

void Init_HBridges(void)
{
	unsigned int I2C_Error;

	SetupI2C();
	// Ignore all errors for initial steps as H-Bridges can already be configured and
	// therefore no device responds to HB_Address_ALL
	// This happens once RESET was applied
	I2C_HB_Write(HB_Address_ALL, HB_IC1_CON_REG, 0x03);				// Configure both H-Bridges to independent mode
	I2C_HB_Write(HB_Address_ALL, HB_IC2_CON_REG, 0x42);				// disable nFAULT pin function and OLD for both H-Bridges connected
	IP_PTC->PDDR |= FaultPin2_Bit;										// switch FAULT pin 2 to output
	IP_PTC->PCOR = FaultPin2_Bit;										// pull nFault of HB2 low -> deselect
	I2C_HB_Write(HB_Address_ALL,HB_SLAVE_ADDR_REG, HB_Address_HB1);	// program HB1 to new slave address
	IP_PTC->PDDR &= ~FaultPin2_Bit;									// switch fault pin of HB2 to input again
	IP_PTC->PDDR |= FaultPin1_Bit;										// switch FAULT pin 1 to output
	IP_PTC->PCOR = FaultPin1_Bit;										// pull nFault of HB1 low -> deselect
	I2C_HB_Write(HB_Address_ALL,HB_SLAVE_ADDR_REG, HB_Address_HB2);	// program HB2 to new slave address
	IP_PTC->PDDR &= ~FaultPin1_Bit;									// switch fault pin of HB1 to input again
	// Now both H-Bridges have been set up for individual address
	// check whether we get an answer from each one by an access
	I2C_Error = I2C_HB_Write(HB_Address_HB1, HB_IC2_CON_REG, 0x02);	// enable nFAULT pin function for HB1
	I2C_Error += I2C_HB_Write(HB_Address_HB2, HB_IC2_CON_REG, 0x02);// enable nFAULT pin function for HB2
	if(CloseI2C) i2c_disable();										// all done on I2C, so close it

	if(!I2C_Error)
	{
		// now as the H-Bridges are configured, we can enable the IRQs
		//PORTC->PCR[FaultPin1] |= PORT_PCR_IRQC(8);						// enable IRQ capability for falling edge
		//PORTC->PCR[FaultPin2] |= PORT_PCR_IRQC(8);						// enable IRQ capability for falling edge
	} else {
	// Cleanup in case of error
	IP_PTC->PDDR &= ~(FaultPin1_Bit|FaultPin2_Bit);					// switch both fault pins to input
	//PORTC->PCR[FaultPin1] &= ~PORT_PCR_IRQC(8);						// disable IRQ capability
	//PORTC->PCR[FaultPin2] &= ~PORT_PCR_IRQC(8);						// disable IRQ capability
	if(CurrentLinePtr) error("H-Bridge not responding");
	MMPrintString("H-Bridge not responding");
	MMPrintString("\r\n");
	}
}
