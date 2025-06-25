/************************************************************************************************************************
fischertechnik-Raspberry Pi-Stand-Alone Controller

BootLoader.c

Routines to handle the serial upload of a new firmware to get rid of the required debugger
All functions in this file will be copied into the standard RAM to allow write access to the Flash memory

Copyright 2023 Robert Lippmann

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
   developed by Robert Lippmann.

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
#include <stdio.h>
#if defined(CPU_S32K144_M4)
#include <S32K144_features.h>
#elif defined(CPU_S32K142_M4)
#include <S32K142_features.h>
#elif defined(CPU_S32K146_M4)
#include <S32K146_features.h>
#endif

extern uint32_t* __RAM_VECTOR_TABLE[];
extern unsigned int __attribute__((section(".NoInit"))) _excep_MagicNumber;

// static char ConsoleBufRX[CONSOLE_BUF_SIZE]; /* We use the ConsoleBuf... defined in the console.c file */
static char ConsoleBufTX[CONSOLE_BUF_SIZE];
static volatile int ConsoleBufTXHead = 0;
static volatile int ConsoleBufTXTail = 0;

int srec_line = 0;
static srec_info_t srinfo;
static uint8_t sr_data_buf[SREC_DATA_MAX_BYTES];

void __attribute__((section(".BootLoader.$func"), noinline)) BL_DefaultISR()
{
	while(1);
}

void __attribute__((section(".BootLoader.$func"), noinline)) BL_SystemSoftwareReset(void)
{
    uint32_t regValue;

    /* Read Application Interrupt and Reset Control Register */
    regValue = S32_SCB->AIRCR;

    /* Clear register key */
    regValue &= ~( S32_SCB_AIRCR_VECTKEY_MASK);

    /* Configure System reset request bit and Register Key */
    regValue |= S32_SCB_AIRCR_VECTKEY(FEATURE_SCB_VECTKEY);
    regValue |= S32_SCB_AIRCR_SYSRESETREQ(0x1u);

    /* Write computed register value */
    S32_SCB->AIRCR = regValue;
}

void __attribute__((section(".BootLoader.$func"), noinline)) BL_Int2Str(uint32_t Number2Convert, uint8_t *str)
{
	uint32_t Divider = 100000L;					// this represents the max number to convert

	while (Number2Convert < Divider)			// modify divider until it's in range with number to convert
		Divider /= 10;
	while (Divider)        						// repeat for each digit
	{
		*str++ = Number2Convert/Divider + '0';  // convert/store digit to string
		Number2Convert %= Divider;              // keep remainder
		Divider /= 10;                 			// move on to next digit
	}
	*str = 0;                   				// Append a string end identifier
}

uint8_t __attribute__((section(".BootLoader.$func"), noinline)) nybble_to_val (char x)
{
    if (x >= '0' && x <= '9')
	return (uint8_t)(x-'0');

    return (uint8_t)((x-'A') + 10);
}

uint8_t __attribute__((section(".BootLoader.$func"), noinline)) grab_hex_byte (uint8_t *buf)
{
    return  (uint8_t)((nybble_to_val ((char)buf[0]) << 4) +
                       nybble_to_val ((char)buf[1]));
}

uint16_t __attribute__((section(".BootLoader.$func"), noinline)) grab_hex_word (uint8_t *buf)
{
    return (uint16_t)(((uint16_t)grab_hex_byte (buf) << 8)
		      + grab_hex_byte ((uint8_t*)((int)buf+2))
                     );
}

uint32_t __attribute__((section(".BootLoader.$func"), noinline)) grab_hex_word24 (uint8_t *buf)
{
    return (uint32_t)(((uint32_t)grab_hex_byte (buf) << 16)
		      + grab_hex_word ((uint8_t*)((int)buf+2))
                     );
}

uint32_t __attribute__((section(".BootLoader.$func"), noinline)) grab_hex_dword (uint8_t *buf)
{
    return (uint32_t)(((uint32_t)grab_hex_word (buf) << 16)
		      + grab_hex_word ((uint8_t*)((int)buf+4))
                     );
}

uint8_t __attribute__((section(".BootLoader.$func"), noinline)) decode_srec_data (uint8_t *bufs, uint8_t *bufd, uint8_t count, uint8_t skip)
{
    uint8_t cksum = 0, cbyte;
    int i;

    /* Parse remaining character pairs */
    for (i=0; i < count; i++) {
        cbyte = grab_hex_byte (bufs);
        if ((i >= skip - 1) && (i != count-1))   /* Copy over only data bytes */
            *bufd++ = cbyte;
        bufs  += 2;
        cksum += cbyte;
    }

    return cksum;
}

uint8_t __attribute__((section(".BootLoader.$func"), noinline)) eatup_srec_line (uint8_t *bufs, uint8_t count)
{
    int i;
    uint8_t cksum = 0;

    for (i=0; i < count; i++) {
        cksum += grab_hex_byte(bufs);
        bufs += 2;
    }

    return cksum;
}

uint8_t __attribute__((section(".BootLoader.$func"), noinline)) BL_ParseString(uint8_t* sr_buf, srec_info_t *info)
{
    uint8_t count;
    uint8_t *bufs;
    uint8_t cksum = 0, skip;
    int type;

    bufs = sr_buf;

    srec_line++; /* for debug purposes on errors */

    if ( *bufs != 'S') {
    	return SREC_PARSE_ERROR;
    }

    type = *++bufs - '0';
    count = grab_hex_byte (++bufs);
    bufs += 2;
    cksum = count;

    switch (type)
    {
	case 0:
	    info->type = SREC_TYPE_0;
	    info->dlen = count;
        cksum += eatup_srec_line (bufs, count);
	    break;
/*	case 1:
	    info->type = SREC_TYPE_1;
	    skip = 3;
	    info->addr = (uint8_t*)(uint32_t)grab_hex_word (bufs);
	    info->dlen = count - skip;
        cksum += decode_srec_data (bufs, info->sr_data, count, skip);
        break;
	case 2:
	    info->type = SREC_TYPE_2;
	    skip = 4;
	    info->addr = (uint8_t*)(uint32_t)grab_hex_word24 (bufs);
	    info->dlen = count - skip;
        cksum += decode_srec_data (bufs, info->sr_data, count, skip);
	    break;
*/
	case 3:
	    info->type = SREC_TYPE_3;
	    skip = 5;
	    info->addr = (uint8_t*)(uint32_t)grab_hex_dword (bufs);
	    info->dlen = count - skip;
	    if(info->dlen != 1 && info->dlen != 8 && info->dlen !=16)
	    	return LD_SREC_LINE_ERROR;
        cksum += decode_srec_data (bufs, info->sr_data, count, skip);
        if(info->dlen==4 || info->dlen==12)
        	*(uint32_t*)(info->sr_data+info->dlen)=0xffffffff;
	    break;
/*	case 5:
	    info->type = SREC_TYPE_5;
	    info->addr = (uint8_t*)(uint32_t)grab_hex_word (bufs);
        cksum += eatup_srec_line (bufs, count);
	    break;
	    */
	case 7:
	    info->type = SREC_TYPE_7;
	    info->addr = (uint8_t*)(uint32_t)grab_hex_dword (bufs);
        cksum += eatup_srec_line (bufs, count);
	    break;
/*	case 8:
	    info->type = SREC_TYPE_8;
	    info->addr = (uint8_t*)(uint32_t)grab_hex_word24 (bufs);
        cksum += eatup_srec_line (bufs, count);
	    break;
	case 9:
	    info->type = SREC_TYPE_9;
	    info->addr = (uint8_t*)(uint32_t)grab_hex_word (bufs);
        cksum += eatup_srec_line (bufs, count);
	    break;
	    */
	default:
	    return SREC_PARSE_ERROR;
    }

    if (++cksum) {
    	return SREC_CKSUM_ERROR;
    }
    return 0;
}

int __attribute__((section(".BootLoader.$func"), noinline)) BL_NVMWritePhrase(char* address, int* data)
{
	unsigned int* FCCOB_ptr=(unsigned int*)IP_FTFC->FCCOB;

#if defined(CPU_S32K144_M4)
#define MAX_ADDRESS 0x00080000
#elif defined (CPU_S32K142_M4)
#define MAX_ADDRESS 0x00040000
#elif defined (CPU_S32K146_M4)
#define MAX_ADDRESS 0x00100000
#endif

	if ((((uint32_t)address >= MAX_ADDRESS) && ((uint32_t)address < 0x10000000)) || ((uint32_t)address > 0x1000FFFF))	// is address out of range for program & data flash?
		goto WriteError;
	// Program 8 consecutive bytes at once (max. 225us required)
	if ((*(uint32_t*)address) == *data && (*((uint32_t*)address+1)) == *(data+1)) return 0;	// flash content already equiv to data
	if (((uint32_t)address >= 0x10000000) && ((uint32_t)address <= 0x1000ffff))
		address = (char*)((address - (char*)0x10000000) | (1<<23));		// reformat address to physical DFLASH range (bit 23 set -> DFLASH) see "36.5.11 Flash command descriptions" in the Reference Manual

	*FCCOB_ptr = (0x07<<24)|(int)address;						// include command 0x07 for programming
	*(FCCOB_ptr+1) = *data;										// set up data
	*(FCCOB_ptr+2) = *(data+1);
	IP_FTFC->FSTAT = FTFC_FSTAT_CCIF_MASK;							// fire programming command
	while(!(IP_FTFC->FSTAT & FTFC_FSTAT_CCIF_MASK));				// wait for command to be finished
	if (IP_FTFC->FSTAT & (FTFC_FSTAT_RDCOLERR_MASK|FTFC_FSTAT_ACCERR_MASK|FTFC_FSTAT_FPVIOL_MASK)) goto WriteError;

	// Now check on first word whether the content has been programmed (max. 95us required)
	*FCCOB_ptr = (0x02<<24)|(int)address;						// include command 0x02 for verify
	*(FCCOB_ptr+1) = 0x02<<24;								    // compare with factory margin
	*(FCCOB_ptr+2) = *data;
	IP_FTFC->FSTAT = FTFC_FSTAT_CCIF_MASK;							// fire compare command
	while(!(IP_FTFC->FSTAT & FTFC_FSTAT_CCIF_MASK));				// wait for command to be finished
	if (IP_FTFC->FSTAT & (FTFC_FSTAT_RDCOLERR_MASK|FTFC_FSTAT_ACCERR_MASK|FTFC_FSTAT_FPVIOL_MASK)) goto WriteError;

	// Now check on second word whether the content has been programmed (max. 95us required)
	*FCCOB_ptr = (0x02<<24)|(int)address;						// include command 0x02 for verify
	*(FCCOB_ptr+1) = 0x02<<24;									// compare with factory margin
	*(FCCOB_ptr+2) = *(data+1);
	IP_FTFC->FSTAT = FTFC_FSTAT_CCIF_MASK;							// fire compare command
	while(!(IP_FTFC->FSTAT & FTFC_FSTAT_CCIF_MASK));				// wait for command to be finished
	if (IP_FTFC->FSTAT & (FTFC_FSTAT_RDCOLERR_MASK|FTFC_FSTAT_ACCERR_MASK|FTFC_FSTAT_FPVIOL_MASK)) goto WriteError;

	return 0;
	WriteError:
	return 1;
}

int __attribute__((section(".BootLoader.$func"), noinline)) BL_NVMWriteData(srec_info_t *info)
{
	unsigned int ReturnStatus=0;

	if(((int)info->addr <=0x0007ffff) || ((int)info->addr >= 0x10000000 && (int)info->addr <=0x1000ffff))	// check for valid flash address and simply ignore everything else
	{
		switch(info->dlen)
		{
		case 1:			/* Just ignore lines with no data given */
			break;
		case 8:
			ReturnStatus+=BL_NVMWritePhrase((char*)info->addr, (int*)info->sr_data);
			break;
		case 16:
			ReturnStatus+=BL_NVMWritePhrase((char*)info->addr, (int*)info->sr_data);
			ReturnStatus+=BL_NVMWritePhrase((char*)(info->addr+8), (int*)(info->sr_data+8));
			break;
		default:
			return SREC_LENGTH_ERROR;
		}
	}
	return ReturnStatus;
}

int __attribute__((section(".BootLoader.$func"), noinline)) BL_NVMEraseAll()
{
	uint32_t* FCCOB_ptr=(uint32_t*)IP_FTFC->FCCOB;

	*FCCOB_ptr = (0x08<<24)|(int)0x00000000;					// include command 0x08 for erasing program flash block
	IP_FTFC->FSTAT = FTFC_FSTAT_CCIF_MASK;							// fire erase command
	while(!(IP_FTFC->FSTAT & FTFC_FSTAT_CCIF_MASK));				// wait for command to be finished
	IP_FTFC->FSTAT = FTFC_FSTAT_RDCOLERR_MASK;
	if (IP_FTFC->FSTAT & (FTFC_FSTAT_ACCERR_MASK|FTFC_FSTAT_FPVIOL_MASK)) goto FlashError;

	*FCCOB_ptr = (0x08<<24)|(int)0x00000000|(1<<23);			// include command 0x08 for erasing data flash block (bit 23 set to 1)
	IP_FTFC->FSTAT = FTFC_FSTAT_CCIF_MASK;							// fire erase command
	while(!(IP_FTFC->FSTAT & FTFC_FSTAT_CCIF_MASK));				// wait for command to be finished
	IP_FTFC->FSTAT = FTFC_FSTAT_RDCOLERR_MASK;
	if (IP_FTFC->FSTAT & (FTFC_FSTAT_ACCERR_MASK|FTFC_FSTAT_FPVIOL_MASK)) goto FlashError;

	return 0;
	FlashError:
	return 1;
}

// LPUART 1 interrupt handler
void __attribute__((section(".BootLoader.$func"), noinline)) BL_LPUART1_RxTx_IRQHandler(void) {

    while(IP_LPUART1->STAT & LPUART_STAT_RDRF_MASK) {                  // while there is data to read
        if(IP_LPUART1->STAT  & (0b111<<17)) {                    	 	// first check for errors (OR, NF, FE)
            (void)IP_LPUART1->DATA;                                 	// and if there was an error throw away the char
            IP_LPUART1->STAT = 0b111<<17;                              // clear the error on the UART
            continue;                                               // and try the next char
        }
        ConsoleBuf[ConsoleBufHead]  = IP_LPUART1->DATA;       			// store the byte in the ring buffer (and clear IRQ flag)
        ConsoleBufHead = (ConsoleBufHead + 1) % CONSOLE_BUF_SIZE;   // advance the head of the queue
        if(ConsoleBufHead == ConsoleBufTail) {                      // if the buffer has overflowed
            ConsoleBufTail = (ConsoleBufTail + 1) % CONSOLE_BUF_SIZE; // throw away the oldest char
        }
    }
    while((ConsoleBufTXHead != ConsoleBufTXTail) && (IP_LPUART1->STAT & LPUART_STAT_TDRE_MASK))	// any data to send and transmit register empty?
    {
		IP_LPUART1->DATA = ConsoleBufTX[ConsoleBufTXTail];					// put char into TX data register and clear IRQ flag
		ConsoleBufTXTail = (ConsoleBufTXTail + 1) % CONSOLE_BUF_SIZE;	// increase TX get pointer
		if(ConsoleBufTXHead == ConsoleBufTXTail)						// Are we done with all characters to be sent?
			IP_LPUART1->CTRL &= ~LPUART_CTRL_TIE_MASK;						// Yes, disable TX interrupt
    }
}

char __attribute__((section(".BootLoader.$func"))) BL_GetCharFrom_RXQueue()
{
	char character;

	character=ConsoleBuf[ConsoleBufTail];		// discard following CR or LF
	ConsoleBufTail = (ConsoleBufTail + 1) % CONSOLE_BUF_SIZE;	// increase TX get pointer
	return character;
}

void __attribute__((section(".BootLoader.$func"))) BL_PutCharInto_TXQueue(char character)
{
	ConsoleBufTX[ConsoleBufTXHead] = character;
	ConsoleBufTXHead = (ConsoleBufTXHead + 1) % CONSOLE_BUF_SIZE;	// increase TX put pointer
}

void __attribute__((section(".BootLoader.$func"), noinline)) BL_GetString(uint8_t* message)
{
	char ch;

	while(1)
	{
		while(ConsoleBufTail == ConsoleBufHead);		// wait until char is received
		ch = BL_GetCharFrom_RXQueue();					// read character and increase read pointer

		if ((ch >= 'A' && ch <= 'Z')||
			(ch >= '0' && ch <= '9'))
		  {
			*message++  = ch;
		  }
		else
		if (ch == '\r' || ch == '\n')
		{
			*message++ = '\0';
			while(ConsoleBufTail == ConsoleBufHead);		// wait until char is received
			if(ConsoleBuf[ConsoleBufTail] == '\r' || ConsoleBuf[ConsoleBufTail] == '\n')	// eat up following control char (every '\r' a '\n' follows or vice versa)
				(void)BL_GetCharFrom_RXQueue();		// discard following CR or LF
		break;
		}
	}
}

uint8_t __attribute__((section(".BootLoader.$func"), noinline)) BL_GetChar()
{
	uint8_t character;

	while(ConsoleBufTail == ConsoleBufHead);		// wait until char is received
    character=BL_GetCharFrom_RXQueue();
	return character;
}

void __attribute__((section(".BootLoader.$func"), noinline)) BL_PutChar(uint8_t character)
{
	DISABLE_INTERRUPTS();							// disable interrupts to avoid race conditions
	if(IP_LPUART1->CTRL & LPUART_CTRL_TIE_MASK)		// Check whether a Transmission is already ongoing?
	{
		ENABLE_INTERRUPTS();						// re-enable interrupts to continue with transmission
		while((ConsoleBufTXHead==ConsoleBufTXTail) && (IP_LPUART1->CTRL & LPUART_CTRL_TIE_MASK));	// wait in case of full TX queue or queue is empty and the TX interrupt was just disabled (corner case)
	}
	BL_PutCharInto_TXQueue(character);				// put character into queue
	IP_LPUART1->CTRL |= LPUART_CTRL_TIE_MASK;			// Enable TX interrupt
	ENABLE_INTERRUPTS();							// re-enable interrupts
}

void __attribute__((section(".BootLoader.$func"), noinline)) BL_Print(char* message)
{
	while(*message != 0)
	{
		BL_PutChar(*message++);
	}
}
void __attribute__((section(".BootLoader.$func"), noinline)) BL_InitSystem()
{
	DISABLE_INTERRUPTS();

	/* Point the VTOR to the position of new vector table */
    S32_SCB->VTOR = (uint32_t)__RAM_VECTOR_TABLE;

    if(Option.Baudrate>115200 || Option.Baudrate==0)	// check for baudrate (0 is the case once wrong firmware installed)
    	Option.Baudrate=115200;
    uint32_t SBR=48000000/(16*Option.Baudrate);

	IP_PCC->PCCn[PCC_LPUART1_INDEX] = 0;						// disable clock
	IP_PCC->PCCn[PCC_LPUART1_INDEX] = PCC_PCCn_PCS(3);			// PCS = 3: Select FIRCDIV2 = 48MHz
	IP_PCC->PCCn[PCC_LPUART1_INDEX] |= PCC_PCCn_CGC_MASK;		// Enable clock for LPUART1
    IP_LPUART1->BAUD = LPUART_BAUD_OSR(15)|LPUART_BAUD_SBR(SBR);		// configure Baudrate to last stored value with 16 times oversampling
    IP_LPUART1->FIFO = LPUART_FIFO_TXFE(1)|LPUART_FIFO_RXFE(1);		// enable 4-byte RX/TX-Fifo
    IP_LPUART1->CTRL = LPUART_CTRL_TE(1)|LPUART_CTRL_RE(1)				// enable receiver & transmitter interrupt from LPUART1
                    | LPUART_CTRL_RIE(1);
    // enable UART1 IRQ
	S32_NVIC->ISER[(LPUART1_RxTx_IRQn>>5)] |= (uint32_t)(1U << ((uint32_t)(LPUART1_RxTx_IRQn) & (uint32_t)0x1FU));

	ENABLE_INTERRUPTS();
}

char MSG0[] __attribute__((section(".BootLoader"))) = "\r\n";
char MSG1[] __attribute__((section(".BootLoader"))) = "\r\n*** ft-RPI-sa BootLoader V1.0 ***\r\n\r\n";
char MSG2[] __attribute__((section(".BootLoader"))) = "Press\r\n<D> to download new firmware\r\n";
char MSG3[] __attribute__((section(".BootLoader"))) = "<C> to check integrity of firmware file\r\n";
char MSG4[] __attribute__((section(".BootLoader"))) = "Checking integrity of firmware file\r\n";
char MSG5[] __attribute__((section(".BootLoader"))) = "ARE YOU SURE TO DOWNLOAD NEW FIRMWARE? (Y)\r\n";
char MSG6[] __attribute__((section(".BootLoader"))) = "Ok, let's download a new firmware\r\n";
char MSG7[] __attribute__((section(".BootLoader"))) = "*** Flash erase error ***\r\n";
char MSG8[] __attribute__((section(".BootLoader"))) = "Flash successfully erased\r\n";
char MSG9[] __attribute__((section(".BootLoader"))) = "\r\nStart file transfer now...\r\n";
char MSG10[] __attribute__((section(".BootLoader"))) = "*";
char MSG11[] __attribute__((section(".BootLoader"))) = "\r\nNo errors found in file";
char MSG12[] __attribute__((section(".BootLoader"))) = "\r\nErrors found in file";
char MSG13[] __attribute__((section(".BootLoader"))) = "\r\nAll done...Press any key for reset";
char MSG14[] __attribute__((section(".BootLoader"))) = "\r\nNo errors during update\r\n";
char MSG15[] __attribute__((section(".BootLoader"))) = "\r\nS-Record error in line ";
char MSG16[] __attribute__((section(".BootLoader"))) = "*** Flash program error ***\r\n";

void __attribute__((section(".BootLoader.$func"), noinline)) BootLoader(void)
{
	uint8_t	String[CONSOLE_BUF_SIZE], ReturnStatus, FileStatus=0, Mode=0;
    srinfo.sr_data = sr_data_buf;


	BL_InitSystem();
	BL_Print(MSG1);
	BL_Print(MSG2);
	BL_Print(MSG3);

	do {
		ReturnStatus=BL_GetChar()&0xdf;
	} while(ReturnStatus!='D' && ReturnStatus!='C');
	BL_Print(MSG0);
	if(ReturnStatus=='C')
		BL_Print(MSG4);
	else
	{
		BL_Print(MSG5);
		do {
			ReturnStatus=BL_GetChar()&0xdf;
		} while(ReturnStatus!='Y');
		BL_Print(MSG6);
		Mode=1;
		if(BL_NVMEraseAll()) {
			BL_Print(MSG7);
			goto BL_EXIT;
		} else
			BL_Print(MSG8);
	}
	BL_Print(MSG9);

	do {
		BL_GetString(String);
		ReturnStatus=BL_ParseString(String, &srinfo);
		FileStatus+=ReturnStatus;
		if(ReturnStatus)
		{
			BL_Print(MSG15);
			BL_Int2Str(srec_line, String);
			BL_Print((char*)String);
			BL_Print(MSG0);
		}
		if(Mode==1 && srinfo.type == SREC_TYPE_3)
		{
			if(BL_NVMWriteData(&srinfo))
				BL_Print(MSG16);
		}

		if((srec_line % 100) == 0) {
			BL_Print(MSG10);
			LEDToggle;
		}
		if((srec_line % 3000) == 0)
			BL_Print(MSG0);
	}
	while(srinfo.type < 7);
	if (Mode==0)
	{
		if (FileStatus==0)
			BL_Print(MSG11);
		else
			BL_Print(MSG12);
	} else
		BL_Print(MSG14);
	BL_Print(MSG13);
	BL_EXIT:
	LEDOn;
	(void)BL_GetChar();
	_excep_MagicNumber = 0;
	BL_SystemSoftwareReset();
}
