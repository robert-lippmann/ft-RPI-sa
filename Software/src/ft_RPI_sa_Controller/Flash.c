/***********************************************************************************************************************
MMBasic

Flash.c

Handles saving and restoring from flash.

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


#define RoundUp(a)     (((a) + (sizeof(int) - 1)) & (~(sizeof(int) - 1)))// round up to the nearest integer size


//////////////////////////////////////////////////////////////////////////////////////////////
// Macro to reserve flash memory for saving/loading data and initialise to 0xFF's
// Note that ?bytes? need to be a multiple of:-
//  - BYTE_PAGE_SIZE (and aligned that way) if you intend to erase
//  - BYTE_ROW_SIZE (and aligned that way) if you intend to write rows
//  - sizeof(int) if you intend to write words
#define NVM_ALLOCATE(name, align, bytes, fill) char name[(bytes)] \
       __attribute__ ((aligned(align),section(".PgmFlashMemory"))) = \
       { [0 ...(bytes)-1] = fill }

// ProgMemory is provided by the Linker (see Linker .ld file)
//NVM_ALLOCATE(ProgMemory, FLASH_PAGE_SIZE, PROG_FLASH_SIZE, 0xff);

// allocate space in flash for the saved variables and options.
NVM_ALLOCATE(SavedVarsFlash, FLASH_PAGE_SIZE, SAVEDVARS_FLASH_SIZE, 0x00);

// The CFUNCTION data comes after the program in program memory
// and this is used to point to its start
unsigned char *CFunctionFlash = NULL;
unsigned char *CFunctionLibrary = NULL;

struct option_s Option;

// globals used when writing bytes to flash
char *flashptr;
union {
  long lng;
  int wrd[2];
  char ch[8];
} WBuf;

// These are the S32K1 specific functions for flash programming

int NVMErasePage(char * address)
{
	unsigned int* FCCOB_ptr=(unsigned int*)IP_FTFC->FCCOB;

	address = (char*)((address - (char*)0x10000000) | (1<<23));		// reformat address to physical and set bit 23 -> DFLASH
	*FCCOB_ptr = (0x09<<24)|(int)address;						// include command 0x08 for erasing flash block
	IP_FTFC->FSTAT = FTFC_FSTAT_CCIF_MASK;							// fire erase command
	while(!(IP_FTFC->FSTAT & FTFC_FSTAT_CCIF_MASK));				// wait for command to be finished
	if (IP_FTFC->FSTAT & (FTFC_FSTAT_RDCOLERR_MASK|FTFC_FSTAT_ACCERR_MASK|FTFC_FSTAT_FPVIOL_MASK)) return 1;
	return 0;
}

int NVMWritePhrase(char *address, int* data)
{
	unsigned int* FCCOB_ptr=(unsigned int*)IP_FTFC->FCCOB;
	char* SaveAddress = address;

	if ((*(int*)SaveAddress) == *data && (*((int*)SaveAddress+1)) == *(data+1)) return 0;	// flash content already equiv to data
	address = (char*)((address - (char*)0x10000000) | (1<<23));		// reformat address to physical and set bit 23 -> DFLASH
	*FCCOB_ptr = (0x07<<24)|(int)address;						// include command 0x07 for programming
	*(FCCOB_ptr+1) = *data;										// set up data
	*(FCCOB_ptr+2) = *(data+1);
	IP_FTFC->FSTAT = FTFC_FSTAT_CCIF_MASK;							// fire programming command
	while(!(IP_FTFC->FSTAT & FTFC_FSTAT_CCIF_MASK));				// wait for command to be finished
	if (IP_FTFC->FSTAT & (FTFC_FSTAT_RDCOLERR_MASK|FTFC_FSTAT_ACCERR_MASK|FTFC_FSTAT_FPVIOL_MASK)) return 1;

	// Now check on first word whether the content has been programmed
	*FCCOB_ptr = (0x02<<24)|(int)address;						// include command 0x02 for verify
	*(FCCOB_ptr+1) = 0x02<<24;								    // compare with factory margin
	*(FCCOB_ptr+2) = *data;
	IP_FTFC->FSTAT = FTFC_FSTAT_CCIF_MASK;							// fire compare command
	while(!(IP_FTFC->FSTAT & FTFC_FSTAT_CCIF_MASK));				// wait for command to be finished
	if (IP_FTFC->FSTAT & (FTFC_FSTAT_RDCOLERR_MASK|FTFC_FSTAT_ACCERR_MASK|FTFC_FSTAT_FPVIOL_MASK)) return 1;

	// Now check on second word whether the content has been programmed
	*FCCOB_ptr = (0x02<<24)|(int)address;						// include command 0x02 for verify
	*(FCCOB_ptr+1) = 0x02<<24;									// compare with factory margin
	*(FCCOB_ptr+2) = *(data+1);
	IP_FTFC->FSTAT = FTFC_FSTAT_CCIF_MASK;							// fire compare command
	while(!(IP_FTFC->FSTAT & FTFC_FSTAT_CCIF_MASK));				// wait for command to be finished
	if (IP_FTFC->FSTAT & (FTFC_FSTAT_RDCOLERR_MASK|FTFC_FSTAT_ACCERR_MASK|FTFC_FSTAT_FPVIOL_MASK)) return 1;

	return 0;
}
// erase the flash and init the variables used to buffer bytes for writing to the flash
void FlashWriteInit(char *p, int nbr) {
    flashptr = p;
    WBuf.lng = 0;
    while(nbr > 0) {
        if(NVMErasePage((char *)p)) error("Flash write");
        p += FLASH_PAGE_SIZE;
        nbr -= FLASH_PAGE_SIZE;
    }
}

// write a byte to flash
// this will buffer four bytes so that the write to flash can be a word
void FlashWriteByte(unsigned char b) {
    WBuf.ch[(int)flashptr & 0b111] = b;
    flashptr++;
    if(((int)flashptr & 0b111) == 0)			// check whether we have a complete phrase (8 bytes) set up
    {
        if(NVMWritePhrase((void *)(flashptr - 8), WBuf.wrd)) error("Flash write");
        WBuf.lng = 0;
    }
}

// utility routine used by SaveProgramToFlash() and cmd_var to write a byte to flash while erasing the page if needed
void FlashWriteByteErase(char c) {
	if(((int)flashptr % FLASH_PAGE_SIZE) == 0) if(NVMErasePage(flashptr))
    	error("Flash write");
    FlashWriteByte(c);

}

// utility routine used by SaveProgramToFlash() and cmd_var to write a byte to flash while erasing the page if needed
void FlashWriteWordErase(unsigned int i) {
    if(((int)flashptr % FLASH_PAGE_SIZE) == 0)
    	if(NVMErasePage(flashptr))
    		error("Flash write");
    FlashWriteByte(i & 0xff);
    FlashWriteByte((i>>8) & 0xff);
    FlashWriteByte((i>>16) & 0xff);
    FlashWriteByte((i>>24) & 0xff);
}

// flush any bytes in the buffer to flash
void FlashWriteClose(void) {
    while(((int)flashptr & 0b111) != 0) FlashWriteByte(0xff);
}

/**********************************************************************************************
   These routines are used to load or save the global Option structure from/to flash.
   These options are stored in the beginning of the flash used to save stored variables.
***********************************************************************************************/

int SaveOptions(void) {
    char buf[FLASH_PAGE_SIZE];
    int i;

    if(!memcmp(&Option, SavedVarsFlash, sizeof(struct option_s)))
        return 0;                                                   // exit if the option has already been set (ie, nothing to do)

    while(!(IP_LPUART1->STAT & LPUART_STAT_TDRE_MASK));                    // wait for the console UART to send whatever is in its buffer
    memcpy(buf, SavedVarsFlash, FLASH_PAGE_SIZE);                   // copy the page to RAM
    memcpy(buf, &Option, sizeof(struct option_s));                  // insert our new data into the RAM copy
    IP_LMEM->PCCCR = LMEM_PCCCR_INVW0(1) | LMEM_PCCCR_INVW1(1) | LMEM_PCCCR_GO(1) | LMEM_PCCCR_ENCACHE(0); // disable cache

    FlashWriteInit(SavedVarsFlash, FLASH_PAGE_SIZE);                // erase the page
    for(i = 0; i < FLASH_PAGE_SIZE; i++) FlashWriteByte(buf[i]);    // write the changed page back to flash
    FlashWriteClose();
    IP_LMEM->PCCCR = LMEM_PCCCR_INVW0(1) | LMEM_PCCCR_INVW1(1) | LMEM_PCCCR_GO(1) | LMEM_PCCCR_ENCACHE(1); // re-enable cache

    return 1;
}


void LoadOptions(void) {
    memcpy(&Option, SavedVarsFlash, sizeof(struct option_s));
    if(!Option.Baudrate) {                                          // if we have never saved any options load the defaults
        Option.Height = SCREENHEIGHT;
        Option.Width = SCREENWIDTH;
        Option.Tab = 2;
        Option.ProgFlashSize = PROG_FLASH_SIZE;
        Option.Baudrate = CONSOLE_BAUDRATE;
    }
}



// erase the option table in flash and reset the options to their defaults
// used on initial firmware run
void ResetAllOptions(void) {
    int i;
    while(!(IP_LPUART1->STAT & LPUART_STAT_TDRE_MASK));				// wait for the console UART to send whatever is in its buffer
    IP_LMEM->PCCCR = LMEM_PCCCR_INVW0(1) | LMEM_PCCCR_INVW1(1) | LMEM_PCCCR_GO(1) | LMEM_PCCCR_ENCACHE(0); // disable cache
    FlashWriteInit(SavedVarsFlash, FLASH_PAGE_SIZE);				// erase flash page containing options variables
    for(i = 0; i < SAVEDVARS_FLASH_SIZE; i++) FlashWriteByte(0);    // set the option table and saved vars in flash to all zeroes
    FlashWriteClose();
    IP_LMEM->PCCCR = LMEM_PCCCR_INVW0(1) | LMEM_PCCCR_INVW1(1) | LMEM_PCCCR_GO(1) | LMEM_PCCCR_ENCACHE(1); // re-enable cache
    LoadOptions();
}


// erase all flash memory and reset the options to their defaults
void ResetAllFlash(void) {
    ResetAllOptions();
    IP_LMEM->PCCCR = LMEM_PCCCR_INVW0(1) | LMEM_PCCCR_INVW1(1) | LMEM_PCCCR_GO(1) | LMEM_PCCCR_ENCACHE(0); // disable cache
    FlashWriteInit(ProgMemory, PROG_FLASH_SIZE);                     // erase program memory
    FlashWriteClose();
    IP_LMEM->PCCCR = LMEM_PCCCR_INVW0(1) | LMEM_PCCCR_INVW1(1) | LMEM_PCCCR_GO(1) | LMEM_PCCCR_ENCACHE(1); // re-enable cache
}

