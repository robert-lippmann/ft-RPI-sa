/***********************************************************************************************************************
MMBasic

Flash.h

Include file that contains the globals and defines for flash save/load in MMBasic.

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


// We're using the 64k DFLASH for program storage
#define FLASH_PAGE_SIZE   2048
#define PROG_FLASH_SIZE  (FLASH_PAGE_SIZE * 30)                 // the amount of flash reserved for the BASIC program
#define EDIT_BUFFER_SIZE  ((int)(RAMEND - (int)RAMBase - 256))  // this is the maximum RAM that we can get
#define SAVEDVARS_FLASH_SIZE (FLASH_PAGE_SIZE)                  // amount of flash reserved for saved variables and options

#if !defined(INCLUDE_COMMAND_TABLE) && !defined(INCLUDE_TOKEN_TABLE) && !defined(FLASH_INCLUDED)
#define FLASH_INCLUDED

struct option_s {
    char Autorun;
    char Tab;
    char Listcase;
    char Height;
    char Width;
    int  PIN;
    int  Baudrate;
    int  ColourCode;
    unsigned int ProgFlashSize;    // used to store the size of the program flash (also start of the LIBRARY code)
};

extern struct option_s Option;

extern char ProgMemory[];
extern unsigned char *CFunctionFlash, *CFunctionLibrary;
extern char SavedVarsFlash[];

extern char *flashptr;

void ResetAllOptions(void);
void ResetAllFlash(void);
int SaveOptions(void);
void LoadOptions(void);
void FlashWriteInit(char *p, int nbr);
void FlashWriteByte(unsigned char b);
void FlashWriteByteErase(char c);
void FlashWriteWordErase(unsigned int i);
void FlashWriteClose(void);
int GetFlashOption(const unsigned int *w) ;
void SetFlashOption(const unsigned int *w, int x) ;
void ClearSavedVars(void);
void SaveOptionString(char *s);

//void cmd_var(void);
//void cmd_library(void);

void RoundDoubleFloat(MMFLOAT *ff);

int NVMErasePage(char * address);
int NVMWritePhrase(char *address, int* data);

#endif


/**********************************************************************************
 All command tokens tokens (eg, PRINT, FOR, etc) should be inserted in this table
**********************************************************************************/
#ifdef INCLUDE_COMMAND_TABLE

#endif


/**********************************************************************************
 All other tokens (keywords, functions, operators) should be inserted in this table
**********************************************************************************/
#ifdef INCLUDE_TOKEN_TABLE

#endif
