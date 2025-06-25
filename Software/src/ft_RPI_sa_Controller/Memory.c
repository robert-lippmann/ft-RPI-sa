/***********************************************************************************************************************

Memory.c

This module manages all RAM memory allocation for MMBasic.

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

**************************************************************************************************************************

NOTE:
  In the S32K1 Design Studio environment, the following variables are set by the linker (see Linker script):

      (unsigned char *)&__stack_end__  This is the address of the top of the stack

      (unsigned char *)&__heap_end__   This is the address of the top of the heap and represents the start of free
                                       and unallocated memory.

      (unsigned char *)&__heap_start__ This is the address of the top of the memory allocated by the compiler (static
                                       variables, etc)

      (unsigned int)&__StackLimit      This is the number of bytes allocated to the stack.  No run time checking is performed
                                       and this value is only used by the linker to warn if memory is over allocated.


  The RAM memory map, for the K144 device, looks like this:

  |--------------------|    <<<   0x20006fff  (Top of RAM)
  |  Exception vars    |
  |--------------------|    <<<   (unsigned char *)&__stack_end__
  |                    |
  | Stack (grows down) |
  |                    |
  |--------------------|    <<<   (unsigned char *)&__stack_start__
  |                    |
  |                    |
  |     Free RAM       |
  |                    |
  |                    |
  |--------------------|   <<<   (unsigned char *)&__heap_end__
  |                    |
  | Heap (if allocated)|
  |                    |
  |--------------------|   <<<   (unsigned char *)&__heap_start__
  |                    |
  |                    |
  |                    |
  |  Static RAM Vars   |
  |                    |
  |                    |
  |                    |
  |--------------------|    <<<   0x1fff8000

	This address map is valid for the S32K144(W) device with 64k of RAM (4k are FlexRAM memory but this is currently not
	used as it supports only 32bit accesses)

************************************************************************************************************************/

#include "Version.h"

unsigned int mmap[(MEMSIZE / PAGESIZE) / PAGESPERWORD];
char *StrTmp[MAXTEMPSTRINGS];                                       // used to track temporary string space on the heap
char StrTmpLocalIndex[MAXTEMPSTRINGS];                              // used to track the LocalIndex for each temporary string space on the heap
int TempMemoryIsChanged = false;                                    // used to prevent unnecessary scanning of strtmp[]
int StrTmpIndex = 0;                                                // index to the next unallocated slot in strtmp[]

void *RAMBase;                                                      // this is the base of the RAM used for the variable table (which grows up))
unsigned char *VarTableTop;                                         // this is the top of the RAM table

unsigned int MBitsGet(void *addr);
void MBitsSet(void *addr, int bits);
void *getheap(int size);
unsigned int UsedHeap(void);
void heapstats(char *m1);

/***********************************************************************************************************************
 MMBasic commands
************************************************************************************************************************/


void cmd_memory(void) {
#if !defined(LITE)
    int i, j, var, nbr, vsize, VarCnt;
    int ProgramSize, ProgramPercent, VarSize, VarPercent, GeneralSize, GeneralPercent, SavedVarSize, SavedVarSizeK, SavedVarPercent, SavedVarCnt;
    int CFunctSize, CFunctSizeK, CFunctNbr, CFunctPercent, FontSize, FontSizeK, FontNbr, FontPercent, LibrarySizeK, LibraryPercent;
    unsigned int CurrentRAM;
    char *p;

    CurrentRAM = (unsigned int)RAMEND - (unsigned int)RAMBase;

    // calculate the space allocated to variables on the heap
    for(i = VarCnt = vsize = var = 0; var < varcnt; var++) {
        if(vartbl[var].type == T_NOTYPE) continue;
        VarCnt++;  vsize += sizeof(struct s_vartbl);
        if(vartbl[var].val.s == NULL) continue;
        if(vartbl[var].type & T_PTR) continue;
        nbr = vartbl[var].dims[0] + 1 - OptionBase;
        if(vartbl[var].dims[0]) {
            for(j = 1; j < MAXDIM && vartbl[var].dims[j]; j++)
                nbr *= (vartbl[var].dims[j] + 1 - OptionBase);
            if(vartbl[var].type & T_NBR)
                i += MRoundUp(nbr * sizeof(MMFLOAT));
            else if(vartbl[var].type & T_INT)
                i += MRoundUp(nbr * sizeof(long long int));
            else
                i += MRoundUp(nbr * (vartbl[var].size + 1));
        } else
            if(vartbl[var].type & T_STR)
                i += STRINGSIZE;
    }

    VarSize = (vsize + i + 512)/1024;                               // this is the memory allocated to variables
    VarPercent = ((vsize + i) * 100)/CurrentRAM;
    if(VarCnt && VarSize == 0) VarPercent = VarSize = 1;            // adjust if it is zero and we have some variables
    i = UsedHeap() - i;
    if(i < 0) i = 0;
    GeneralSize = (i + 512)/1024; GeneralPercent = (i * 100)/CurrentRAM;

    // count the space used by saved variables (in flash)
    p = SavedVarsFlash + sizeof(struct option_s);
    SavedVarCnt = 0;
    while(!(*p == 0 || *p == 0xff)) {
        unsigned char type, array;
        SavedVarCnt++;
        type = *p++;
        array = type & 0x80;  type &= 0x7f;                         // set array to true if it is an array
        p += strlen(p) + 1;
        if(array)
            p += (p[0] | p[1] << 8) + 2;
        else {
            if(type &  T_NBR)
                p += sizeof(MMFLOAT);
            else if(type &  T_INT)
                p += sizeof(long long int);
            else
                p += *p + 1;
        }
    }
    SavedVarSize = p - (SavedVarsFlash + sizeof(struct option_s));
    SavedVarSizeK = (SavedVarSize + 512) / 1024;
    SavedVarPercent = (SavedVarSize * 100) / (PROG_FLASH_SIZE + SAVEDVARS_FLASH_SIZE);
    if(SavedVarCnt && SavedVarSizeK == 0) SavedVarPercent = SavedVarSizeK = 1;        // adjust if it is zero and we have some variables

    // count the space used by CFunctions, CSubs and fonts
    CFunctSize = CFunctNbr = FontSize = FontNbr = 0;
/*
    pint = (unsigned int *)CFunctionFlash;
    while(*pint != 0xffffffff) {
        if(*pint < FONT_TABLE_SIZE) {
            pint++;
            FontNbr++;
            FontSize += *pint + 8;
        } else {
            pint++;
            CFunctNbr++;
            CFunctSize += *pint + 8;
        }
        pint += (*pint + 4) / sizeof(unsigned int);
    } */
    CFunctPercent = (CFunctSize * 100) /  (PROG_FLASH_SIZE + SAVEDVARS_FLASH_SIZE);
    CFunctSizeK = (CFunctSize + 512) / 1024;
    if(CFunctNbr && CFunctSizeK == 0) CFunctPercent = CFunctSizeK = 1;              // adjust if it is zero and we have some functions
    FontPercent = (FontSize * 100) /  (PROG_FLASH_SIZE + SAVEDVARS_FLASH_SIZE);
    FontSizeK = (FontSize + 512) / 1024;
    if(FontNbr && FontSizeK == 0) FontPercent = FontSizeK = 1;                      // adjust if it is zero and we have some functions

    // count the number of lines in the program
    p = ProgMemory;
    i = 0;
    while(*p != 0xff) {                                             // skip if program memory is erased
        if(*p == 0) p++;                                            // if it is at the end of an element skip the zero marker
        if(*p == 0) break;                                          // end of the program or module
        if(*p == T_NEWLINE) {
            i++;                                                    // count the line
            p++;                                                    // skip over the newline token
        }
        if(*p == T_LINENBR) p += 3;                                 // skip over the line number
        skipspace(p);
        if(p[0] == T_LABEL) p += p[1] + 2;                          // skip over the label
        while(*p) p++;                                              // look for the zero marking the start of an element
    }
    ProgramSize = ((p - ProgMemory) + 512)/1024;
    ProgramPercent = ((p - ProgMemory) * 100)/(PROG_FLASH_SIZE + SAVEDVARS_FLASH_SIZE);
    if(ProgramPercent > 100) ProgramPercent = 100;
    if(i && ProgramSize == 0) ProgramPercent = ProgramSize = 1;                                        // adjust if it is zero and we have some lines

    MMPrintString("Flash:\r\n");
    IntToStrPad(inpbuf, ProgramSize, ' ', 4, 10); strcat(inpbuf, "K (");
    IntToStrPad(inpbuf + strlen(inpbuf), ProgramPercent, ' ', 2, 10); strcat(inpbuf, "%) Program (");
    IntToStr(inpbuf + strlen(inpbuf), i, 10); strcat(inpbuf, " lines)\r\n");
    MMPrintString(inpbuf);

    if(CFunctNbr) {
        IntToStrPad(inpbuf, CFunctSizeK, ' ', 4, 10); strcat(inpbuf, "K (");
        IntToStrPad(inpbuf + strlen(inpbuf), CFunctPercent, ' ', 2, 10); strcat(inpbuf, "%) "); MMPrintString(inpbuf);
        IntToStr(inpbuf, CFunctNbr, 10); strcat(inpbuf, " Embedded C Routine"); strcat(inpbuf, CFunctNbr == 1 ? "\r\n":"s\r\n");
        MMPrintString(inpbuf);
    }

    if(FontNbr) {
        IntToStrPad(inpbuf, FontSizeK, ' ', 4, 10); strcat(inpbuf, "K (");
        IntToStrPad(inpbuf + strlen(inpbuf), FontPercent, ' ', 2, 10); strcat(inpbuf, "%) "); MMPrintString(inpbuf);
        IntToStr(inpbuf, FontNbr, 10); strcat(inpbuf, " Embedded Font"); strcat(inpbuf, FontNbr == 1 ? "\r\n":"s\r\n");
        MMPrintString(inpbuf);
    }

    if(SavedVarCnt) {
        IntToStrPad(inpbuf, SavedVarSizeK, ' ', 4, 10); strcat(inpbuf, "K (");
        IntToStrPad(inpbuf + strlen(inpbuf), SavedVarPercent, ' ', 2, 10); strcat(inpbuf, "%)");
        IntToStrPad(inpbuf + strlen(inpbuf), SavedVarCnt, ' ', 2, 10); strcat(inpbuf, " Saved Variable"); strcat(inpbuf, SavedVarCnt == 1 ? " (":"s (");
        IntToStr(inpbuf + strlen(inpbuf), SavedVarSize, 10); strcat(inpbuf, " bytes)\r\n");
        MMPrintString(inpbuf);
    }

    LibrarySizeK = LibraryPercent = 0;
    if(Option.ProgFlashSize != PROG_FLASH_SIZE) {
        LibrarySizeK = PROG_FLASH_SIZE - Option.ProgFlashSize;
        LibraryPercent = (LibrarySizeK * 100)/PROG_FLASH_SIZE;
        LibrarySizeK /= 1024;
        IntToStrPad(inpbuf, LibrarySizeK, ' ', 4, 10); strcat(inpbuf, "K (");
        IntToStrPad(inpbuf + strlen(inpbuf), LibraryPercent, ' ', 2, 10); strcat(inpbuf, "%) "); strcat(inpbuf, "Library\r\n");
        MMPrintString(inpbuf);
    }

    IntToStrPad(inpbuf, ((PROG_FLASH_SIZE + SAVEDVARS_FLASH_SIZE) + 512)/1024 - ProgramSize - CFunctSizeK - FontSizeK - SavedVarSizeK - LibrarySizeK, ' ', 4, 10); strcat(inpbuf, "K (");
    IntToStrPad(inpbuf + strlen(inpbuf), 100 - ProgramPercent - CFunctPercent - FontPercent - SavedVarPercent - LibraryPercent, ' ', 2, 10); strcat(inpbuf, "%) Free\r\n");
  MMPrintString(inpbuf);

    MMPrintString("\r\nRAM:\r\n");
    IntToStrPad(inpbuf, VarSize, ' ', 4, 10); strcat(inpbuf, "K (");
    IntToStrPad(inpbuf + strlen(inpbuf), VarPercent, ' ', 2, 10); strcat(inpbuf, "%) ");
    IntToStr(inpbuf + strlen(inpbuf), VarCnt, 10); strcat(inpbuf, " Variable"); strcat(inpbuf, VarCnt == 1 ? "\r\n":"s\r\n");
  MMPrintString(inpbuf);

    IntToStrPad(inpbuf, GeneralSize, ' ', 4, 10); strcat(inpbuf, "K (");
    IntToStrPad(inpbuf + strlen(inpbuf), GeneralPercent, ' ', 2, 10); strcat(inpbuf, "%) General\r\n");
  MMPrintString(inpbuf);

    IntToStrPad(inpbuf, (CurrentRAM + 512)/1024 - VarSize - GeneralSize, ' ', 4, 10); strcat(inpbuf, "K (");
    IntToStrPad(inpbuf + strlen(inpbuf), 100 - VarPercent - GeneralPercent, ' ', 2, 10); strcat(inpbuf, "%) Free\r\n");
  MMPrintString(inpbuf);
#else
    // count the number of lines in the program
    char *p = ProgMemory;
    while(*p != 0xff) {                                             // skip if program memory is erased
        if(*p == 0) p++;                                            // if it is at the end of an element skip the zero marker
        if(*p == 0) break;                                          // end of the program or module
        if(*p == T_NEWLINE) p++;                                    // skip over the newline token
        if(*p == T_LINENBR) p += 3;                                 // skip over the line number
        skipspace(p);
        if(p[0] == T_LABEL) p += p[1] + 2;                          // skip over the label
        while(*p) p++;                                              // look for the zero marking the start of an element
    }
    IntToStr(inpbuf, ((Option.ProgFlashSize - (p - ProgMemory)) + 512)/1024, 10);
    MMPrintString(inpbuf); MMPrintString("K free flash");
#endif
}



/***********************************************************************************************************************
 Public memory management functions
************************************************************************************************************************/

/* all memory allocation (except for the heap) is made by m_alloc()
   memory layout used by MMBasic:

          |--------------------|    <<<   This is the end of the RAM allocated to MMBasic (defined as RAMEND)
          |                    |
          |    MMBasic Heap    |
          |    (grows down)    |
          |                    |
          |--------------------|   <<<   VarTableTop
          |   Variable Table   |
          |     (grows up)     |
          |--------------------|   <<<   vartbl
                                         This is the start of the RAM allocated to MMBasic (defined as RAMBASE)

   m_alloc(type, size) is called when the program is running and whenever the variable table needs to be expanded

   Separately calls are made to GetMemory() and FreeMemory() to allocate or free space on the heap (which grows downward
   towards the variable table).  While the program is running an out of memory situation will occur when the space between
   the heap (growing downwards) and the variable table (growing up) reaches zero.

*/


// every time the variable table is increased this must be called to verify that enough memory is free
void m_alloc(int type, int size) {
    if(type == M_VAR) {
        vartbl = (struct s_vartbl *)RAMBase;
        VarTableTop = (unsigned char *)vartbl + MRoundUp(size);
        if(MBitsGet(VarTableTop) & PUSED) {
            LocalIndex = 0;
            ClearTempMemory();                                          // hopefully this will give us enough memory to print the prompt
            error("Not enough memory");
        }
    }
}



// get some memory from the heap
void *GetMemory(size_t msize) {
    TestStackOverflow();                                            // throw an error if we have overflowed the PIC32's stack
    return getheap(msize);                                          // allocate space
}


// Get a temporary buffer of any size, returns a pointer to the buffer
// The space only lasts for the length of the command or in the case of a sub/fun until it has exited.
// A pointer to the space is also saved in strtmp[] so that the memory can be automatically freed at the end of the command
// StrTmpLocalIndex[] is used to track the sub/fun nesting level at which it was created
void *GetTempMemory(int NbrBytes) {
    if(StrTmpIndex >= MAXTEMPSTRINGS) error("Not enough memory");
    StrTmpLocalIndex[StrTmpIndex] = LocalIndex;
    StrTmp[StrTmpIndex] = GetMemory(NbrBytes);
    TempMemoryIsChanged = true;
    return StrTmp[StrTmpIndex++];
}


// get a temporary string buffer
// this is used by many BASIC string functions.  The space only lasts for the length of the command.
void *GetTempStrMemory(void) {
    return GetTempMemory(STRINGSIZE);
}


// clear any temporary string spaces (these last for just the life of a command) and return the memory to the heap
// this will not clear memory allocated with a local index less than LocalIndex, sub/funs will increment LocalIndex
// and this prevents the automatic use of ClearTempMemory from clearing memory allocated before calling the sub/fun
void ClearTempMemory(void) {
    while(StrTmpIndex > 0) {
        if(StrTmpLocalIndex[StrTmpIndex - 1] >= LocalIndex) {
            StrTmpIndex--;
            FreeMemory(StrTmp[StrTmpIndex]);
            StrTmp[StrTmpIndex] = NULL;
            TempMemoryIsChanged = false;
        } else
            break;
    }
}



void ClearSpecificTempMemory(void *addr) {
    int i;
    for(i = 0; i < StrTmpIndex; i++) {
        if(StrTmp[i] == addr) {
            FreeMemory(addr);
            StrTmp[i] = NULL;
            StrTmpIndex--;
            while(i < StrTmpIndex) {
                StrTmp[i] = StrTmp[i + 1];
                StrTmpLocalIndex[i] = StrTmpLocalIndex[i + 1];
                i++;
            }
            return;
        }
    }
}



void FreeMemory(void *addr) {
    int bits;
    do {
        if(addr < RAMBase || addr >= (void *)RAMEND) return;
        bits = MBitsGet(addr);
        MBitsSet(addr, 0);
        addr += PAGESIZE;
    } while(bits != (PUSED | PLAST));
}



// test the stack for overflow
// this will probably be caused by a fault within MMBasic but it could also be
// caused by a very complex BASIC expression
void TestStackOverflow(void) {
  register unsigned int msp asm("sp");
  if(msp < (unsigned int)RAMEND)
      error("Expression is too complex");
}



void InitHeap(void) {
    int i;
    for(i = 0; i < (MEMSIZE / PAGESIZE) / PAGESPERWORD; i++) mmap[i] = 0;
    for(i = 0; i < MAXTEMPSTRINGS; i++) StrTmp[i] = NULL;
    MBitsSet((unsigned char *)RAMEND, PUSED | PLAST);
    StrTmpIndex = TempMemoryIsChanged = 0;
}




/***********************************************************************************************************************
 Private memory management functions
************************************************************************************************************************/


unsigned int MBitsGet(void *addr) {
    unsigned int i, *p;
    addr -= (int)RAMBase;
    p = &mmap[((unsigned int)addr/PAGESIZE) / PAGESPERWORD];        // point to the word in the memory map
    i = ((((unsigned int)addr/PAGESIZE)) & (PAGESPERWORD - 1)) * PAGEBITS; // get the position of the bits in the word
    return (*p >> i) & ((1 << PAGEBITS) -1);
}



void MBitsSet(void *addr, int bits) {
    unsigned int i, *p;
    addr -= (int)RAMBase;
    p = &mmap[((unsigned int)addr/PAGESIZE) / PAGESPERWORD];        // point to the word in the memory map
    i = ((((unsigned int)addr/PAGESIZE)) & (PAGESPERWORD - 1)) * PAGEBITS; // get the position of the bits in the word
    *p = (bits << i) | (*p & (~(((1 << PAGEBITS) -1) << i)));
}



void *getheap(int size) {
    unsigned int j, n;
    unsigned char *addr;
    j = n = (size + PAGESIZE - 1)/PAGESIZE;                         // nbr of pages rounded up
    for(addr = (unsigned char *)RAMEND -  PAGESIZE; addr >= VarTableTop; addr -= PAGESIZE) {	// RLI: changed addr > VarTableTop to >=
        if(!(MBitsGet(addr) & PUSED)) {
            if(--n == 0) {                                          // found a free slot
                j--;
                MBitsSet(addr + (j * PAGESIZE), PUSED | PLAST);     // show that this is used and the last in the chain of pages
                while(j--) MBitsSet(addr + (j * PAGESIZE), PUSED);  // set the other pages to show that they are used
                memset(addr, 0, size);                              // zero the memory
                return (void *)addr;
            }
        } else
            n = j;                                                  // not enough space here so reset our count
    }
    // out of memory
    LocalIndex = 0;
    ClearTempMemory();                                              // hopefully this will give us enough to print the prompt
    error("Not enough memory");
    return NULL;                                                    // keep the compiler happy
}



int FreeSpaceOnHeap(void) {
    unsigned int nbr;
    unsigned char *addr;
    nbr = 0;
    for(addr = (unsigned char *)RAMEND -  PAGESIZE; addr > VarTableTop; addr -= PAGESIZE)
        if(!(MBitsGet(addr) & PUSED)) nbr++;
    return nbr * PAGESIZE;
}



unsigned int UsedHeap(void) {
    unsigned int nbr;
    unsigned char *addr;
    nbr = 0;
    for(addr = (unsigned char *)RAMEND -  PAGESIZE; addr > VarTableTop; addr -= PAGESIZE)
        if(MBitsGet(addr) & PUSED) nbr++;
    return nbr * PAGESIZE;
}



char *HeapBottom(void) {
    unsigned char *p;
    unsigned char *addr;

    for(p = addr = (unsigned char *)&_stack - (unsigned int)&_min_stack_size -  PAGESIZE; addr > VarTableTop; addr -= PAGESIZE)
        if(MBitsGet(addr) & PUSED) p = addr;
    return (char *)p;
}



#ifdef __xDEBUG
void heapstats(char *m1) {
    int blk, siz, fre, hol;
    unsigned char *addr;
    blk = siz = fre = hol = 0;
    for(addr = (unsigned char *)RAMEND - PAGESIZE; addr >= VarTableTop; addr -= PAGESIZE) {
        if(MBitsGet((void *)addr) & PUSED)
            {siz++; hol = fre;}
        else
            fre++;
        if(MBitsGet((void *)addr) & PLAST)
            blk++;
    }
    dp("%s allocations = %d (using %dKB)  holes = %dKB   free = %dKB", m1, blk, siz/4, hol/4, (fre - hol)/4);
}
#endif

