/***********************************************************************************************************************
MMBasic

Memory.h

Include file that contains the globals and defines for memory allocation for MMBasic.

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



/**********************************************************************************
 the C language function associated with commands, functions or operators should be
 declared here
**********************************************************************************/
#if !defined(INCLUDE_COMMAND_TABLE) && !defined(INCLUDE_TOKEN_TABLE)
// format:
//      void cmd_???(void)
//      void fun_???(void)
//      void op_???(void)

void cmd_memory(void);

#endif




/**********************************************************************************
 All command tokens tokens (eg, PRINT, FOR, etc) should be inserted in this table
**********************************************************************************/
#ifdef INCLUDE_COMMAND_TABLE
// the format is:
//    TEXT        TYPE                P  FUNCTION TO CALL
// where type is always T_CMD
// and P is the precedence (which is only used for operators and not commands)

  { "Memory",     T_CMD,              0, cmd_memory },

#endif


/**********************************************************************************
 All other tokens (keywords, functions, operators) should be inserted in this table
**********************************************************************************/
#ifdef INCLUDE_TOKEN_TABLE
// the format is:
//    TEXT        TYPE                P  FUNCTION TO CALL
// where type is T_NA, T_FUN, T_FNA or T_OPER augmented by the types T_STR and/or T_NBR
// and P is the precedence (which is only used for operators)

#endif


#if !defined(INCLUDE_COMMAND_TABLE) && !defined(INCLUDE_TOKEN_TABLE)
// General definitions used by other modules

#ifndef MEMORY_HEADER
#define MEMORY_HEADER

extern char *StrTmp[];                                              // used to track temporary string space on the heap
extern int TempMemoryTop;                                           // this is the last index used for allocating temp memory
extern int TempMemoryIsChanged;                                     // used to prevent unnecessary scanning of strtmp[]

typedef enum _M_Req {M_VIDEO, M_PROG, M_EDIT, M_VAR} M_Req;

extern void m_alloc(int type, int size);
extern void *GetMemory(size_t msize);
extern void *GetTempMemory(int NbrBytes);
extern void *GetTempStrMemory(void);
extern void ClearTempMemory(void);
extern void ClearSpecificTempMemory(void *addr);
extern void TestStackOverflow(void);
extern void FreeMemory(void *addr);
extern void InitHeap(void);
extern char *HeapBottom(void);
extern int FreeSpaceOnHeap(void);
extern unsigned int _stack;
extern unsigned int _splim;
extern unsigned int _heap;
extern unsigned int _min_stack_size;
extern unsigned int _text_begin;


// RAM parameters
// ==============
// The following settings will allow MMBasic to use all the free memory on the S32K1 device.  If you need some RAM for
// other purposes you can declare the space needed as a static variable -or- allocate space to the heap (which
// will reduce the memory available to MMBasic) -or- change the definition of RAMEND.
// NOTE: MMBasic does not use the heap.  It has its own heap which is allocated out of its own memory space.

// The virtual address that MMBasic can start using memory. This value is provided by the __MMBASIC_RAM_BASE__ variable,
// which is rounded up to PAGESIZE (Linker script). It is set in the ft-rpi-sa.c file.
extern void *RAMBase;

// The virtual address that marks the end of the RAM allocated to MMBasic.  This must be rounded up to PAGESIZE.
// This determines maximum amount of RAM that MMBasic can use for program, variables and the heap.
// MMBasic uses just over 5K of RAM for static variables and needs at least 4K for the stack (6K preferably).
// So, using a chip with 32KB, RAMEND could be set to RAMBASE + (22 * 1024).
// The available RAM of S32K1 family, given in the datasheet, is a bit misleading, as the 4k of the FlexRAM is
// added to the overall RAM area. The FlexRAM is, beside the fact that it's not contiguous in the memory map
// with the standard RAM, only supporting 32bit accesses and therefore not usable as standard RAM which supports
// any type of access (8, 16 and 32bit).
// Therefore, the usable RAM for MMBasic is 4k less than the size given in the datasheet.
// The values below are not fixed and may change once more RAM is used by MMBASIC and/or it's implemented ft-RPI-sa
// commands. To get the new values, you need to check the .map file and look for the __MMBASIC_RAM_SIZE__ variable.
// Divide the value by 1024 and round down to the next even value.
#if defined(CPU_S32K144_M4)
#define MAXRAM          50
#elif defined (CPU_S32K142_M4)
#define MAXRAM          18
#elif defined (CPU_S32K146_M4)
#define MAXRAM          114
#endif

#define RAMEND          RAMBase + MAXRAM*1024

// The total amount of memory (in bytes) that MMBasic might use.  This must be a constant (ie, not defined by the Linker)
// Used only to declare a static array to track memory use.  It does not consume much RAM so we set it to the largest possible size for the S32K1 device.
#define MEMSIZE        MAXRAM*1024


// other (minor) memory management parameters
#define PAGESIZE        256                                         // the allocation granularity
#define PAGEBITS        2                                           // nbr of status bits per page of allocated memory, must be a power of 2

#define PUSED           0b01                                        // flag that indicates that the page is in use
#define PLAST           0b10                                        // flag to show that this is the last page in a single allocation

#define PAGESPERWORD    ((sizeof(unsigned int) * 8)/PAGEBITS)
#define MRoundUp(a)     (((a) + (PAGESIZE - 1)) & (~(PAGESIZE - 1)))// round up to the nearest page size


#endif
#endif

