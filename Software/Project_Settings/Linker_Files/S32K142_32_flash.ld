/*
** ###################################################################
**     Processor:           S32K144W with 64 KB SRAM
**     Compiler:            GNU C Compiler
**
**     Abstract:
**         Linker file for the GNU C Compiler
**
** Copyright 2022 Robert Lippmann
** 
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
** 
** 1. Redistributions of source code must retain the above copyright notice,
**    this list of conditions and the following disclaimer.
** 
** 2. Redistributions in binary form must reproduce the above copyright notice,
**    this list of conditions and the following disclaimer in the documentation
**    and/or other materials provided with the distribution.
** 
** ** 3. Neither the name of the copyright holders nor the names of its contributors
**    may be used to endorse or promote products derived from this software
**    without specific prior written permission.
**
** 4. The name MMBasic be used when referring to the interpreter in any
**    documentation and promotional material and the original copyright message
**    be displayed  on the console at startup (additional copyright messages may
**    be added).
** 
** 5. All advertising materials mentioning features or use of this software must
**    display the following acknowledgement: This product includes software
**    developed by Robert Lippmann.
** 
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
** ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE
** FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
** DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
** SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
** CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
** OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
** ###################################################################
*/

/* Entry Point */
ENTRY(Reset_Handler)
/*
To use "new" operator with EWL in C++ project the following symbol shall be defined
*/
/*EXTERN(_ZN10__cxxabiv119__terminate_handlerE)*/


HEAP_SIZE  = DEFINED(__heap_size__)  ? __heap_size__  : 0x00000400;
/*
Need to substract 16 bytes of the stack, as the no_init area is on top of the RAM,
but I want to align the RAMtop to PAGESIZE (memory.h) to optimize RAM usage
*/
STACK_SIZE = DEFINED(__stack_size__) ? __stack_size__ : 0x00001400;

/* If symbol __flash_vector_table__=1 is defined at link time
 * the interrupt vector will not be copied to RAM.
 * Warning: Using the interrupt vector from Flash will not allow
 * INT_SYS_InstallHandler because the section is Read Only.
 */
/* Specify the memory areas */
MEMORY
{
  /* Flash */
  m_interrupts          (RX)  : ORIGIN = 0x00000000, LENGTH = 0x00000400
  m_flash_config        (RX)  : ORIGIN = 0x00000400, LENGTH = 0x00000010
  m_text                (RX)  : ORIGIN = 0x00000410, LENGTH = 0x0003FBF0
  m_dflash				(RX)  : ORIGIN = 0x10000000, LENGTH = 0x0000FFFF
  m_flexram				(RW)  : ORIGIN = 0x14000000, LENGTH = 0x00001000
  /* SRAM_L */
  m_data                (RW)  : ORIGIN = 0x1FFFC000, LENGTH = 0x00004000
  /* SRAM_U */
  m_data_U              (RW)  : ORIGIN = 0x20000000, LENGTH = 0x00003000
}

/* Define output sections */
SECTIONS
{
  /* The startup code goes first into internal flash */
  .interrupts :
  {
    __VECTOR_TABLE = .;
    __interrupts_start__ = .;
    . = ALIGN(4);
    KEEP(*(.isr_vector))     /* Startup code */
    __interrupts_end__ = .;
    . = ALIGN(4);
  } > m_interrupts

  .flash_config :
  {
    . = ALIGN(4);
    KEEP(*(.FlashConfig))    /* Flash Configuration Field (FCF) */
    . = ALIGN(4);
  } > m_flash_config

  __BootLoader_Flash_Start__ = ORIGIN(m_text);										/* This is where the BootLoader code in the Flash starts */
  __BootLoader_Flash_End__ =__BootLoader_Flash_Start__								/* This is where the BootLoader code in the Flash ends */
  							+ (__BootLoader_Ram_End__ - __BootLoader_Ram_Start__);	/* SIZEOF(.BootLoader) can't be used here as no allocation took place yet for
  																					.BootLoader and therefore, I need to calculate the BootLoader length seperately
  																					See GNU-Linker manual pg. 74 for more details */
	
  /* The program code and other data goes into internal flash, starting at __BootLoader_Flash_End__ */
  .text  __BootLoader_Flash_End__ :
  {
    . = ALIGN(4);
    *(.text)                 /* .text sections (code) */
    *(.text*)                /* .text* sections (code) */
    *(.rodata)               /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)              /* .rodata* sections (constants, strings, etc.) */
    *(.glue_7)               /* glue arm to thumb code */
    *(.glue_7t)              /* glue thumb to arm code */
    *(.eh_frame)
    KEEP (*(.init))
    KEEP (*(.fini))
     FILL (0xffffffff);
    . = ALIGN(8);
  } > m_text

  .ARM.extab :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } > m_text

  .ARM :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } > m_text

 .ctors :
  {
    __CTOR_LIST__ = .;
    /* gcc uses crtbegin.o to find the start of
       the constructors, so we make sure it is
       first.  Because this is a wildcard, it
       doesn't matter if the user does not
       actually link against crtbegin.o; the
       linker won't look for a file to match a
       wildcard.  The wildcard also means that it
       doesn't matter which directory crtbegin.o
       is in.  */
    KEEP (*crtbegin.o(.ctors))
    KEEP (*crtbegin?.o(.ctors))
    /* We don't want to include the .ctor section from
       from the crtend.o file until after the sorted ctors.
       The .ctor section from the crtend file contains the
       end of ctors marker and it must be last */
    KEEP (*(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors))
    KEEP (*(SORT(.ctors.*)))
    KEEP (*(.ctors))
    FILL (0xffffffff);
    . = ALIGN(8);
    __CTOR_END__ = .;
  } > m_text

  .dtors :
  {
    __DTOR_LIST__ = .;
    KEEP (*crtbegin.o(.dtors))
    KEEP (*crtbegin?.o(.dtors))
    KEEP (*(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors))
    KEEP (*(SORT(.dtors.*)))
    KEEP (*(.dtors))
    FILL (0xffffffff);
    . = ALIGN(8);
    __DTOR_END__ = .;
  } > m_text

  .preinit_array :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
    FILL (0xffffffff);
    . = ALIGN(8);
  } > m_text

  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
    FILL (0xffffffff);
    . = ALIGN(8);
  } > m_text

  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
    FILL (0xffffffff);
    . = ALIGN(8);
  } > m_text


  __DATA_ROM = .;
	
  .data : AT(__DATA_ROM)
  {
    . = ALIGN(4);
    __RAM_START = .;
    __DATA_RAM = .;
    __data_start__ = .;      /* Create a global symbol at data start. */
    *(.data)                 /* .data sections */
    *(.data*)                /* .data* sections */
    KEEP(*(.jcr*))
    FILL (0xffffffff);
    . = ALIGN(8);
    __data_end__ = .;        /* Define a global symbol at data end. */
  } > m_data

  __DATA_END = __DATA_ROM + (__data_end__ - __data_start__);
  __CODE_ROM = __DATA_END; /* Symbol is used by code initialization. */

  .code : AT(__CODE_ROM)
  {
    . = ALIGN(4);
    __CODE_RAM = .;
    __code_start__ = .;      /* Create a global symbol at code start. */
    __code_ram_start__ = .;
    *(.code_ram)             /* Custom section for storing code in RAM */
    FILL (0xffffffff);
    . = ALIGN(8);
    __code_end__ = .;        /* Define a global symbol at code end. */
    __code_ram_end__ = .;
  } > m_data

  __CODE_END = __CODE_ROM + (__code_end__ - __code_start__);
  __CUSTOM_ROM = __CODE_END;

  /* Custom Section Block that can be used to place data at absolute address. */
  /* Use __attribute__((section (".customSection"))) to place data here. */
  .customSectionBlock  ORIGIN(m_data) : AT(__CUSTOM_ROM)
  {
    __customSection_start__ = .;
    KEEP(*(.customSection))  /* Keep section even if not referenced. */
    FILL (0xffffffff);
    . = ALIGN(8);
    __customSection_end__ = .;
  } > m_data
  __CUSTOM_END = __CUSTOM_ROM + (__customSection_end__ - __customSection_start__);

  /* Uninitialized data section. */
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss section. */
    . = ALIGN(4);
    __BSS_START = .;
    __bss_start__ = .;
    *(.bss)
    *(.bss*)
    *(COMMON)
    FILL (0xffffffff);
    . = ALIGN(8);
    __bss_end__ = .;
    __BSS_END = .;
  } > m_data

  .heap :
  {
    . = ALIGN(4);
    __end__ = .;
    __heap_start__ = .;
    PROVIDE(end = .);
    PROVIDE(_end = .);
    PROVIDE(__end = .);
    __HeapBase = .;
    . += HEAP_SIZE;
    . = ALIGN(256);
    __HeapLimit = .;
    __heap_limit = .;
    __heap_end__ = .;
    __MMBASIC_RAM_BASE__ = .;
  } > m_data

.BootLoader : AT(__BootLoader_Flash_Start__) ALIGN(256)
  {
    __BootLoader_Ram_Start__ = .;    
    __RAM_VECTOR_TABLE = .;
    KEEP(*(.BootLoader.Vectors))	/* Interrupt vectors used by BootLoader */
    KEEP(*(.BootLoader.$func))		/* BootLoader functions code */
    KEEP(*(.BootLoader))			/* BootLoader variables */
    FILL (0xffffffff);
    . = ALIGN(8);
    __BootLoader_Ram_End__ = .;    
  } > m_data_U
  	
  /* Initializes stack on the end of block */
  __StackTop   = ORIGIN(m_data_U) + LENGTH(m_data_U);
  __StackLimit = __StackTop - STACK_SIZE;
  __MMBASIC_MIN_RAM__ = __MMBASIC_RAM_BASE__;
  __MMBASIC_MAX_RAM__ = __StackLimit;
  __MMBASIC_RAM_SIZE__ = (__MMBASIC_MAX_RAM__ - __MMBASIC_MIN_RAM__);
  PROVIDE(__stack = __StackTop);
  __RAM_END = __StackTop;

  .stack __StackLimit :
  {
    . = ALIGN(4);
    __stack_start__ = .;
    . += STACK_SIZE;
    __stack_end__ = .;
  } > m_data_U

.no_init :
{
	. = ALIGN(4);
} > m_flexram
 
.pgm_data :
{
	KEEP(*(.PgmFlashMemory))
	ProgMemory = .;
} > m_dflash

  /* Labels required by EWL */
  __START_BSS = __BSS_START;
  __END_BSS = __BSS_END;
  __SP_INIT = __StackTop;  
  
  .ARM.attributes 0 : { *(.ARM.attributes) }

  ASSERT(__StackLimit >= __HeapLimit, "region m_data overflowed with stack and heap")
}

