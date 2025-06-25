/***********************************************************************************************************************
Configuration.h

Include file that contains the configuration details for the Micromite using MMBasic.

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

// The main clock frequency the CPU core is running at
#define CLOCKFREQ     (80000000L)                   // This is set in in Configuration Bits.h

#define CONSOLE_BAUDRATE    115200

#define nop __asm__ ("NOP")

#define forever 1
#define true  1
#define false   0

#define MMFLOAT float
#define STR_SIG_DIGITS 5                            // number of significant digits to use when converting MMFLOAT to a string
#define FLOAT_ROUNDING_LIMIT 0x7fffff               // used to limit rounding for large numbers in FloatToInt64()


#define BOOL_ALREADY_DEFINED

#define MAXVARLEN           32                      // maximum length of a variable name
#define MAXSTRLEN           255                     // maximum length of a string
#define STRINGSIZE          256                     // must be 1 more than MAXSTRLEN.  3 of these buffers are statically created
#define MAXERRMSG           64                      // max error msg size (MM.ErrMsg$ is truncated to this)
#define MAXDIM              8                       // maximum nbr of dimensions to an array

#define MAXFORLOOPS         10                      // maximum nbr of nested for-next loops, each entry uses 17 bytes
#define MAXDOLOOPS          10                      // maximum nbr of nested do-loops, each entry uses 12 bytes
#define MAXGOSUB            50                      // maximum nbr of nested gosubs and defined subs/functs, each entry uses 8 bytes
#define MAX_MULTILINE_IF    10                      // maximum nbr of nested multiline IFs, each entry uses 8 bytes
#define MAXTEMPSTRINGS      64                      // maximum nbr of temporary strings allowed, each entry takes up 5 bytes
#define MAXSUBFUN           100                     // maximum nbr of defined subroutines or functions in a program. each entry takes up 4 bytes
#define NBRSETTICKS         4                       // the number of SETTICK interrupts available

#define BREAK_KEY           3                       // the default value (CTRL-C) for the break key.  Reset at the command prompt.

// define the maximum number of arguments to PRINT, INPUT, WRITE, ON, DIM, ERASE, DATA and READ
// each entry uses zero bytes.  The number is limited by the length of a command line
#define MAX_ARG_COUNT       50

// size of the console terminal emulator's screen
#define SCREENWIDTH     80
#define SCREENHEIGHT    24                          // this is the default and it can be changed using the OPTION command

