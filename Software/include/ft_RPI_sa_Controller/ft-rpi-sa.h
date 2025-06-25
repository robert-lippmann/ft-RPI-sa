/***********************************************************************************************************************
fischertechnik-Raspberry Pi-Stand-Alone Controller

ft_rpi_sa.h

Include file that contains the globals and defines for Misc.c in MMBasic.
These are miscellaneous commands and functions that do not easily sit anywhere else.

Copyright 2022 Robert Lippmann.  All Rights Reserved.

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

void cmd_counter(void);
void cmd_reset(void);
void cmd_output(void);
void cmd_epull(void);

void fun_counter(void);
void fun_eana(void);
void fun_edig(void);
void fun_dipswitch(void);
void fun_einput(void);
void fun_rtcalarm(void);
void fun_distance(void);

#endif




/**********************************************************************************
 All command tokens tokens (eg, PRINT, FOR, etc) should be inserted in this table
**********************************************************************************/
#ifdef INCLUDE_COMMAND_TABLE

  { "Counter(", 		T_CMD | T_FUN,      0, cmd_counter  },
  { "Reset",      	T_CMD,              0, cmd_reset    },
  { "Output",		T_CMD,				0, cmd_output  },
  { "EPull",		T_CMD,				0, cmd_epull   },

#endif


/**********************************************************************************
 All other tokens (keywords, functions, operators) should be inserted in this table
**********************************************************************************/
#ifdef INCLUDE_TOKEN_TABLE

  { "Counter(",     T_FUN | T_INT,        0, fun_counter  },
  { "E_Ana(",		T_FUN | T_INT,        0, fun_eana	},
  { "E_Dig(",     	T_FUN | T_INT,        0, fun_edig   },
  { "DipSwitch",    T_FNA | T_INT,        0, fun_dipswitch},
  { "E_Inp",		T_FNA | T_INT,		  0, fun_einput},
  { "Distance(",	T_FUN | T_NBR,		  0, fun_distance},
  { "RtcAlarm",		T_FNA | T_INT,		  0, fun_rtcalarm},

#endif


#if !defined(INCLUDE_COMMAND_TABLE) && !defined(INCLUDE_TOKEN_TABLE)
    // General definitions used by other modules

	#ifndef FTRPISA_HEADER
	#define FTRPISA_HEADER

  	void InitProcessor(void);
	void __attribute__((section(".flexram.$func"), noinline)) BootLoader(void);
	void AllOutputsOff(void);

	// Global vars provided by this module
	extern unsigned int				ModeSetting;			// Stores the config of the mode Jumpers JP5/JP6
	extern volatile unsigned int	Counter[8];				// These are the four Cn counter inputs
	extern volatile unsigned int   	E_input_adc[9];			// containing the ADC value for each input [5:0]=[E6:E1] / [6]=Poti
	extern volatile unsigned int	E_Input;				// each bit represents an input [E8:E1]=E_Input[7:0]
	extern volatile unsigned int  	DIP_switches;			// containing the debounced DIP-switch settings [4:0]=[DIP5:DIP1]
	extern volatile unsigned int 	DigitalInputPosEdges	// each bit represents a pos edge on Ex (after usage, user need to clean the appropriate bit!)
								  , DigitalInputNegEdges;	// each bit represents a new edge on Ex (after usage, user need to clean the appropriate bit!)
	extern volatile unsigned int	DebounceMask;		// a 1 indicates that the input needs to be debounced [5:0] = [E6:E1]

	#endif
#endif
