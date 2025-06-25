/***********************************************************************************************************************
Main.h

Include file that contains the globals and defines for Main.c in MMBasic.

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

// global variables
extern int MMCharPos;
extern char *StartEditPoint;
extern int StartEditChar;
extern char *InterruptReturn;
extern char IgnorePIN;
extern char WatchdogSet;
extern int BasicRunning;
extern int PromptFont, PromptFC, PromptBC;                          // the font and colours selected at the prompt;

// console related functions
extern int MMInkey(void);
extern char MMputchar(char c);


// Use the core timer for misc delays.  The maximum delay is 4 seconds
void uSec(unsigned int us);

// used to control the processor reset
#define RESET_COMMAND       9999                                    // indicates that the reset was caused by the RESET command
#define WATCHDOG_TIMEOUT    9998                                    // reset caused by the watchdog timer
#define PIN_RESTART         9997                                    // reset caused by entering 0 at the PIN prompt
#define RESTART_NOAUTORUN   9996                                    // reset required after changing the LCD or touch config
#define RESTART_DOAUTORUN   9995                                    // reset required by OPTION SET (ie, re runs the program)
extern unsigned int _excep_MagicNumber;
extern unsigned int _excep_code;
extern unsigned int _excep_addr;
extern unsigned int _excep_cause;
// used to determine if the exception occurred during setup
#define CAUSE_NOTHING           0
#define CAUSE_CLOCK             1
#define CAUSE_FILEIO            2
#define CAUSE_KEYBOARD          3
#define CAUSE_RTC               4
#define CAUSE_TOUCH             5
#define CAUSE_MMSTARTUP         6

#if defined(DEBUGMODE)
    void dump(char *p, int nbr);
#endif
