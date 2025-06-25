/*****************************************************************************************************************************
fischertechnik-Raspberry Pi-Stand-Alone Controller

Main.c

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


This is the main source file for the ft-RPA-sa project.

Development Environment
    To compile this you need:
     - S32 Design Studio 3.5 with S32K1 family support package

********************************************************************************/

#define DEFINE_PINDEF_TABLE                                         // this will create the PinDef tables defined in IOPorts.h
#include "Version.h"

// global variables used in MMBasic but must be maintained outside of the interpreter
int MMCharPos;
volatile int MMAbort = false;
//char *InterruptReturn = NULL;

char BreakKey = BREAK_KEY;                                          // defaults to CTRL-C.  Set to zero to disable the break function
char IgnorePIN = false;
char WatchdogSet = false;


unsigned int __attribute__((section(".NoInit"))) _excep_MagicNumber;
unsigned int __attribute__((section(".NoInit"))) _excep_code;
unsigned int __attribute__((section(".NoInit"))) _excep_addr;
unsigned int __attribute__((section(".NoInit"))) _excep_cause;

int main(int argc, char* argv[]) {
    int i;
    static int ErrorInPrompt;

    InitProcessor();                                                // do the setup stuff specific to the particular chip

    if((IP_RCM->SSRS & RCM_SSRS_SPOR_MASK) || (_excep_MagicNumber != 0x27fe0811)) // check whether device is starting up first time
    {
    	IP_RCM->SSRS = RCM_SSRS_SPOR_MASK;				// clear POR sticky bit
        // Need to initialize ECC before using these RAM addresses
    	_excep_code = 0;							// clear/init all variables and save MagicNumber
    	_excep_cause = 0;							// to avoid initialization next time
    	_excep_addr = 0;
    	_excep_MagicNumber = 0x27fe0811;
    }

    InitBasic();
    ErrorInPrompt = false;

    WatchdogSet = false;
    if(_excep_code != 0) {                                             // this will only happen if we had a software triggered reset
        if(!(_excep_code == RESTART_NOAUTORUN || _excep_code == RESTART_DOAUTORUN)) {
            if(_excep_code == WATCHDOG_TIMEOUT) {
                WatchdogSet = true;                                 // remember if it was a watchdog timeout
                MMPrintString("\r\n\nWatchdog timeout");
            } else if(_excep_code == RESET_COMMAND) {
            	MMPrintString("\r\n\nRESET command");
            } else if(_excep_code != PIN_RESTART) {
                if(_excep_cause != CAUSE_NOTHING) {
                    if(_excep_cause == CAUSE_MMSTARTUP)
                        MMPrintString("\r\n\nError in MM.STARTUP");
                } else {
                    MMPrintString("\r\n\nCPU exception #");
                    IntToStrPad(inpbuf, _excep_code, '0', 1, 10); MMPrintString(inpbuf);
                    PrepareProgram(false);
                    if(CFunctionFlash != NULL && _excep_addr >= (unsigned int)CFunctionFlash && _excep_addr < (unsigned int)ProgMemory + PROG_FLASH_SIZE) {
                        unsigned int *wrd;
                        char *p;
                        MMPrintString("\r\nIn CFunction ");
                        wrd = (unsigned int *)CFunctionFlash;
                        while(*wrd != 0xffffffff) {
                            if(_excep_addr <= (unsigned int)wrd + wrd[1] + 4) {
                                _excep_addr -= ((unsigned int)wrd + 12);
                                p = (char *)wrd[0];
                                p++; skipspace(p);
                                MMputchar(*p++);
                                while(isnamechar((int)*p)) MMputchar(*p++);
                                break;
                            }
                            wrd++;
                            wrd += (*wrd + 4) / sizeof(unsigned int);
                        }
                    }
                    MMPrintString(" at address 0x");
                    IntToStrPad(inpbuf, _excep_addr, '0', 4, 16); MMPrintString(inpbuf);
                }
            }
            MMPrintString("\r\nProcessor restarted\r\n\n");
            uSec(200000);
        }
    }
    else {
#if defined(DEBUGMODE) || defined(__DEBUG)
        dp("Debug mode.  %d command and %d token slots free.", 127 - CommandTableSize, 127 - TokenTableSize);
#else
        MMPrintString(MES_SIGNON);                                  // print sign on message
        MMPrintString(COPYRIGHT);                                   // print copyright message
        MMPrintString("\r\n");
#endif
    }

    if(setjmp(mark) != 0) {
        // we got here via a long jump which means an error or CTRL-C or the program wants to exit to the command prompt
    	AllOutputsOff();											// First, switch all outputs off once CTRL-C is pressed
        ContinuePoint = nextstmt;                                   // in case the user wants to use the continue command
        *tknbuf = 0;                                                // we do not want to run whatever is in the token buffer
    } else {
        if(_excep_cause != CAUSE_MMSTARTUP) {
            ClearProgram();
            PrepareProgram(true);
            _excep_cause = CAUSE_MMSTARTUP;
#if defined(TEST_CONFIG)
            CurrentLinePtr = inpbuf;
            strcpy(inpbuf, TEST_CONFIG);
            tokenise(true);
            ExecuteProgram(tknbuf);
#endif
            if(FindSubFun("MM.STARTUP", 0) >= 0) ExecuteProgram("MM.STARTUP\0");
            _excep_cause = CAUSE_NOTHING;
            if((Option.Autorun || _excep_code == RESTART_DOAUTORUN) && *ProgMemory == 0x01 && _excep_code != RESTART_NOAUTORUN) {
                CurrentLinePtr = NULL;                                      // do not use the line number in error reporting
                //if(Option.ProgFlashSize != PROG_FLASH_SIZE) ExecuteProgram(ProgMemory + Option.ProgFlashSize);       // run anything that might be in the library
                ExecuteProgram(ProgMemory);                                                                          // then run the program if autorun is on
            }
        }
    }
    _excep_cause = CAUSE_NOTHING;

    while(1) {
        MMAbort = false;
        BreakKey = BREAK_KEY;
        EchoOption = true;
        LocalIndex = 0;                                             // this should not be needed but it ensures that all space will be cleared
        ClearTempMemory();                                          // clear temp string space (might have been used by the prompt)
        CurrentLinePtr = NULL;                                      // do not use the line number in error reporting
        if(MMCharPos > 1) MMPrintString("\r\n");                    // prompt should be on a new line
        while(Option.PIN && !IgnorePIN) {
            _excep_code = PIN_RESTART;
            if(Option.PIN == 99999999)                              // 99999999 is permanent lockdown
                MMPrintString("Console locked, press enter to restart: ");
            else
                MMPrintString("Enter PIN or 0 to restart: ");
            MMgetline(0, inpbuf);
            if(Option.PIN == 99999999) SystemSoftwareReset();
            if(*inpbuf != 0) {
            	for(i=0;i<30;i++)
            	{
            		uSec(100000);	// wait for 100ms (3s in total)
            	}
                i = atoi(inpbuf);
                if(i == 0) SystemSoftwareReset();
                if(i == Option.PIN) {
                    IgnorePIN = true;
                    break;
                }
            }
        }
        _excep_code = 0;
        PrepareProgram(false);
        CurrentLinePtr = NULL;                  // PrepareProgram changes this ptr but with that, some commands
        										// are not usable even on cli (e.g. edit) -> therefore I need to clear this here (added by RLI)
        if(!ErrorInPrompt && FindSubFun("MM.PROMPT", 0) >= 0) {
            ErrorInPrompt = true;
            ExecuteProgram("MM.PROMPT\0");
        } else
            MMPrintString("> ");                                    // print the prompt
        ErrorInPrompt = false;
        MMgetline(0, inpbuf);                                       // get the input
        if(!*inpbuf) continue;                                      // ignore an empty line
        tokenise(true);                                             // turn into executable code
        ExecuteProgram(tknbuf);                                     // execute the line straight away
    }
}

/********************************************************************************************************************************************
 Program memory management
*********************************************************************************************************************************************/

// takes a pointer to RAM containing a program (in clear text) and writes it to program flash in tokenised format
void SaveProgramToFlash(char *pm, int msg) {
    char *p, prevchar = 0, buf[STRINGSIZE];
    int nbr;

    memcpy(buf, tknbuf, STRINGSIZE);                                // save the token buffer because we are going to use it
    IP_LMEM->PCCCR = LMEM_PCCCR_INVW0(1) | LMEM_PCCCR_INVW1(1) | LMEM_PCCCR_GO(1) | LMEM_PCCCR_ENCACHE(0); // disable cache

    FlashWriteInit(ProgMemory, 0);                                  // Initialize for flash write but do not erase any pages

    nbr = 0;                                                        // this is used to count the number of bytes written to flash
    while(*pm) {
        p = inpbuf;
        while(!(*pm == 0 || *pm == '\r' || (*pm == '\n' && prevchar != '\r'))) {
            if(*pm == TAB) {
                do {*p++ = ' ';
                    if((p - inpbuf) >= MAXSTRLEN) goto exiterror2;
                } while((p - inpbuf) % 2);
            } else {
                if(isprint((int)*pm)) {
                    *p++ = *pm;
                    if((p - inpbuf) >= MAXSTRLEN) goto exiterror2;
                }
            }
            prevchar = *pm++;
        }
        if(*pm) prevchar = *pm++;                                   // step over the end of line char but not the terminating zero
        *p = 0;                                                     // terminate the string in inpbuf

        if(*inpbuf == 0 && (*pm == 0 || (!isprint((int)*pm) && pm[1] == 0))) break; // don't save a trailing newline

        tokenise(false);                                            // turn into executable code
        p = tknbuf;
        while(!(p[0] == 0 && p[1] == 0)) {
            FlashWriteByteErase(*p++); nbr++;

            if((int)(flashptr - ProgMemory) >= Option.ProgFlashSize - 9)
                goto exiterror1;
        }
        FlashWriteByteErase(0); nbr++;                              // terminate that line in flash
    }
    while(flashptr<(ProgMemory+12)) FlashWriteByteErase(0);
    FlashWriteByteErase(0); FlashWriteByteErase(0);                 // terminate the program in flash
    FlashWriteByteErase(0xff);                                      // this is for routines that scan the flash and expected the program area to be terminated with 0xff
    FlashWriteClose();
    IP_LMEM->PCCCR = LMEM_PCCCR_INVW0(1) | LMEM_PCCCR_INVW1(1) | LMEM_PCCCR_GO(1) | LMEM_PCCCR_ENCACHE(1); // re-enable cache

    if(msg) {                                                       // if requested by the caller, print an informative message
        if(MMCharPos > 1) MMPrintString("\r\n");                    // message should be on a new line
        MMPrintString("Saved ");
        IntToStr(tknbuf, nbr + 3, 10);
        MMPrintString(tknbuf);
        MMPrintString(" bytes\r\n");
    }
    memcpy(tknbuf, buf, STRINGSIZE);                                // restore the token buffer in case there are other commands in it
    return;

    // we only get here if we have run out of memory
    exiterror1:
        FlashWriteByteErase(0); FlashWriteByteErase(0); FlashWriteByteErase(0);    // terminate the program in flash
        FlashWriteClose();
        IP_LMEM->PCCCR = LMEM_PCCCR_INVW0(1) | LMEM_PCCCR_INVW1(1) | LMEM_PCCCR_GO(1) | LMEM_PCCCR_ENCACHE(1); // re-enable cache
        error("Not enough memory");

    // we only get here if a line was too long
    exiterror2:
        FlashWriteByteErase(0); FlashWriteByteErase(0); FlashWriteByteErase(0);    // terminate the program in flash
        FlashWriteClose();
        IP_LMEM->PCCCR = LMEM_PCCCR_INVW0(1) | LMEM_PCCCR_INVW1(1) | LMEM_PCCCR_GO(1) | LMEM_PCCCR_ENCACHE(1); // re-enable cache
        error("Line is too long");
}




// put a character out to the serial console
char MMputchar(char c) {
    putConsole(c);
    if(isprint(c)) MMCharPos++;
    if(c == '\r') {
        MMCharPos = 1;
    }
    return c;
}



/*****************************************************************************************
The vt100 escape code sequences
===============================
3 char codes            Arrow Up    esc [ A
                        Arrow Down  esc [ B
                        Arrow Right esc [ C
                        Arrow Left  esc [ D

4 char codes            Home        esc [ 1 ~
                        Insert      esc [ 2 ~
                        Del         esc [ 3 ~
                        End         esc [ 4 ~
                        Page Up     esc [ 5 ~
                        Page Down   esc [ 6 ~

5 char codes            F1          esc [ 1 1 ~
                        F2          esc [ 1 2 ~
                        F3          esc [ 1 3 ~
                        F4          esc [ 1 4 ~
                        F5          esc [ 1 5 ~         note the
                        F6          esc [ 1 7 ~         disconnect
                        F7          esc [ 1 8 ~
                        F8          esc [ 1 9 ~
                        F9          esc [ 2 0 ~
                        F10         esc [ 2 1 ~         note the
                        F11         esc [ 2 3 ~         disconnect
                        F12         esc [ 2 4 ~

                        SHIFT-F3    esc [ 2 5 ~         used in the editor

 Linux terminal emulators use the older escape codes for F1-F4 (was PF1-PF4 on the original VT100).
 These use <esc>OP through to <esc>OS rather than the [ sequence.
 
*****************************************************************************************/

// check if there is a keystroke waiting in the buffer and, if so, return with the char
// returns -1 if no char waiting
// the main work is to check for vt100 escape code sequences and map to Maximite codes
int MMInkey(void) {
    unsigned int c = -1;                                            // default no character
    unsigned int tc = -1;                                           // default no character
    unsigned int ttc = -1;                                          // default no character
    static unsigned int c1 = -1;
    static unsigned int c2 = -1;
    static unsigned int c3 = -1;
    static unsigned int c4 = -1;

    if(c1 != -1) {                                                  // check if there are discarded chars from a previous sequence
        c = c1; c1 = c2; c2 = c3; c3 = c4; c4 = -1;                 // shuffle the queue down
        return c;                                                   // and return the head of the queue
    }

    c = getConsole();                                               // do discarded chars so get the char
    if(c == 0x1b) {
        InkeyTimer = 0;                                             // start the timer
        while((c = getConsole()) == -1 && InkeyTimer < 30);         // get the second char with a delay of 30mS to allow the next char to arrive
        if(c == 'O') {                                              // support for many Linux terminal emulators (Peter Mather)
            while((c = getConsole()) == -1 && InkeyTimer < 50);     // delay some more to allow the final chars to arrive, even at 1200 baud
            if(c == 'P') return F1;
            if(c == 'Q') return F2;
            if(c == 'R') return F3;
            if(c == 'S') return F4;
            if(c == 'F') return END;								// minicom in VT102 mode
            if(c == 'P') {
                while((tc = getConsole()) == -1 && InkeyTimer < 70);// more delay for more chars to arrive
                if(tc == 'R') return F3 + 0x20;
                c1 = 'O'; c2 = c; c3 = tc; return 0x1b;             // not a valid 4 char code
            }
            c1 = 'O'; c2 = c; return 0x1b;                          // not a valid code
        }
        if(c != '[') { c1 = c; return 0x1b; }                       // must be a square bracket
        while((c = getConsole()) == -1 && InkeyTimer < 50);         // get the third char with delay
        if(c == 'A') return UP;                                     // the arrow keys are three chars
        if(c == 'B') return DOWN;
        if(c == 'C') return RIGHT;
        if(c == 'D') return LEFT;
        if(c < '1' && c > '6') { c1 = '['; c2 = c; return 0x1b; }   // the 3rd char must be in this range
        while((tc = getConsole()) == -1 && InkeyTimer < 70);        // delay some more to allow the final chars to arrive, even at 1200 baud
        if(tc == '~') {                                             // all 4 char codes must be terminated with ~
            if(c == '1') return HOME;
            if(c == '2') return INSERT;
            if(c == '3') return DEL;
//            if(c == '4') return END;
            if(c == '5') return PUP;
            if(c == '6') return PDOWN;
            c1 = '['; c2 = c; c3 = tc; return 0x1b;                 // not a valid 4 char code
        }
        while((ttc = getConsole()) == -1 && InkeyTimer < 90);       // get the 5th char with delay
        if(ttc == '~') {                                            // must be a ~
            if(c == '1') {
                if(tc >='1' && tc <= '4') return F1 + (tc - '1');   // F1 to F4 (was 5 instead of 4)
                if(tc >='6' && tc <= '9') return F5 + (tc - '6');   // F5 to F8 (was 7 instead of 6 and F6 instead of F5!)
            }
            if(c == '2') {
                if(tc =='0' || tc == '1') return F9 + (tc - '0');   // F9 and F10
                if(tc =='3' || tc == '4') return F11 + (tc - '3');  // F11 and F12
                if(tc =='5') return F3 + 0x20;                      // SHIFT-F3
            }
        }
        // nothing worked so bomb out
        c1 = '['; c2 = c; c3 = tc; c4 = ttc;
        return 0x1b;
    }
    return c;
}

void DefaultISR(void) {
    _excep_code = (_excep_code & 0x0000007C) >> 2;

    SystemSoftwareReset();        // this will restart the processor
}

#if defined(DEBUGMODE)
// dump a memory area to the console
// for debugging
void dump(char *p, int nbr) {
    char buf1[80], buf2[80], *b1, *b2, *pt;
    b1 = buf1; b2 = buf2;
    MMPrintString("   addr    0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F    0123456789ABCDEF\r\n");
    b1 += sprintf(b1, "%8x: ", (unsigned int)p);
    for(pt = p; (unsigned int)pt % 16 != 0; pt--) {
        b1 += sprintf(b1, "   ");
        b2 += sprintf(b2, " ");
    }
    while(nbr > 0) {
        b1 += sprintf(b1, "%02x ", *p);
        b2 += sprintf(b2, "%c", (*p >= ' ' && *p < 0x7f) ? *p : '.');
        p++;
        nbr--;
        if((unsigned int)p % 16 == 0) {
            MMPrintString(buf1);
            MMPrintString("   ");
            MMPrintString(buf2);
            b1 = buf1; b2 = buf2;
            b1 += sprintf(b1, "\r\n%8x: ", (unsigned int)p);
        }
    }
    if(b2 != buf2) {
        MMPrintString(buf1);
        MMPrintString("   ");
        for(pt = p; (unsigned int)pt % 16 != 0; pt++) {
            MMPrintString("   ");
        }
      MMPrintString(buf2);
    }
    MMPrintString("\r\n");
}
#endif

// a crude memory dump that does not use sprintf()
void cdump(char *p, int nbr) {
    while(nbr--) {
        if(((int)p & 0b11111) == 0) MMPrintString("\r\n");
        if(*p == 0)
            MMPrintString("= ");
        else if(*p == T_NEWLINE)
            MMPrintString("@ ");
        else if(*p == T_LINENBR)
            MMPrintString("% ");
        else if(isprint((int)*p))
            { MMputchar(*p); MMputchar(' '); }
        else MMPrintString(". ");
        p++;
    }
    MMPrintString("\r\n");
}



