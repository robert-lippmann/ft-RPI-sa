/***********************************************************************************************************************
MMBasic

SerialFileIO.c

Handles all the serial file input/output in MMBasic.

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

#include "Version.h"

// list of open files
// it is a union of an unsigned int and a FsFat file pointer.
// if it is a COM port the unsigned int will be a number from 1 to 4 being the COMx: number
// if the unsigned int is greater than the number of COM ports it must be a FsFat file pointer (MM+ only)
// NOTE: the first entry OpenFileTable[0] is not used so the range of valid entries is 1 to MAXOPENFILES
union uFileTable FileTable[MAXOPENFILES + 1];

void cmd_open(void) {
  int fnbr;
  char *fname;
  char ss[3];                                                       // this will be used to split up the argument line

  ss[0] = tokenAS;
    ss[1] = tokenFOR;
  ss[2] = 0;
  {                                                                 // start a new block
      getargs(&cmdline, 5, ss);                                     // getargs macro must be the first executable stmt in a block
        if(!(argc == 3 || argc == 5)) error("Invalid Syntax");
      fname = getCstring(argv[0]);

      // check that it is a serial port that we are opening
        if(*argv[2] == '#') argv[2]++;
        fnbr = getint(argv[2], 1, MAXOPENFILES);
        if(FileTable[fnbr].com != 0) error("Already open");
        SerialOpen(fname);
        FileTable[fnbr].com = fname[3] - '0';
      }
}



void cmd_close(void) {
  int i, fnbr;
  getargs(&cmdline, (MAX_ARG_COUNT * 2) - 1, ",");                 // getargs macro must be the first executable stmt in a block
  if((argc & 0x01) == 0) error("Syntax");
  for(i = 0; i < argc; i += 2) {
      if(*argv[i] == '#') {
    	  argv[i]++;
      }
      fnbr = getint(argv[i], 1, MAXOPENFILES);
        if(FileTable[fnbr].com == 0) {
        	error("File number is not open");
        }
        while(SerialTxStatus(FileTable[fnbr].com) && !MMAbort);     // wait for anything in the buffer to be transmitted
        SerialClose(FileTable[fnbr].com);
        FileTable[fnbr].com = 0;
  }
}


void fun_inputstr(void) {
  int i, nbr, fnbr;
  getargs(&ep, 3, ",");
  if(argc != 3) error("Syntax");
  sret = GetTempStrMemory();                                        // this will last for the life of the command
  nbr = getint(argv[0], 1, MAXSTRLEN);
  if(*argv[2] == '#') argv[2]++;
  fnbr = getinteger(argv[2]);
    if(fnbr == 0) {                                                 // accessing the console
        for(i = 1; i <= nbr && kbhitConsole(); i++)
            sret[i] = getConsole();                                 // get the char from the console input buffer and save in our returned string
    } else {
        if(fnbr < 1 || fnbr > MAXOPENFILES) error("File number");
        if(FileTable[fnbr].com == 0) error("File number is not open");
        targ = T_STR;
         for(i = 1; i <= nbr && SerialRxStatus(FileTable[fnbr].com); i++)
            sret[i] = SerialGetchar(FileTable[fnbr].com);           // get the char from the serial input buffer and save in our returned string
    }
    *sret = i - 1;
}



void fun_eof(void) {
    int fnbr;
  getargs(&ep, 1, ",");
  if(argc == 0) error("Syntax");
  if(*argv[0] == '#') argv[0]++;
  fnbr = getinteger(argv[0]);
    iret = MMfeof(fnbr);
    targ = T_INT;
}



void fun_loc(void) {
  int fnbr;
  getargs(&ep, 1, ",");
  if(argc == 0) error("Syntax");
  if(*argv[0] == '#') argv[0]++;
  fnbr = getinteger(argv[0]);
    if(fnbr == 0)                                                   // accessing the console
        iret = kbhitConsole();
    else {
        if(fnbr < 1 || fnbr > MAXOPENFILES) error("File number");
        if(FileTable[fnbr].com == 0) error("File number is not open");
        iret = SerialRxStatus(FileTable[fnbr].com);
    }
    targ = T_INT;
}



void fun_lof(void) {
  int fnbr;
  getargs(&ep, 1, ",");
  if(argc == 0) error("Syntax");
  if(*argv[0] == '#') argv[0]++;
  fnbr = getinteger(argv[0]);
    if(fnbr == 0)                                                   // accessing the console
        iret = 0;
    else {
        if(fnbr < 1 || fnbr > MAXOPENFILES) error("File number");
        if(FileTable[fnbr].com == 0) error("File number is not open");
        iret = (TX_BUFFER_SIZE - SerialTxStatus(FileTable[fnbr].com));
    }
    targ = T_INT;
}



/*******************************************************************************************
********************************************************************************************

Utility routines for the file I/O commands in MMBasic

********************************************************************************************
********************************************************************************************/



// get a line from the keyboard or a serial file handle
// filenbr == 0 means the console input
void MMgetline(int filenbr, char *p) {
    int c, nbrchars = 0;
    char *tp;

    while(1) {

        CheckAbort();                                                 // jump right out if CTRL-C

         c = MMfgetc(filenbr);
        

        // #if defined(MX470)
        //    if(FileTable[filenbr].com > MAXCOMPORTS && FileEOF(filenbr)) break;
        // #endif

        if(c == -1) continue;                                       // keep looping if there are no chars

        // if this is the console, check for a programmed function key and insert the text
        if(filenbr == 0) {
            tp = NULL;
            if(c == F2)  tp = "RUN";
            if(c == F3)  tp = "LIST";
            if(c == F4)  tp = "EDIT";
            if(c == F10) tp = "AUTOSAVE";
            if(c == F11) tp = "XMODEM RECEIVE";
            if(c == F12) tp = "XMODEM SEND";
            if(tp) {
                strcpy(p, tp);
                if(EchoOption) { MMPrintString(tp); MMPrintString("\r\n"); }
                return;
            }
        }

        if(c == '\t') {                                               // expand tabs to spaces
            do {
                if(++nbrchars > MAXSTRLEN) error("Line is too long");
                *p++ = ' ';
                if(filenbr == 0 && EchoOption) putConsole(' ');
            } while(nbrchars % Option.Tab);
            continue;
        }

        if(c == '\b') {                                               // handle the backspace
            if(nbrchars) {
                if(filenbr == 0 && EchoOption) MMPrintString("\b \b");
                nbrchars--;
                p--;
            }
            continue;
        }

        if(c == '\n') {                                             // what to do with a newline
                break;                                              // a newline terminates a line (for a file)
        }

        if(c == '\r') {
            if(filenbr == 0 && EchoOption) {
                MMPrintString("\r\n");
                break;                                              // on the console this means the end of the line - stop collecting
            } else
                continue ;                                          // for files loop around looking for the following newline
        }

        if(isprint(c)) {
            if(filenbr == 0 && EchoOption) putConsole(c);             // The console requires that chars be echoed
        }
        if(++nbrchars > MAXSTRLEN) error("Line is too long");         // stop collecting if maximum length
        *p++ = c;                                                     // save our char
    }
    *p = 0;
}


// get a character from a file or the console
// if fnbr == 0 then get the char from the console
// otherwise the COM port or file opened as #fnbr
int MMfgetc(int fnbr) {
  int ch;
  if(fnbr == 0) return MMgetchar();                                 // accessing the console
    if(fnbr < 1 || fnbr > MAXOPENFILES) error("File number");
    if(FileTable[fnbr].com == 0) error("File number is not open");
    ch = SerialGetchar(FileTable[fnbr].com);                        // get the char from the serial port
  return ch;
}



// send a character to a file or the console
// if fnbr == 0 then send the char to the console
// otherwise the COM port or file opened as #fnbr
char MMfputc(char c, int fnbr) {
  if(fnbr == 0) return MMputchar(c);                                // accessing the console
    if(fnbr < 1 || fnbr > MAXOPENFILES) error("File number");
    if(FileTable[fnbr].com == 0) error("File number is not open");
    return SerialPutchar(FileTable[fnbr].com, c);                   // send the char to the serial port
}



int MMfeof(int fnbr) {
  if(fnbr == 0) return (kbhitConsole() == 0);                       // accessing the console
    if(fnbr < 1 || fnbr > MAXOPENFILES) error("File number");
    if(FileTable[fnbr].com == 0) error("File number is not open");
  return SerialRxStatus(FileTable[fnbr].com) == 0;
}



void CloseAllFiles(void) {
  int i;
  for(i = 1; i <= MAXOPENFILES; i++) {
        if(FileTable[i].com != 0) {
            SerialClose(FileTable[i].com);
        FileTable[i].com = 0;
        }
  }
}

