/***********************************************************************************************************************
MMBasic

Misc.c

Handles the a few miscellaneous functions for the OPTIONS command.

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


///////////////////////////////////////////////////////////////////////////////////////////////
// constants and functions used in the OPTION LIST command
char *CaseList[] = {"", "LOWER", "UPPER"};

void PO(char *s) {
    MMPrintString("OPTION "); MMPrintString(s); MMPrintString(" ");
}

void PInt(int n) {
    char s[20];
    IntToStr(s, n, 10);
    MMPrintString(s);
}

void PIntComma(int n) {
    MMPrintString(", "); PInt(n);
}

void PO2Str(char *s1, char *s2) {
    PO(s1); MMPrintString(s2); MMPrintString("\r\n");
}


void PO2Int(char *s1, int n) {
    PO(s1); PInt(n); MMPrintString("\r\n");
}

void PO3Int(char *s1, int n1, int n2) {
    PO(s1); PInt(n1); PIntComma(n2); MMPrintString("\r\n");
}
///////////////////////////////////////////////////////////////////////////////////////////////


void OtherOptions(void) {
  char *tp;

  tp = checkstring(cmdline, "RESET");
  if(tp) {
        ResetAllOptions();
      goto saveandreset;
  }

    tp = checkstring(cmdline, "CONSOLE");
  if(tp) {
      if(checkstring(tp, "ECHO"))         { EchoOption = true; return; }
      if(checkstring(tp, "NOECHO"))   { EchoOption = false; return; }
  }
    tp = checkstring(cmdline, "LIST");
    if(tp) {
        if(Option.Autorun == true) PO2Str("AUTORUN", "ON");
        if(Option.Baudrate != CONSOLE_BAUDRATE) PO2Int("BAUDRATE", Option.Baudrate);
        if(Option.ColourCode == true) PO2Str("COLOURCODE", "ON");
        if(Option.Listcase != CONFIG_TITLE) PO2Str("CASE", CaseList[(int)Option.Listcase]);
        if(Option.Tab != 2) PO2Int("TAB", Option.Tab);
        if(Option.Height != 24 || Option.Width != 80) PO3Int("DISPLAY", Option.Height, Option.Width);
        return;
    }

    error("Invalid option");

saveandreset:
    if(!CurrentLinePtr) {
       if(SaveOptions()) {
           uSec(200000);
           _excep_code = RESTART_NOAUTORUN;
            SystemSoftwareReset();                                            // this will restart the processor
       }
    }
}

