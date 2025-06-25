/***********************************************************************************************************************
MMBasic

MM_Misc.c

Handles all the miscellaneous commands and functions in MMBasic.  These are commands and functions that do not
comfortably fit anywhere else.

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

extern void CallCFuncInt(void);                                     // this is implemented in CFunction.c
extern unsigned int CFuncInt;                                       // we should call the CFunction mSec function if this is non zero

struct s_inttbl inttbl[NBRINTERRUPTS];
char *InterruptReturn;

int TickPeriod[NBRSETTICKS];
volatile int TickTimer[NBRSETTICKS];
char *TickInt[NBRSETTICKS];
char *OnKeyGOSUB = NULL;

char EchoOption = true;

unsigned int ClockSpeed = CLOCKFREQ;
unsigned int BusSpeed;

int OptionErrorCheck;



// this is invoked as a command (ie, TIMER = 0)
// search through the line looking for the equals sign and step over it,
// evaluate the rest of the command and save in the timer
void cmd_timer(void) {
  while(*cmdline && tokenfunction(*cmdline) != op_equal) cmdline++;
  if(!*cmdline) error("Syntax");
  mSecTimer = getinteger(++cmdline);
}



// this is invoked as a function
void fun_timer(void) {
  iret = mSecTimer;
    targ = T_INT;
}



void cmd_pause(void) {
    static int interrupted = false;
    MMFLOAT f;

    f = getnumber(cmdline);                                         // get the pulse width
    if(f < 0) error("Number out of bounds");
    if(f < 0.05) return;

    if(f < 1.5) {
        uSec(f * 1000);                                             // if less than 1.5mS do the pause right now
        return;                                                     // and exit straight away
      }

    if(InterruptReturn == NULL) {
        // we are running pause in a normal program
        // first check if we have reentered (from an interrupt) and only zero the timer if we have NOT been interrupted.
        // This means an interrupted pause will resume from where it was when interrupted
        if(!interrupted) PauseTimer = 0;
        interrupted = false;

        while(PauseTimer < FloatToInt32(f)) {
            CheckAbort();
            if(check_interrupt()) {
                // if there is an interrupt fake the return point to the start of this stmt
                // and return immediately to the program processor so that it can send us off
                // to the interrupt routine.  When the interrupt routine finishes we should reexecute
                // this stmt and because the variable interrupted is static we can see that we need to
                // resume pausing rather than start a new pause time.
                while(*cmdline && *cmdline != cmdtoken) cmdline--;  // step back to find the command token
                InterruptReturn = cmdline;                          // point to it
                interrupted = true;                                 // show that this stmt was interrupted
                return;                                             // and let the interrupt run
            }
        }
          interrupted = false;
    }
    else {
        // we are running pause in an interrupt, this is much simpler but note that
        // we use a different timer from the main pause code (above)
        IntPauseTimer = 0;
        while(IntPauseTimer < FloatToInt32(f)) CheckAbort();
    }
//    #if defined(MX470)
//      InPause = false;
//    #endif
}



// this is invoked as a command (ie, date$ = "6/7/2010")
// search through the line looking for the equals sign and step over it,
// evaluate the rest of the command, split it up and save in the system counters
void cmd_date(void) {
  char *arg;
  int d, m, y;
  while(*cmdline && tokenfunction(*cmdline) != op_equal) cmdline++;
  if(!*cmdline) error("Syntax");
  ++cmdline;
  arg = getCstring(cmdline);
  {
      getargs(&arg, 5, "-/");                                       // this is a macro and must be the first executable stmt in a block
      if(argc != 5) error("Syntax");
      d = atoi(argv[0]);
      m = atoi(argv[2]);
      y = atoi(argv[4]);
      if(y >= 0 && y < 100) y += 2000;
      if(d < 1 || d > 31 || m < 1 || m > 12 || y < 2000 || y > 2999) error("Invalid date");

      //mT4IntEnable(0);                                              // disable the timer interrupt to prevent any conflicts while updating
      day = d;
      month = m;
      year = y;
      //mT4IntEnable(1);                                              // enable interrupt
  }
}

// this is invoked as a function
void fun_date(void) {
    sret = GetTempStrMemory();                                      // this will last for the life of the command
  //mT4IntEnable(0);                                                  // disable the timer interrupt to prevent any conflicts while updating
    IntToStrPad(sret, day, '0', 2, 10);
    sret[2] = '-'; IntToStrPad(sret + 3, month, '0', 2, 10);
    sret[5] = '-'; IntToStr(sret + 6, year, 10);
  //mT4IntEnable(1);                                                  // enable interrupt
  CtoM(sret);
    targ = T_STR;
}



// this is invoked as a command (ie, time$ = "6:10:45")
// search through the line looking for the equals sign and step over it,
// evaluate the rest of the command, split it up and save in the system counters
void cmd_time(void) {
  char *arg;
  int h = 0;
  int m = 0;
  int s = 0;

  while(*cmdline && tokenfunction(*cmdline) != op_equal) cmdline++;
  if(!*cmdline) error("Syntax");
  ++cmdline;
  arg = getCstring(cmdline);
  {
      getargs(&arg, 5, ":");                                        // this is a macro and must be the first executable stmt in a block
      if(argc%2 == 0) error("Syntax");
      h = atoi(argv[0]);
      if(argc >= 3) m = atoi(argv[2]);
      if(argc == 5) s = atoi(argv[4]);
      if(h < 0 || h > 23 || m < 0 || m > 59 || s < 0 || s > 59) error("Invalid time");
      DISABLE_INTERRUPTS();
      hour = h;
      minute = m;
      second = s;
      SecondsTimer = 0;
      ENABLE_INTERRUPTS();
    }
}




// this is invoked as a function
void fun_time(void) {
  sret = GetTempStrMemory();                                        // this will last for the life of the command
  DISABLE_INTERRUPTS();
  IntToStrPad(sret, hour, '0', 2, 10);
  sret[2] = ':'; IntToStrPad(sret + 3, minute, '0', 2, 10);
  sret[5] = ':'; IntToStrPad(sret + 6, second, '0', 2, 10);
  ENABLE_INTERRUPTS();
  CtoM(sret);
  targ = T_STR;
}



void cmd_ireturn(void){
  if(InterruptReturn == NULL) error("Not in interrupt");
  checkend(cmdline);
  nextstmt = InterruptReturn;
  InterruptReturn = NULL;
  if(LocalIndex) ClearVars(LocalIndex--);                           // delete any local variables
  TempMemoryIsChanged = true;                                       // signal that temporary memory should be checked
  *CurrentInterruptName = 0;                                        // for static vars we are not in an interrupt
}


// set up the tick interrupt
void cmd_settick(void){
  int period;
  int irq;
  getargs(&cmdline, 5, ",");
  if(!(argc == 3 || argc == 5)) error("Argument count");
  period = getint(argv[0], 0, INT_MAX);
  if(argc == 5)
      irq = getint(argv[4], 1, NBRSETTICKS) - 1;
  else
      irq = 0;
  if(period == 0) {
      TickInt[irq] = NULL;                                          // turn off the interrupt
    } else {
      TickPeriod[irq] = period;
      TickInt[irq] = GetIntAddress(argv[2]);                        // get a pointer to the interrupt routine
      TickTimer[irq] = 0;                                           // set the timer running
      //InterruptUsed = true;

  }
}



void cmd_watchdog(void) {
    int i;

    if(checkstring(cmdline, "OFF") != NULL) {
        WDTimer = 0;
    } else {
        i = getint(cmdline, 1, INT_MAX);
        WDTimer = i;
    }
}


void fun_restart(void) {
    iret = WatchdogSet;
    targ = T_INT;
}


void cmd_option(void) {
  char *tp;

    // this is used by options that need a reboot after setting
    //OptionErrorCheck = CP_IGNORE_INUSE;
    //if(CurrentLinePtr) OptionErrorCheck |= CP_NOABORT;              // do not abort on error if we are setting the option in a program


  tp = checkstring(cmdline, "BASE");
  if(tp) {
      if(DimUsed) error("Must be before DIM or LOCAL");
      OptionBase = getint(tp, 0, 1);
      return;
  }

  tp = checkstring(cmdline, "EXPLICIT");
  if(tp) {
      if(varcnt != 0) error("Variables already defined");
      OptionExplicit = true;
      return;
  }

    tp = checkstring(cmdline, "DEFAULT");
  if(tp) {
      if(checkstring(tp, "INTEGER"))  { DefaultType = T_INT;    return; }
      if(checkstring(tp, "FLOAT"))    { DefaultType = T_NBR;    return; }
      if(checkstring(tp, "STRING")) { DefaultType = T_STR;  return; }
      if(checkstring(tp, "NONE"))       { DefaultType = T_NOTYPE;     return; }
  }

  tp = checkstring(cmdline, "BREAK");
  if(tp) {
      BreakKey = getinteger(tp);
      return;
  }

    tp = checkstring(cmdline, "AUTORUN");
  if(tp) {
      if(checkstring(tp, "ON"))   { Option.Autorun = true; SaveOptions(); return; }
      if(checkstring(tp, "OFF"))      { Option.Autorun = false; SaveOptions(); return;  }
  }

    tp = checkstring(cmdline, "CASE");
  if(tp) {
      if(checkstring(tp, "LOWER"))    { Option.Listcase = CONFIG_LOWER; SaveOptions(); return; }
      if(checkstring(tp, "UPPER"))    { Option.Listcase = CONFIG_UPPER; SaveOptions(); return; }
      if(checkstring(tp, "TITLE"))    { Option.Listcase = CONFIG_TITLE; SaveOptions(); return; }
  }

    tp = checkstring(cmdline, "TAB");
  if(tp) {
      if(checkstring(tp, "2"))        { Option.Tab = 2; SaveOptions(); return; }
      if(checkstring(tp, "4"))        { Option.Tab = 4; SaveOptions(); return; }
      if(checkstring(tp, "8"))        { Option.Tab = 8; SaveOptions(); return; }
  }

    tp = checkstring(cmdline, "BAUDRATE");
  if(tp) {
        int i;
        i = getint(tp, 100, BusSpeed/130);
        Option.Baudrate = i;
        SaveOptions();
        initSerialConsole();
      return;
  }

    tp = checkstring(cmdline, "PIN");
  if(tp) {
      int i;
      i = getint(tp, 0, 99999999);
        Option.PIN = i;
        SaveOptions();
      return;
  }

    tp = checkstring(cmdline, "DISPLAY");
  if(tp) {
        getargs(&tp, 3, ",");
        Option.Height = getint(argv[0], 5, 100);
        if(argc == 3) Option.Width = getint(argv[2], 37, 132);
        SaveOptions();
      return;
  }

    tp = checkstring(cmdline, "COLOURCODE");
    if(tp == NULL) tp = checkstring(cmdline, "COLORCODE");
    if(tp) {
        if(checkstring(tp, "ON"))   { Option.ColourCode = true; SaveOptions(); return; }
        if(checkstring(tp, "OFF"))      { Option.ColourCode = false; SaveOptions(); return;  }
    }

    void SaveOptionString(char *s);
    tp = checkstring(cmdline, "SAVE");
    if(tp) {
        if(!CurrentLinePtr) return;
        if(SaveOptions()) {
            // if the option table has been changed
            _excep_code = RESTART_DOAUTORUN;
            SystemSoftwareReset();                                            // this will restart the processor
        }
        return;
    }

    OtherOptions();
}

// remove unnecessary text
void CrunchData(char **p, int c) {
    static char inquotes, lastch, incomment;

    if(c == '\n') c = '\r';                                         // CR is the end of line terminator
    if(c == 0  || c == '\r' ) {
        inquotes = false; incomment = false;                        // newline so reset our flags
        if(c) {
            if(lastch == '\r') return;                              // remove two newlines in a row (ie, empty lines)
            *((*p)++) = '\r';
        }
        lastch = '\r';
        return;
    }
        
    if(incomment) return;                                           // discard comments
    if(c == ' ' && lastch == '\r') return;                          // trim all spaces at the start of the line
    if(c == '"') inquotes = !inquotes;
    if(inquotes) {
        *((*p)++) = c;                                              // copy everything within quotes
        return;
    }
    if(c == '\'') {                                                 // skip everything following a comment
        incomment = true;
        return;
    }
    if(c == ' ' && (lastch == ' ' || lastch == ',')) {
        lastch = ' ';
        return;                                                     // remove more than one space or a space after a comma
    }
    *((*p)++) = lastch = c;
}

void cmd_autosave(void) {
    char *buf, *p;
    int c, prevc = 0, crunch = false;

    if(CurrentLinePtr) error("Invalid in a program");
    if(*cmdline) {
        if(toupper(*cmdline) == 'C') 
            crunch = true;
        else
            error("Unknown command");
    }
    
    ClearProgram();                                                 // clear any leftovers from the previous program
    p = buf = GetTempMemory(EDIT_BUFFER_SIZE);
    CrunchData(&p, 0);                                              // initialise the crunch data subroutine
    while((c = (getConsole() & 0x7f)) != 0x1a) {                    // while waiting for the end of text char
        if(p == buf && c == '\n') continue;                         // throw away an initial line feed which can follow the command
        if((p - buf) >= EDIT_BUFFER_SIZE) error("Not enough memory");
        if(isprint(c) || c == '\r' || c == '\n' || c == TAB) {
            if(c == TAB) c = ' ';
            if(crunch)
                CrunchData(&p, c);                                  // insert into RAM after throwing away comments. etc
            else
                *p++ = c;                                           // insert the input into RAM
            {
                if(!(c == '\n' && prevc == '\r')) MMputchar(c);     // and echo it
                if(c == '\r') MMputchar('\n');
            }
            prevc = c;
        }
    }
    *p = 0;                                                         // terminate the string in RAM
    while(getConsole() != -1);                                      // clear any rubbish in the input
    //ClearSavedVars();                                               // clear any saved variables
    SaveProgramToFlash(buf, true);
}


/***********************************************************************************************
interrupt check

The priority of interrupts (highest to low) is:
Touch (MM+ only)
CFunction Interrupt
ON KEY
I2C Slave Rx
I2C Slave Tx
COM1
COM2
COM3 (MM+ only)
COM4 (MM+ only)
GUI Int Down (MM+ only)
GUI Int Up (MM+ only)
WAV Finished (MM+ only)
IR Receive
I/O Pin Interrupts in order of definition
Tick Interrupts (1 to 4 in that order)

************************************************************************************************/

// check if an interrupt has occurred and if so, set the next command to the interrupt routine
// will return true if interrupt detected or false if not
int check_interrupt(void) {
/*
	int i, v;
    char *intaddr;
    static char rti[2];

    #if defined(MX470)
        ProcessTouch();                                             // check GUI touch
        CheckSDCard();
        if(CheckGuiFlag) CheckGui();                                // This implements a LED flash
    #endif

    if(CFuncInt) CallCFuncInt();                                    // check if the CFunction wants to do anything (see CFunction.c)
    //if(!InterruptUsed) return 0;                                    // quick exit if there are no interrupts set
    if(InterruptReturn != NULL || CurrentLinePtr == NULL) return 0; // skip if we are in an interrupt or in immediate mode

    // check for an  ON KEY loc  interrupt
    if(OnKeyGOSUB && kbhitConsole()) {
        intaddr = OnKeyGOSUB;                                       // set the next stmt to the interrupt location
        goto GotAnInterrupt;
    }

//    if ((I2C_Status & I2C_Status_Interrupt) && (I2C_Status & I2C_Status_Completed)) {
//        I2C_Status &= ~I2C_Status_Completed;                      // clear completed flag
//        intaddr = I2C_IntLine;                                    // set the next stmt to the interrupt location
//        goto GotAnInterrupt;
//    }

#ifdef INCLUDE_I2C_SLAVE

  if ((I2C_Status & I2C_Status_Slave_Receive_Rdy)) {
      I2C_Status &= ~I2C_Status_Slave_Receive_Rdy;                  // clear completed flag
      intaddr = I2C_Slave_Receive_IntLine;                          // set the next stmt to the interrupt location
      goto GotAnInterrupt;
  }
  if ((I2C_Status & I2C_Status_Slave_Send_Rdy)) {
      I2C_Status &= ~I2C_Status_Slave_Send_Rdy;                     // clear completed flag
      intaddr = I2C_Slave_Send_IntLine;                             // set the next stmt to the interrupt location
      goto GotAnInterrupt;
  }
#endif

  // interrupt routines for the serial ports
  if(com_interrupt & (1 << 1)) {                                    // is the interrupt flag set?
      com_interrupt &= (~(1 << 1));                                 // clear the flag
      intaddr = com1_interrupt;                                     // set the next stmt to the interrupt location
      goto GotAnInterrupt;
  }
  if(com_interrupt & (1 << 2)) {                                    // is the interrupt flag set?
      com_interrupt &= (~(1 << 2));                                 // clear the flag
      intaddr = com2_interrupt;                                     // set the next stmt to the interrupt location
      goto GotAnInterrupt;
  }
#if defined(MX470)

  if(com_interrupt & (1 << 3)) {                                    // is the interrupt flag set?
      com_interrupt &= (~(1 << 3));                                 // clear the flag
      intaddr = com3_interrupt;                                     // set the next stmt to the interrupt location
      goto GotAnInterrupt;
  }

  if(com_interrupt & (1 << 4)) {                                    // is the interrupt flag set?
      com_interrupt &= (~(1 << 4));                                 // clear the flag
      intaddr = com4_interrupt;                                     // set the next stmt to the interrupt location
      goto GotAnInterrupt;
  }

    if(gui_int_down && GuiIntDownVector) {                          // interrupt on pen down
      intaddr = GuiIntDownVector;                                   // get a pointer to the interrupt routine
        gui_int_down = false;
      goto GotAnInterrupt;
  }

    if(gui_int_up && GuiIntUpVector) {
      intaddr = GuiIntUpVector;                                     // get a pointer to the interrupt routine
        gui_int_up = false;
      goto GotAnInterrupt;
  }

    if(WAVInterrupt != NULL && WAVcomplete) {
        WAVcomplete=false;
      intaddr = WAVInterrupt;                                       // set the next stmt to the interrupt location
      goto GotAnInterrupt;
  }

#endif

    if(IrGotMsg && IrInterrupt != NULL) {
        IrGotMsg = false;
      intaddr = IrInterrupt;                                        // set the next stmt to the interrupt location
      goto GotAnInterrupt;
  }

#if !defined(LITE)
    if(KeypadInterrupt != NULL && KeypadCheck()) {
      intaddr = KeypadInterrupt;                                    // set the next stmt to the interrupt location
      goto GotAnInterrupt;
  }
#endif

  for(i = 0; i < NBRINTERRUPTS; i++) {                              // scan through the interrupt table
      if(inttbl[i].pin != 0) {                                      // if this entry has an interrupt pin set
          v = ExtInp(inttbl[i].pin);                                // get the current value of the pin
          // check if interrupt occured
          if((inttbl[i].lohi == T_HILO && v < inttbl[i].last) || (inttbl[i].lohi == T_LOHI && v > inttbl[i].last) || (inttbl[i].lohi == T_BOTH && v != inttbl[i].last)) {
              intaddr = inttbl[i].intp;                             // set the next stmt to the interrupt location
              inttbl[i].last = v;                                   // save the new pin value
              goto GotAnInterrupt;
          } else
              inttbl[i].last = v;                                   // no interrupt, just update the pin value
      }
  }

  // check if one of the tick interrupts is enabled and if it has occured
  for(i = 0; i < NBRSETTICKS; i++) {
      if(TickInt[i] != NULL && TickTimer[i] > TickPeriod[i]) {
          // reset for the next tick but skip any ticks completely missed
          while(TickTimer[i] > TickPeriod[i]) TickTimer[i] -= TickPeriod[i];
          intaddr = TickInt[i];
          goto GotAnInterrupt;
      }
    }

    // if no interrupt was found then return having done nothing
  return 0;

    // an interrupt was found if we jumped to here
GotAnInterrupt:
    LocalIndex++;                                                   // IRETURN will decrement this
    InterruptReturn = nextstmt;                                     // for when IRETURN is executed
    // if the interrupt is pointing to a SUB token we need to call a subroutine
    if(*intaddr == cmdSUB) {
        strncpy(CurrentInterruptName, intaddr + 1, MAXVARLEN);
        rti[0] = cmdIRET;                                           // setup a dummy IRETURN command
        rti[1] = 0;
        if(gosubindex >= MAXGOSUB) error("Too many SUBs for interrupt");
        errorstack[gosubindex] = CurrentLinePtr;
        gosubstack[gosubindex++] = rti;                             // return from the subroutine to the dummy IRETURN command
        LocalIndex++;                                               // return from the subroutine will decrement LocalIndex
        skipelement(intaddr);                                       // point to the body of the subroutine
    }

    nextstmt = intaddr;                                             // the next command will be in the interrupt routine
*/
	return 0;
}



// get the address for a MMBasic interrupt
// this will handle a line number, a label or a subroutine
// all areas of MMBasic that can generate an interrupt use this function
char *GetIntAddress(char *p) {
    int i;
  if(isnamestart((int)*p)) {                                             // if it starts with a valid name char
      i = FindSubFun(p, 0);                                         // try to find a matching subroutine
      if(i == -1)
          return findlabel(p);                                      // if a subroutine was NOT found it must be a label
      else
          return subfun[i];                                         // if a subroutine was found, return the address of the sub
  }

  return findline(getinteger(p), true);                             // otherwise try for a line number
}
