/***********************************************************************************************************************
MMBasic

xmodem.c

Implements the xmodem command and protocol.

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

void xmodemTransmit(char *p);
void xmodemReceive(char *sp, int maxbytes, int crunch);

void cmd_xmodem(void) {
    char *buf, BreakKeySave, *p, *fromp;

    if(CurrentLinePtr) error("Invalid in a program");
    ClearProgram();
    buf = GetTempMemory(EDIT_BUFFER_SIZE);

    BreakKeySave = BreakKey;
    BreakKey = 0;
    if(toupper(*cmdline) == 'R' || toupper(*cmdline) == 'C') {
        xmodemReceive(buf, EDIT_BUFFER_SIZE, (toupper(*cmdline) == 'C'));
        //ClearSavedVars();                                           // clear any saved variables
        SaveProgramToFlash(buf, true);
    }
    else if(toupper(*cmdline) == 'S') {
        fromp  = ProgMemory;
        p = buf;
        while(1) {
            if(*fromp == T_NEWLINE) {
                fromp = llist(p, fromp);                            // expand the line
                p += strlen(p);
                if(p - buf + 40 > EDIT_BUFFER_SIZE) error("Not enough memory");
                *p++ = '\n'; *p = 0;
            }

            if(fromp[0] == 0 || fromp[0] == 0xff) break;            // finally, is it the end of the program?
        }
        --p; *p = 0;                                                // erase the last line terminator
        xmodemTransmit(buf);                                        // send it off
      }
    else
        error("Syntax");

    BreakKey = BreakKeySave;
}


int _inbyte(int timeout) {
    int c;

    PauseTimer = 0;
    while(PauseTimer < timeout) {
        c = getConsole();
        if(c != -1) {
            return c;
        }
    }
    return -1;
}



/***********************************************************************************************
the xmodem protocol
************************************************************************************************/

/* derived from the work of Georges Menie (www.menie.org) Copyright 2001-2010 Georges Menie
 * very much debugged and changed
 *
 * this is just the basic XModem protocol (no 1K blocks, crc, etc).  It has been tested on
 * Terra Term and is intended for use with that software.
 */


#define SOH  0x01
#define STX  0x02
#define EOT  0x04
#define ACK  0x06
#define NAK  0x15
#define CAN  0x18
#define PAD  0x1a

#define DLY_1S 1000
#define MAXRETRANS 25

#define X_BLOCK_SIZE  128
#define X_BUF_SIZE    X_BLOCK_SIZE + 6                              // 128 for XModem + 3 head chars + 2 crc + nul


static int check(const unsigned char *buf, int sz)
{
  int i;
  unsigned char cks = 0;
  for (i = 0; i < sz; ++i) {
      cks += buf[i];
  }
  if (cks == buf[sz])
      return 1;

  return 0;
}


static void flushinput(void)
{
  while (_inbyte(((DLY_1S)*3)>>1) >= 0);
}


void xmodemReceive(char *sp, int maxbytes, int crunch) {
    unsigned char xbuff[X_BUF_SIZE];
    unsigned char *p;
    unsigned char trychar = NAK; //'C';
    unsigned char packetno = 1;
    int i, c;
    int retry, retrans = MAXRETRANS;

    CrunchData(&sp, 0);                                             // initialise the crunch subroutine

        // first establish communication with the remote
    while(1) {
        for( retry = 0; retry < 32; ++retry) {
            if(trychar) putConsole(trychar);
            if ((c = _inbyte((DLY_1S)<<1)) >= 0) {
                switch (c) {
                case SOH:
                    goto start_recv;
                case EOT:
                    flushinput();
                    putConsole(ACK);
                    if(maxbytes <= 0) error("Not enough memory");
                    *sp++ = 0;                                      // terminate the data
                    return;                                         // no more data
                case CAN:
                    flushinput();
                    putConsole(ACK);
                    error("Cancelled by remote");
                    break;
                default:
                    break;
                }
            }
        }
        flushinput();
        putConsole(CAN);
        putConsole(CAN);
        putConsole(CAN);
        error("Remote did not respond");                            // no sync

  start_recv:
        trychar = 0;
        p = xbuff;
        *p++ = SOH;
        for (i = 0;  i < (X_BLOCK_SIZE+3); ++i) {
            if ((c = _inbyte(DLY_1S)) < 0) goto reject;
            *p++ = c;
        }

        if (xbuff[1] == (unsigned char)(~xbuff[2]) && (xbuff[1] == packetno || xbuff[1] == (unsigned char)packetno-1) && check(&xbuff[3], X_BLOCK_SIZE)) {
            if (xbuff[1] == packetno) {
                for(i = 0 ; i < X_BLOCK_SIZE ; i++) {
                    if(--maxbytes > 0) {
                        if(xbuff[i + 3] == PAD)
                            *sp++ = 0;                              // replace any EOF's (used to pad out a block) with NUL
                        else
                            if(crunch)
                                CrunchData(&sp, xbuff[i + 3]);
                            else
                                *sp++ = xbuff[i + 3];               // saving to a memory buffer
                    }
                }
                ++packetno;
                retrans = MAXRETRANS+1;
            }
            if (--retrans <= 0) {
                flushinput();
                putConsole(CAN);
                putConsole(CAN);
                putConsole(CAN);
                error("Too many errors");
            }
            putConsole(ACK);
            continue;
        }
  reject:
        flushinput();
        putConsole(NAK);
    }
}


void xmodemTransmit(char *p)
{
  unsigned char xbuff[X_BUF_SIZE];
  unsigned char packetno = 1;
    char prevchar = 0;
  int i, c, len;
  int retry;

  // first establish communication with the remote
  while(1) {
      for( retry = 0; retry < 32; ++retry) {
          if ((c = _inbyte((DLY_1S)<<1)) >= 0) {
              switch (c) {
              case NAK:                                             // start sending
                  goto start_trans;
              case CAN:
                  if ((c = _inbyte(DLY_1S)) == CAN) {
                      putConsole(ACK);
                      flushinput();
                      error("Cancelled by remote");
                  }
                  break;
              default:
                  break;
              }
          }
      }
      putConsole(CAN);
      putConsole(CAN);
      putConsole(CAN);
      flushinput();
      error("Remote did not respond");                              // no sync

      // send a packet
      while(1) {
      start_trans:
          memset (xbuff, 0, X_BUF_SIZE);                            // start with an empty buffer

          xbuff[0] = SOH;                                           // copy the header
          xbuff[1] = packetno;
          xbuff[2] = ~packetno;

          for(len = 0; len < 128 && *p; len++) {
                if(*p == '\n' && prevchar != '\r')
                    prevchar = xbuff[len + 3] = '\r';
                else
                    prevchar = xbuff[len + 3] = *p++;               // copy the data into the packet
            }
          if (len > 0) {
              unsigned char ccks = 0;
              for (i = 3; i < X_BLOCK_SIZE+3; ++i) {
                  ccks += xbuff[i];
              }
              xbuff[X_BLOCK_SIZE+3] = ccks;

              // now send the block
              for (retry = 0; retry < MAXRETRANS && !MMAbort; ++retry) {
                  // send the block
                  for (i = 0; i < X_BLOCK_SIZE+4 && !MMAbort; ++i) {
                      putConsole(xbuff[i]);
                  }
                  // check the response
                  if ((c = _inbyte(DLY_1S)) >= 0 ) {
                      switch (c) {
                      case ACK:
                          ++packetno;
                          goto start_trans;
                      case CAN:                                     // cancelled by remote
                          putConsole(ACK);
                          flushinput();
                          error("Cancelled by remote");
                          break;
                      case NAK:                                     // receiver got a corrupt block
                      default:
                          break;
                      }
                  }
              }
              // too many retrys... give up
              putConsole(CAN);
              putConsole(CAN);
              putConsole(CAN);
              flushinput();
              error("Too many errors");
          }

          // finished sending - send end of text
          else {
              for (retry = 0; retry < 10; ++retry) {
                  putConsole(EOT);
                  if ((c = _inbyte((DLY_1S)<<1)) == ACK) break;
              }
              flushinput();
              if(c == ACK) return;
              error("Error closing");
          }
      }
  }
}

