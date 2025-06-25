/*********************************************************************************
fischertechnik-Raspberry Pi-Stand-Alone Controller

BootLoader.h

Include file that contains the globals and defines for the BootLoader

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

********************************************************************************/

#ifndef FT_RPI_SA_CONTROLLER_GENERAL_BOOTLOADER_H_
#define FT_RPI_SA_CONTROLLER_GENERAL_BOOTLOADER_H_

#ifndef BL_SREC_H
#define BL_SREC_H
#endif

#define SREC_MAX_BYTES        78  /* 39 character pairs maximum record length */
#define SREC_DATA_MAX_BYTES   64  /* 32 character pairs representing data     */

#define SREC_TYPE_0  0
#define SREC_TYPE_1  1
#define SREC_TYPE_2  2
#define SREC_TYPE_3  3
#define SREC_TYPE_5  5
#define SREC_TYPE_7  7
#define SREC_TYPE_8  8
#define SREC_TYPE_9  9

#define LD_MEM_WRITE_ERROR  1
#define LD_SREC_LINE_ERROR  2
#define SREC_PARSE_ERROR    3
#define SREC_CKSUM_ERROR    4
#define SREC_LENGTH_ERROR	5

typedef struct srec_info_s {
    int8_t    type;
    uint8_t*  addr;
    uint8_t*  sr_data;
    uint8_t   dlen;
} srec_info_t;

uint8_t   decode_srec_line (uint8_t *sr_buf, srec_info_t *info);

#endif /* FT_RPI_SA_CONTROLLER_GENERAL_BOOTLOADER_H_ */
