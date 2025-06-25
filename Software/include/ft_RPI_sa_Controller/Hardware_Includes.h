/***********************************************************************************************************************
MMBasic

Hardware_Includes.h

Defines the hardware aspects for Micromite MMBasic.

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



#if !defined(HARDWARE_INCLUDES_H)
    #define HARDWARE_INCLUDES_H

	#if defined(CPU_S32K144_M4)
		#include "S32K144.h"
		#include "system_S32K144.h"
	#elif defined(CPU_S32K142_M4)
		#include "S32K142.h"
		#include "system_S32K142.h"
	#elif defined(CPU_S32K146_M4)
		#include "S32K146.h"
		#include "system_S32K146.h"
    #endif

	#include "s32_core_cm4.h"
	#include "Configuration.h"
	#include "Console.h"
	#include "Serial.h"
	#include "S32_system.h"
	#include "ft-rpi-sa_Controller.h"
    #include "Main.h"
    #include "BootLoader.h"
    #include "Flash.h"
    #include "SerialFileIO.h"
    #include "Memory.h"
    #include "Editor.h"
    #include "Timers.h"
    #include "MM_Misc.h"
	#include "ft-rpi-sa.h"
    #include "I2C.h"
    #include "XModem.h"


#endif


