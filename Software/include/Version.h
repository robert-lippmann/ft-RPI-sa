/***********************************************************************************************************************
MMBasic

Version.h

Copyright 2011 - 2021 Geoff Graham.  All Rights Reserved.

This file and modified versions of this file are supplied to specific individuals or organisations under the following
provisions:

- This file, or any files that comprise the MMBasic source (modified or not), may not be distributed or copied to any other
  person or organisation without written permission.

- Object files (.o and .hex files) generated using this file (modified or not) may not be distributed or copied to any other
  person or organisation without written permission.

- This file is provided in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 ************************************************************************************************************************/



#if !defined(VERSION_INCLUDED)
    #define VERSION_INCLUDED

    #define VERSION         "5.05.05"
    #define YEAR            "2011-2021"
	#if defined(CPU_S32K144_M4)
	#define DEVICE "K144"
	#elif defined (CPU_S32K142_M4)
	#define DEVICE "K142"
	#elif defined (CPU_S32K146_M4)
	#define DEVICE "K146"
	#endif

	#define COPYRIGHT       "Copyright " YEAR " Geoff Graham\r\nS32" DEVICE " Adaption (C) 2022-2025 Robert Lippmann\r\nFirmware version: "__DATE__"/"__TIME__"\r\n"

    #include <stdlib.h>
    #include <setjmp.h>
    #include <string.h>
    #include <ctype.h>
    #include <limits.h>
    #include <math.h>
    #include <float.h>

    // set the startup message
	#define MES_SIGNON "\r\nft-RPI-sa Controller MMBasic Ver " VERSION "\r\n"



    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // debugging options
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


	#include "ft_RPI_sa_Controller/Hardware_Includes.h"

    #include "MMBasic/MMBasic_Includes.h"

#endif

