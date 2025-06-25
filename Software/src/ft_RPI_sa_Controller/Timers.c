/***********************************************************************************************************************
Micromite

timers.c

This module manages various timers (counting variables), the date/time,
counting inputs and generates the sound.  All this is contained within the timer 4 interrupt.

Copyright 2011 - 2021 Geoff Graham.  All Rights Reserved.
Adaption to S32K1 family: Copyright 2022 Robert Lippmann

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


// timer variables
volatile unsigned int SecondsTimer = 0;
volatile unsigned int PauseTimer = 0;
volatile unsigned int IntPauseTimer = 0;
volatile unsigned int InkeyTimer = 0;
volatile unsigned int WDTimer = 0;

volatile long long int mSecTimer = 0;                               // this is used to count mSec
volatile int second = 0;                                            // date/time counters
volatile int minute = 0;
volatile int hour = 0;
volatile int day = 1;
volatile int month = 1;
volatile int year = 2000;

const char DaysInMonth[] = { 0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

extern void CallCFuncmSec(void);                                    // this is implemented in CFunction.c
extern void CallCFuncT1(void);                                      // this is implemented in CFunction.c
extern unsigned int CFuncmSec;                                      // we should call the CFunction mSec function if this is non zero
extern unsigned int CFuncT1;                                        // we should call the CFunction T1 interrupt function if this is non zero

/***************************************************************************************************
initTimers
Initialise the 1 mSec timer used for internal timekeeping.
****************************************************************************************************/
// Do not use console debugging here - the console is not set up
void initTimers(void) {
	// Using LPTMR0 as system counter
	IP_LPTMR0->CSR = LPTMR_CSR_TIE_MASK|LPTMR_CSR_TEN_MASK;	// enable LPTMR with IRQ enabled

	// Now enable LPTMR IRQ in the NVIC
	S32_NVIC->ISER[(LPTMR0_IRQn>>5)] |= (uint32_t)(1U << ((uint32_t)(LPTMR0_IRQn) & (uint32_t)0x1FU));
}

/****************************************************************************************************************
LPTMR0 interrupt service routine
This fires every mSec and is responsible for tracking the time and
the counts of various timing variables
*****************************************************************************************************************/
void LPTMR0_IRQHandler(void)								// This IRQ occurres every 1ms
{
	static unsigned int Tms, Ex_debounce_buf, DIP_debounce_buf, Ex_Buf;
	static unsigned int DIP_Buf, LED_On_Time;
    static int IRQCount=0, FaultPinCount=0;
    static int LED_State=0, FaultActive=FALSE;
    static int CountDebounceBuf[4], CountBuf[4], CountMeta[4];

	int LoopCounter, LEDEdge;
	int TempKey;

	IP_LPTMR0->CSR |= LPTMR_CSR_TCF_MASK;		// clear interrupt flag first

  /////////////////// the following is executed once a millisecond/////////////////////

  /////////////////////////////// count up timers /////////////////////////////////////

  Tms++;															// used by this IRQ for LED timing
  mSecTimer++;                                                      // used by the TIMER function
  PauseTimer++;                                                     // used by the PAUSE command
  IntPauseTimer++;                                                  // used by the PAUSE command inside an interrupt
  InkeyTimer++;                                                     // used to delay on an escape character

  if(WDTimer) {
      if(--WDTimer == 0) {
            _excep_code = WATCHDOG_TIMEOUT;
            SystemSoftwareReset();                                            // crude way of implementing a watchdog timer.
        }
    }

/*
  if(InterruptUsed) {
      int i;
      for(i = 0; i < NBRSETTICKS; i++) TickTimer[i]++;              // used in the interrupt tick
  }

  // if we are measuring period increment the count
  if(ExtCurrentConfig[INT1PIN] == EXT_PER_IN) INT1Count++;
  if(ExtCurrentConfig[INT2PIN] == EXT_PER_IN) INT2Count++;
  if(ExtCurrentConfig[INT3PIN] == EXT_PER_IN) INT3Count++;
  if(ExtCurrentConfig[INT4PIN] == EXT_PER_IN) INT4Count++;
*/
	if (I2C_Timer) {
	  if (--I2C_Timer == 0) {
		  I2C_Status |= I2C_Status_Timeout;
		  S32_NVIC->ISPR[(LPI2C0_Master_IRQn>>5)] = (uint32_t)(1U << ((uint32_t)(LPI2C0_Master_IRQn) & (uint32_t)0x1FU)); // set interrupt
	  }
	}

	TempKey=0;
	for (LoopCounter=0;LoopCounter<=8;LoopCounter++)				// get all 9 ADC channel results
	{
		E_input_adc[LoopCounter] = IP_ADC0->R[LoopCounter];			// get 12-bit ADC result
		TempKey |= (E_input_adc[LoopCounter]>2333)<<LoopCounter;	// "1" threshold is 1.88V (3.3V/4096 * 2333)
	}
	IP_PDB0->SC |= PDB_SC_SWTRIG_MASK; 								// restart all ADC conversions
																	// complete conversion of all 9 channels require ~60us

	Ex_debounce_buf<<=1;											// Debounce of Ex inputs
	if ((TempKey&DebounceMask) == Ex_Buf)							// any change on inputs compared to last round?
		Ex_debounce_buf |= 0x01;									// no, still the same!
	if ((Ex_debounce_buf & 0x0f) == 0x0f)							// same results on last four rounds (4ms)??
	{
		LEDEdge = (~(E_Input&0xff)) & Ex_Buf & 0xff;				// extract the pos edges for LED toggling
		DigitalInputPosEdges |= (~(E_Input&0xff)) & Ex_Buf;			// extract pos edges for sub programs
		DigitalInputNegEdges |= (E_Input&0xff) & (~Ex_Buf);			// same for neg edges
		E_Input = TempKey&DebounceMask;								// provide inputs to sub programs
		// LED off only on pos edges on any Ex input
		if (LEDEdge && (LED_On_Time==0) && !FaultActive)
		{
			LED_On_Time = Tms + 150;								// Turn LED back on in 150ms
			LEDOff;
		}
	}
	Ex_Buf = TempKey&DebounceMask;									// remember Ex state for next IRQ

	if (Tms == LED_On_Time)											// Need to switch the LED back on?
	{
		LED_On_Time=0;
		LEDOn;
	}

	TempKey=((IP_PTD->PDIR & 0xf)|((IP_PTC->PDIR & (1<<5))>>1))^0x1f;	// Get DIP Switch pins in correct order
	// Debounce of DIP Switches
	DIP_debounce_buf<<=1;						// get DIP switches and debounce
	if (TempKey == DIP_Buf)
		DIP_debounce_buf |= 0x01;				// no, still the same
	if ((DIP_debounce_buf & 0x0f) == 0x0f)		// four rounds (4ms) done?
		DIP_switches = TempKey;					// get DIP switches in Bit5-0
	DIP_Buf = TempKey;

	for (LoopCounter=0; LoopCounter<4; LoopCounter++)
	{
		// get actual number of counted edges and add them to the appropriate counter
		Counter[LoopCounter+4] += (0x7fff - (IP_DMA->TCD[LoopCounter].CITER.ELINKNO)); // & DMA_TCD_CITER_ELINKNO_CITER_MASK));
		// "reset" counter
		IP_DMA->TCD[LoopCounter].CITER.ELINKNO = DMA_TCD_CITER_ELINKNO_CITER_MASK;

		switch(LoopCounter)
		{
			case 0: TempKey=((IP_PTE->PDIR & (1<<8))>>8); break;
			case 1: TempKey=((IP_PTD->PDIR & (1<<5))>>5); break;
			case 2: TempKey=((IP_PTA->PDIR & (1<<6))>>6); break;
			case 3: TempKey=((IP_PTB->PDIR & (1<<13))>>13); break;
		}
		CountDebounceBuf[LoopCounter]<<=1;                // Debounce of counter input 0
		if(TempKey == CountBuf[LoopCounter])              // is pin identical to last round?
			   CountDebounceBuf[LoopCounter]|=0x1;
		if((CountDebounceBuf[LoopCounter]&0x0f) == 0x0f)  // are we done with 4 rounds (4ms) of debounce?
		{
			   if(CountMeta[LoopCounter] & (~CountBuf[LoopCounter]))    // check for negative edge
					  Counter[LoopCounter]++;
			   CountMeta[LoopCounter] = CountBuf[LoopCounter];
		}
		CountBuf[LoopCounter]=TempKey;                    // save current value for next round
	}

	// Check on short on the outputs
    if ((~(IP_PTC->PDIR)&(FaultPin1_Bit|FaultPin2_Bit)) || FaultActive==TRUE)
    {
    	if (!FaultActive) {
    		LEDOff;
    		FaultActive=TRUE;
			LED_State=0;
			IRQCount=0;
    	}
		if (IRQCount==150)    // LED_State change every 150ms
		{
			IRQCount=0;
			LED_State=(LED_State<<1)|0x01;    // move on to next state
			if (LED_State>0xffff)            // already 17 states done? (each bit is one state -> 17*150ms=2.55s
				LED_State=0;
			if (LED_State<=0x7f)            // toggle LED on first 7 states -> 4 times on and 3 times off -> 7*150ms=1.05s
				LEDToggle;
		} else
			IRQCount++;
		if (!(~(IP_PTC->PDIR)&(FaultPin1_Bit|FaultPin2_Bit)))		// check fault pin condition (low active)
		{
			FaultPinCount++;				// currently fault has gone
			if (FaultPinCount>100)                            	// fault condition gone for >100ms?
			{
				LEDOn;                                        	// LED on
	    		FaultActive=FALSE;
			}
		}
		else FaultPinCount=0;				// no, fault is still present
    }

	////////////////////////////////// this code runs once a second /////////////////////////////////
	if(++SecondsTimer >= 1000) {
		SecondsTimer -= 1000;                                         // reset every second (thanks to matherp)
		// keep track of the time and date
		if(++second >= 60) {
			second = 0 ;
			if(++minute >= 60) {
				minute = 0;
				if(++hour >= 24) {
					hour = 0;
					if(++day > DaysInMonth[month] + ((month == 2 && (year % 4) == 0)?1:0)) {
						day = 1;
						if(++month > 12) {
						  month = 1;
						  year++;
						}
					}
				}
			}
			}
	}
}

// general purpose time delay
// it is based on the SysTick timer which is running core clock
// maximum delay is ~209ms@80MHz / ~149ms@112MHz

#define SetupTime 13                                        // the time it takes to call the function and setup the loop (in core timer ticks)
void uSec(unsigned int us) {
       us = 0xffffff - (((us * 80u)) - SetupTime);			// 1us is 80 core clock cycles (1MHz vs. 80MHz) and SysTick counter is running on core clock
       S32_SysTick->CVR = 0;								// reset counter
       asm("nop");											// spend one CPU cycle to get the counter updated to 0xffffff
    while(S32_SysTick->CVR > us);							// wait until counter reaches target
}

/* A bit more in detail explained in terms of timing: Optimize to size!
00000000 <uSec>:
// it is based on the SysTick timer which is running core clock
// maximum delay is 209 ms

#define SetupTime 13                                        // the time it takes to call the function and setup the loop (in core timer ticks)
void uSec(unsigned int us) {
       us = 0xffffff - (((us * 80u)) - SetupTime);			// 1us is 80 core clock cycles (1MHz vs. 80MHz) and SysTick counter is running on core clock
   0:	4a05      	ldr	r2, [pc, #20]	; (18 <uSec+0x18>)	// 2 cycles
   2:	2350      	movs	r3, #80	; 0x50					// 1 cycle
   4:	fb03 2010 	mls	r0, r3, r0, r2						// 2 cycles
       S32_SysTick->CVR = 0;								// reset counter
   8:	4b04      	ldr	r3, [pc, #16]	; (1c <uSec+0x1c>)	// 2 cycles
   a:	2200      	movs	r2, #0							// 1 cycle
   c:	609a      	str	r2, [r3, #8]						// 2 cycles
       asm("nop");											// spend one CPU cycle to get the counter updated to 0xffffff
   e:	bf00      	nop										// 1 cycle
    while(S32_SysTick->CVR > us);							// wait until counter reaches target
  10:	689a      	ldr	r2, [r3, #8]						// 2 cycles (Here we read the counter)
  12:	4282      	cmp	r2, r0								// if counter matches, this instruction is not counted anymore -> 13 cycles required
  14:	d8fc      	bhi.n	10 <uSec+0x10>
}
  16:	4770      	bx	lr
  18:	01000013 	.word	0x01000013
  1c:	e000e010 	.word	0xe000e010
*/

