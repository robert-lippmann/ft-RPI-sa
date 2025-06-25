/************************************************************************************************************************
fischertechnik-Raspberry Pi-Stand-Alone Controller

ft_rpi_sa.c

Routines to handle S32K1 HW setup, provide board specific commands/functions
and internal functions such as DMA counting

Copyright 2022 Robert Lippmann

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

*******************************************************************************/

#include "Version.h"
#include <stdio.h>

//#define CLKOUT

extern uint32_t __BootLoader_Flash_Start__;
extern uint32_t __BootLoader_Flash_End__;
extern uint32_t __BootLoader_Ram_Start__;
extern uint32_t __MMBASIC_RAM_BASE__;

// global vars
unsigned char 				dummy[4];				// used for DMA pulse counting
volatile unsigned int		Counter[8];				// These are the four Cn counter inputs (1-4 "debounced" 5-7 DMA)
volatile unsigned int   	E_input_adc[9];			// containing the ADC value for each input [5:0]=[E6:E1] / [6]=Poti
volatile unsigned int		E_Input;				// each bit represents an input [E8:E1]=E_Input[7:0]
volatile unsigned int  		DIP_switches;			// containing the debounced DIP-switch settings [4:0]=[DIP5:DIP1]
volatile unsigned int 		DigitalInputPosEdges	// each bit represents a pos edge on Ex (after usage, user need to clean the appropriate bit!)
						  , DigitalInputNegEdges;	// each bit represents a new edge on Ex (after usage, user need to clean the appropriate bit!)
volatile unsigned int		DebounceMask=0x1ff;		// a 1 indicates that the input needs to be debounced [5:0] = [E6:E1]
unsigned int				ModeSetting;			// Stores the config of the mode Jumpers JP5/JP6
unsigned int				HB_MotorConfig;			// stores the config of each H-Bridge device
struct StepperStruct		Stepper[2];				// keep track of stepper position

// Stepper timing table for full/half step
const short StepCurve[] = {MaxPwmPeriod+1,0,0,MaxPwmPeriod+1,
					 MaxPwmPeriod+1,0,0,0,
					 MaxPwmPeriod+1,0,MaxPwmPeriod+1,0,
					 0,0,MaxPwmPeriod+1,0,
					 0,MaxPwmPeriod+1,MaxPwmPeriod+1,0,
					 0,MaxPwmPeriod+1,0,0,
					 0,MaxPwmPeriod+1,0,MaxPwmPeriod+1,
					 0,0,0,MaxPwmPeriod+1
};

// function (which looks like a pre defined variable) to return the type of platform
void fun_device(void){
  sret = GetTempStrMemory();                                  // this will last for the life of the command
    strcpy(sret, "ft-RPI-sa Controller V2.0");
    CtoM(sret);
    targ = T_STR;
}

// return the number of counts on an Cx input
void fun_counter(void) {
	int channel;

	channel = getnumber(ep);
	if(channel<1 || channel>8) error("Counter channel range 1..8");
    iret = Counter[channel-1];
    targ = T_INT;
}

// reset a counter input
void cmd_counter(void) {
	int channel, value;

	  channel = getinteger(cmdline);
      if(channel<1 || channel>8) error("Counter channel range 1..8");
	  while(*cmdline && tokenfunction(*cmdline) != op_equal) cmdline++;
	  if(!*cmdline) error("Syntax");
	  ++cmdline;
	  if(!*cmdline) error("Syntax");
	  value = getinteger(cmdline);
	  if(value != 0) error("Counter can be set to 0 only");
	  Counter[channel-1] = 0;
}

void cmd_reset(void)
{
	_excep_code = RESET_COMMAND;
	SystemSoftwareReset();
}

// return ADC value for specific E_Input
void fun_eana(void) {
	int channel;

	channel = getnumber(ep);
	if(channel<1 || channel>9) error("E_Ana channel range 1..9");
    iret = E_input_adc[channel-1];
    targ = T_INT;
}

// return 0/1 for each E_Input
void fun_edig(void) {
	int channel;

	channel = getnumber(ep);
	if(channel<1 || channel>8) error("E_Dig channel range 1..8");
    iret = (E_Input & (1<<(channel-1)))>>(channel-1);
    targ = T_INT;
}

// return the value of the DIP switches
void fun_dipswitch(void) {
    iret = DIP_switches;
    targ = T_INT;
}

// return the digital value of all E inputs
void fun_einput(void) {
	iret = E_Input&0x000000ff;
    targ = T_INT;
}

// return the value of RTC_IRQ line
void fun_rtcalarm(void) {
	iret = 1 ^ (IP_PTE->PDIR & (1<<PTE10))>>PTE10;
	targ = T_INT;
}

void hb_conf(char *p) {
	int device, mode, count, from, to;

    getargs(&p, 3, ",");
    if(!(argc == 3)) error("Argument count");
    device = getint(argv[0], 1, 2)-1; 		// device # ranges from 0-1
    mode = getint(argv[2], 0, 3);			// mode can be 0 -> 3
    if (!(mode==0 || mode==3)) error("Only mode 0 & 3 are supported");
    I2C_SetHBridge((device==0 ? HB_Address_HB1 : HB_Address_HB2), mode);
    HB_MotorConfig &= (0b11<<((1-device)*2));	// keep settings for un-selected device
    HB_MotorConfig |= (mode<<(device*2));		// save new settings for selected device

    from=0;
    to=8;
    if(device) {
    	from=4;
    } else {
    	to=4;
    }
    for(count=from;count<to;count++) {
    	IP_FTM1->CONTROLS[count].CnV = 0;
    }
}

void AllOutputsOff(void)
{
	for(unsigned int channel=0;channel<=3;channel++)
	{
		IP_FTM1->CONTROLS[channel*2].CnV = MaxPwmPeriod+1;		// set both pins to 1 => fast brake
		IP_FTM1->CONTROLS[channel*2+1].CnV = MaxPwmPeriod+1;
	}
}

void hb_set(char *p) {
    int channel, value, count, from, to;

    getargs(&p, 3, ",");
    if(!(argc == 3)) error("Argument count");
    channel = getint(argv[0], 0, 255);            					// each "bit" represents a channel
    if (checkstring(argv[2], "ON") != NULL) {
        value=0xffff;                                    			// switch channel on
    } else {
        if (checkstring(argv[2], "OFF") != NULL) {
            value=0;                                    			// switch channel off
        } else {
            value = getint(argv[2], 0, 100);            			// get percentage value for PWM setting
        }
    }
    from=0;
    to=8;
    if((HB_MotorConfig & 0b11) != 0b11)								// check config of first h-Bridge
    	from=4;
    if(((HB_MotorConfig & (0b11<<2))>>2) != 0b11)
    	to=4;
    if (from==to)
    	error("No H-Bridge available");

    for (count=from; count<to;count++)
    {
        if (channel & (1<<count)) {                                	// check whether channel need to be updated
            if (value==0xffff)
                IP_FTM1->CONTROLS[count].CnV = MaxPwmPeriod+1;        	// output ON
            else if (value==0)
                IP_FTM1->CONTROLS[count].CnV = 0;                    	// output OFF
            else {
                IP_FTM1->CONTROLS[count].CnV = (MaxPwmPeriod+1)*value/100;	// set output PWM value
            }
        }
    }
}

void hb_write(char *p) {
    int channel, count, from, to;

    channel = getinteger(p);            					// each "bit" represents a channel
    if (channel>255 || channel <0) error("Out of range");
    from=0;
    to=8;
    if((HB_MotorConfig & 0b11) != 0b11)								// check config of first h-Bridge
    	from=4;
    if(((HB_MotorConfig & (0b11<<2))>>2) != 0b11)
    	to=4;
    if (from==to)
    	error("No H-Bridge available");

    for (count=from; count<to;count++)
    {
        if (channel & (1<<count)) {                             // check which value for current channel
            IP_FTM1->CONTROLS[count].CnV = MaxPwmPeriod+1;        	// output ON
        } else {
            IP_FTM1->CONTROLS[count].CnV = 0;                    	// output OFF
        }
    }
}

void hb_motor(char *p) {
    int channel, value=0, direction;

    getargs(&p, 5, ",");
    if(argc < 3 || argc>5) error("Argument count");					// need a minimum of 2 parameter and maximum of 3 ("," counts as well!)
    channel = getint(argv[0], 1, 4)-1;            					// maximum of 4 motors to be controlled
    direction = getint(argv[2], 1, 4);								// 4 modi possible (1/2 turn on, 3 fast brake, 4 coasting)
    if (direction<3 && argc<5) error("Argument count");				// Error as we need a percentage for mode 1 & 2
    if (direction>2 && argc>3) error("Argument count");				// Error as we do not need any 3rd parameter for mode 3 & 4
    if (argc==5)
    	value = getint(argv[4], 0, 100);							// PWM value in percent
    if(((HB_MotorConfig & (0b11<<((channel>>1)*2)))>>((channel>>1)*2)) != 0)	// is the requested H-Bridge configured correctly?
    	error("Wrong H-Bridge config");
    value = (MaxPwmPeriod+1)*value/100;								// Calculate PWM value
    if (direction==1) {
		IP_FTM1->CONTROLS[channel*2].CnV = 0;				// pin of channel 0 ="0" -> coasting in low phase
		IP_FTM1->CONTROLS[channel*2+1].CnV = value;		// set PWM speed according to given speed
    } else if (direction==2) {
		IP_FTM1->CONTROLS[channel*2].CnV = value;			// set PWM speed according to given speed
		IP_FTM1->CONTROLS[channel*2+1].CnV = 0;			// pin of channel 0 ="0" -> coasting in low phase
    } else if (direction==3) {
		IP_FTM1->CONTROLS[channel*2].CnV = MaxPwmPeriod+1;		// set both pins to 1 => fast brake
		IP_FTM1->CONTROLS[channel*2+1].CnV = MaxPwmPeriod+1;
    } else {
		IP_FTM1->CONTROLS[channel*2].CnV = 0;				// set both pins to 0 => coasting
		IP_FTM1->CONTROLS[channel*2+1].CnV = 0;
    }
}

/* output encmotor <h-bridge>,<counterinput>,<steps>,<minspeed>,<maxspeed>

   Curves of speed (X=t / Y=v)

             (B)____________________________(C)
               /                            \
              /                              \
             /                                \
            /                                  \
           /                                    \
  (A) ____/                                      \_______(A)

   (A) MinSpeed
   (B) MaxSpeed
   (C) Trip point for speed decrease (calculated)

              (B)
               /\
              /  \
             /    \
            /      \
           /        \
  (A) ____/          \_______(A)

   (A) MinSpeed
   (B) Trip point (calculated) with either v=MaxSpeed or point of speed decrease reached before v=MaxSpeed
*/
void hb_encmotor(char *p) {
    int Channel, MinSpeed, MaxSpeed, Direction, Timer, SpeedDelta, ActualSpeed;
    int CounterInput, CountsToMove, InitialCounterValue, TargetCounts; // FinalCounterValue;
    volatile int TripPoint;

    getargs(&p, 9, ",");
    if(argc != 9) error("Argument count");					// need 9 parameter ("," counts as well!)
    Channel = getint(argv[0], 1, 4)-1;     					// maximum of 4 motors to be controlled
	CounterInput = getint(argv[2],1,4)-1+4;					// get counter input (+4 as we use the DMA counters)
	CountsToMove = getint(argv[4],-10000,10000);			// get number of counts to move
	if (CountsToMove>0)
		Direction=1;
	else {
		Direction=2;
		CountsToMove = -CountsToMove;
	}
	MinSpeed = getint(argv[6],1,100);						// minimum speed
	MaxSpeed = getint(argv[8],1,100);						// maximum speed
	if (MinSpeed>MaxSpeed) error("MinSpeed greater than MaxSpeed");
    if(((HB_MotorConfig & (0b11<<((Channel>>1)*2)))>>((Channel>>1)*2)) != 0)	// is the requested H-Bridge configured correctly?
    {
    	error("Wrong H-Bridge config");
    }
    InitialCounterValue = Counter[CounterInput];
	TripPoint = Counter[CounterInput]+((CountsToMove*922)/2048);	// Speed decrease or MaxSpeed point in curve (a bit less than half of counts to move)
//	FinalCounterValue = Counter[CounterInput]+CountsToMove;	// calculate final counter value after move
    MinSpeed = (MaxPwmPeriod+1)*MinSpeed/100;				// Calculate PWM value
    MaxSpeed = (MaxPwmPeriod+1)*MaxSpeed/100;				// Calculate PWM value
    SpeedDelta = (MaxSpeed-MinSpeed)/10;					// 10 steps until max speed
	TargetCounts = Counter[CounterInput]+CountsToMove;	// Use DMA counter 4-7 and calculate target position
	ActualSpeed = MinSpeed;									// Start with min speed
	for(;;)
	{
		Timer=SecondsTimer+50;								// SecondsTimer counts in ms -> Speed in-/decrease in 50ms steps
		if(Timer>=1000) Timer-=1000;
		if (Direction==1) {
			IP_FTM1->CONTROLS[Channel*2].CnV = 0;				// pin of channel 0 ="0" -> coasting in low phase
			IP_FTM1->CONTROLS[Channel*2+1].CnV = ActualSpeed;	// set PWM speed according to given speed
		} else {
			IP_FTM1->CONTROLS[Channel*2].CnV = ActualSpeed;	// set PWM speed according to given speed
			IP_FTM1->CONTROLS[Channel*2+1].CnV = 0;			// pin of channel 0 ="0" -> coasting in low phase
		}
		while(SecondsTimer!=Timer)							// while the motor in-/decreases to next step
		{
			if(MMAbort)										// potential CTRL+C abort
			{
				IP_FTM1->CONTROLS[Channel*2].CnV = MaxPwmPeriod+1;		// set both pins to 1 => fast brake
				IP_FTM1->CONTROLS[Channel*2+1].CnV = MaxPwmPeriod+1;
				WDTimer = 0;                                                // turn off the watchdog timer
				longjmp(mark, 1);                                           // jump back to the input prompt
			}
			if(Counter[CounterInput]>=TargetCounts)		// did we reach the target position?
				goto exitfun;								// exit for(;;) loop
		}
		if((Counter[CounterInput] == InitialCounterValue) && (ActualSpeed>MinSpeed))	// check after first speed increase whether we got counter pulses from motor
			goto exitfun;

		if(Counter[CounterInput]<TripPoint)				// Until we've not reached the point of slowing down
		{
			if(ActualSpeed<MaxSpeed)						// Increase Speed
			{
				ActualSpeed+=SpeedDelta;
				if(ActualSpeed>MaxSpeed){
					ActualSpeed=MaxSpeed;
					TripPoint=TargetCounts-(Counter[CounterInput]-InitialCounterValue)-MaxSpeed/2048; // define new trip point
				}
			}
		} else {
			if(ActualSpeed>MinSpeed)						// decrease speed
			{
				ActualSpeed-=SpeedDelta;
				if(ActualSpeed<MinSpeed) ActualSpeed=MinSpeed;	// until minimum
			}
		}
	} // end (for(;;)
	exitfun:
	IP_FTM1->CONTROLS[Channel*2].CnV = MaxPwmPeriod+1;		// set both pins to 1 => fast brake
	IP_FTM1->CONTROLS[Channel*2+1].CnV = MaxPwmPeriod+1;
	uSec(50000);										// wait 50ms until motor stops
	if(Counter[CounterInput] == InitialCounterValue)	// check whether we got counter pulses from motor
		error("No counter pulses from motor");
	else if (Counter[CounterInput] != TargetCounts)
		MMPrintString("WARNING: MinSpeed was too high - counter overflow\n\r");
}

/* output encmotor2 <h-bridge>,<counterinput>,<steps>,<minspeed>,<maxspeed>,<input>,<stop value>

   Curves of speed (X=t / Y=v)

             (B)____________________________(C)
               /                            \
              /                              \
             /                                \
            /                                  \
           /                                    \
  (A) ____/                                      \_______(A)

   (A) MinSpeed
   (B) MaxSpeed
   (C) Trip point for speed decrease (calculated)

              (B)
               /\
              /  \
             /    \
            /      \
           /        \
  (A) ____/          \_______(A)

   (A) MinSpeed
   (B) Trip point (calculated) with either v=MaxSpeed or point of speed decrease reached before v=MaxSpeed

   <input>: Is the input which is monitored for motor stop
   <stop value>: Is either 0 or 1 and reflects the value to stop the motor
*/

void hb_encmotor2(char *p) {
    int Channel, MinSpeed, MaxSpeed, Direction, Timer, SpeedDelta, ActualSpeed;
    int CounterInput, CountsToMove, InitialCounterValue, TargetCounts; // FinalCounterValue;
    int StopInput, StopValue, CounterBeforeStop;
    volatile int TripPoint;
    char TempString[256];

    getargs(&p, 13, ",");
    if(argc != 13) error("Argument count");					// need 13 parameter ("," counts as well!)
    Channel = getint(argv[0], 1, 4)-1;     					// maximum of 4 motors to be controlled
	CounterInput = getint(argv[2],1,4)-1+4;					// get counter input (+4 as we use the DMA counters)
	CountsToMove = getint(argv[4],-10000,10000);			// get number of counts to move
	if (CountsToMove>0)
		Direction=1;
	else {
		Direction=2;
		CountsToMove = -CountsToMove;
	}
	MinSpeed = getint(argv[6],1,100);						// minimum speed
	MaxSpeed = getint(argv[8],1,100);						// maximum speed
	StopInput = getint(argv[10],1,8)-1;						// One of the 8 inputs can be used
	StopValue = getint(argv[12],0,1);						// Input, providing the stop value, is either 0 or 1

	if (MinSpeed>MaxSpeed) error("MinSpeed greater than MaxSpeed");
    if(((HB_MotorConfig & (0b11<<((Channel>>1)*2)))>>((Channel>>1)*2)) != 0)	// is the requested H-Bridge configured correctly?
    {
    	error("Wrong H-Bridge config");
    }
    InitialCounterValue = Counter[CounterInput];
	TripPoint = Counter[CounterInput]+((CountsToMove*922)/2048);	// Speed decrease or MaxSpeed point in curve (a bit less than half of counts to move)
//	FinalCounterValue = Counter[CounterInput]+CountsToMove;	// calculate final counter value after move
    MinSpeed = (MaxPwmPeriod+1)*MinSpeed/100;				// Calculate PWM value
    MaxSpeed = (MaxPwmPeriod+1)*MaxSpeed/100;				// Calculate PWM value
    SpeedDelta = (MaxSpeed-MinSpeed)/10;					// 10 steps until max speed
	TargetCounts = Counter[CounterInput]+CountsToMove;	// Use DMA counter 4-7 and calculate target position
	ActualSpeed = MinSpeed;									// Start with min speed
	for(;;)
	{
		Timer=SecondsTimer+50;								// SecondsTimer counts in ms -> Speed in-/decrease in 50ms steps
		if(Timer>=1000) Timer-=1000;
		if (Direction==1) {
			IP_FTM1->CONTROLS[Channel*2].CnV = 0;				// pin of channel 0 ="0" -> coasting in low phase
			IP_FTM1->CONTROLS[Channel*2+1].CnV = ActualSpeed;	// set PWM speed according to given speed
		} else {
			IP_FTM1->CONTROLS[Channel*2].CnV = ActualSpeed;	// set PWM speed according to given speed
			IP_FTM1->CONTROLS[Channel*2+1].CnV = 0;			// pin of channel 0 ="0" -> coasting in low phase
		}
		while(SecondsTimer!=Timer)							// while the motor in-/decreases to next step
		{
			if(MMAbort)										// potential CTRL+C abort
			{
				IP_FTM1->CONTROLS[Channel*2].CnV = MaxPwmPeriod+1;		// set both pins to 1 => fast brake
				IP_FTM1->CONTROLS[Channel*2+1].CnV = MaxPwmPeriod+1;
				WDTimer = 0;                                                // turn off the watchdog timer
				longjmp(mark, 1);                                           // jump back to the input prompt
			}
			if(Counter[CounterInput]>=TargetCounts)		// did we reach the target position?
				goto exitfun;								// exit for(;;) loop
			if((E_Input&(1<<StopInput)) == (StopValue<<StopInput))	// does the selected Input to stop motor contain stop value?
			{
				goto exitfun;
			}
		}
		if((Counter[CounterInput] == InitialCounterValue) && (ActualSpeed>MinSpeed))	// check after first speed increase whether we got counter pulses from motor
			goto exitfun;

		if(Counter[CounterInput]<TripPoint)				// Until we've not reached the point of slowing down
		{
			if(ActualSpeed<MaxSpeed)						// Increase Speed
			{
				ActualSpeed+=SpeedDelta;
				if(ActualSpeed>MaxSpeed){
					ActualSpeed=MaxSpeed;
					TripPoint=TargetCounts-(Counter[CounterInput]-InitialCounterValue)-MaxSpeed/2048; // define new trip point
				}
			}
		} else {
			if(ActualSpeed>MinSpeed)						// decrease speed
			{
				ActualSpeed-=SpeedDelta;
				if(ActualSpeed<MinSpeed) ActualSpeed=MinSpeed;	// until minimum
			}
		}
	} // end (for(;;)
	exitfun:
	CounterBeforeStop=Counter[CounterInput];
	IP_FTM1->CONTROLS[Channel*2].CnV = MaxPwmPeriod+1;		// set both pins to 1 => fast brake
	IP_FTM1->CONTROLS[Channel*2+1].CnV = MaxPwmPeriod+1;
	uSec(50000);										// wait 50ms until motor stops
	if(Counter[CounterInput] == InitialCounterValue)	// check whether we got counter pulses from motor
		error("No counter pulses from motor");
	else if (Counter[CounterInput] != CounterBeforeStop) {
			sprintf(TempString,"%d Pulses received after stop", Counter[CounterInput]-CounterBeforeStop);
			MMPrintString(TempString);
			//MMPrintString("WARNING: Pulses received after stop - Counter undefined\n\r");
	}
}

// IRQ for stepper on H-Bridge 1 for applying one step
void LPIT0_Ch0_IRQHandler(int motor) {
	int CurrentStep;

	IP_LPIT0->MSR = 0x01;		// clear LPIT Ch0 interrupt
	Stepper[0].Position += Stepper[0].Direction;
	if (!(--Stepper[0].StepsToGo))
	    IP_LPIT0->TMR[0].TCTRL = 0x0;					// disable channel
	CurrentStep = Stepper[0].Position % Stepper[0].Mode;
	if(Stepper[0].Mode == 4)
		CurrentStep *=2;

	IP_FTM1->CONTROLS[0].CnV = StepCurve[CurrentStep*4];
	IP_FTM1->CONTROLS[1].CnV = StepCurve[CurrentStep*4+1];
	IP_FTM1->CONTROLS[2].CnV = StepCurve[CurrentStep*4+2];
	IP_FTM1->CONTROLS[3].CnV = StepCurve[CurrentStep*4+3];
}

// IRQ for stepper on H-Bridge 2 for applying one step
void LPIT0_Ch1_IRQHandler(int motor) {
	int CurrentStep;

	IP_LPIT0->MSR = 0x02;		// clear LPIT Ch1 interrupt
	Stepper[1].Position += Stepper[1].Direction;
	if (!(--Stepper[1].StepsToGo))
	    IP_LPIT0->TMR[1].TCTRL = 0x0;					// disable channel
	CurrentStep = Stepper[1].Position % Stepper[1].Mode;

	if(Stepper[1].Mode == 4)
		CurrentStep *=2;

	IP_FTM1->CONTROLS[4].CnV = StepCurve[CurrentStep*4];
	IP_FTM1->CONTROLS[5].CnV = StepCurve[CurrentStep*4+1];
	IP_FTM1->CONTROLS[6].CnV = StepCurve[CurrentStep*4+2];
	IP_FTM1->CONTROLS[7].CnV = StepCurve[CurrentStep*4+3];
}

// Output Stepper [1..4],<steps>,<dir>,<freq>
void hb_stepper(char *p) {
    int motor, steps;

    getargs(&p, 5, ",");
    if(!(argc == 5)) error("Argument count");
    motor = getint(argv[0], 1, 4)-1;        // maximum of 2 motors to be controlled
    										// but motor 3/4 is the same as motor 1/2 but half step instead of full step
    if(motor>1) {
    	motor -=2;
    	Stepper[motor].Mode = 8;			// half step operation
    }
    else
    	Stepper[motor].Mode = 4;			// full step operation

    // motor is now either 0 or 1
    if((HB_MotorConfig & (0b11<<(motor*2))) != 0)	// is the requested H-Bridge configured correctly?
    	error("Wrong H-Bridge config");
    steps=getinteger(argv[2]);
    if(steps<0) {
    	Stepper[motor].StepsToGo = -steps;
    	Stepper[motor].Direction = -1;
    } else {
    	Stepper[motor].StepsToGo = steps;
    	Stepper[motor].Direction = 1;
    }

    IP_LPIT0->TMR[motor].TVAL = 8000000/getint(argv[4], 5, 250);			// delay between steps
    IP_LPIT0->TMR[motor].TCTRL = LPIT_TMR_TCTRL_T_EN(1);					// enable channel
    while(IP_LPIT0->TMR[motor].TCTRL) {
		if(MMAbort) {
		    IP_LPIT0->TMR[motor].TCTRL = 0x0;					// disable channel
			WDTimer = 0;                                                // turn off the watchdog timer
			longjmp(mark, 1);                                           // jump back to the input prompt
		}
    }
}

void cmd_output(void) {
    char *p;

    if((p = checkstring(cmdline, "CONF")) != NULL)
        hb_conf(p);
    else if((p = checkstring(cmdline, "SET")) != NULL)
        hb_set(p);
    else if((p = checkstring(cmdline, "WRITE")) != NULL)
        hb_write(p);
    else if((p = checkstring(cmdline, "MOTOR")) != NULL)
        hb_motor(p);
    else if((p = checkstring(cmdline, "ENCMOTOR")) != NULL)
        hb_encmotor(p);
//    else if((p = checkstring(cmdline, "ENCMOTOR2")) != NULL)
//        hb_encmotor2(p);
    else if((p = checkstring(cmdline, "STEPPER")) != NULL)
    	hb_stepper(p);
    else
        error("Unknown command");
}

void cmd_epull(void){
	int inputs, pull;

    getargs(&cmdline, 3, ",");
    if(argc != 3) error("Argument count");
    inputs = getint(argv[0], 0, 255);		// each "bit" represents an input
    pull = getint(argv[2], 0, 255);			// PU/PD config
    I2C_SetEInput(3,inputs^0xff);			// switch selected pins to output
    I2C_SetEInput(1,pull);					// write pull config
}

// distance(trig, echo)
// trig can be in range of 1..4 reflecting the Cx input (used as output for this function)
// echo can be in the range of 1..8 reflecting the Ex input measuring the echo pulse
void fun_distance(void) {
    int trig, echo, counter;

    static unsigned int* CPortDDR[] = {
			(unsigned int*)&(IP_PTE->PDDR),
			(unsigned int*)&(IP_PTD->PDDR),
			(unsigned int*)&(IP_PTA->PDDR),
			(unsigned int*)&(IP_PTB->PDDR)
    };
    /*
    static unsigned int* CPortPDIR[] = {
			(unsigned int*)&(IP_PTE->PDIR),
			(unsigned int*)&(IP_PTD->PDIR),
			(unsigned int*)&(IP_PTA->PDIR),
			(unsigned int*)&(IP_PTB->PDIR)
    }; */
    static const unsigned char CPortNum[] = {
    		8,
			5,
			6,
			13
    };
    static const int* EPortPDIR[] = {
			(int*)&(IP_PTC->PDIR),
			(int*)&(IP_PTC->PDIR),
			(int*)&(IP_PTC->PDIR),
			(int*)&(IP_PTC->PDIR),
			(int*)&(IP_PTA->PDIR),
			(int*)&(IP_PTA->PDIR),
			(int*)&(IP_PTB->PDIR),
			(int*)&(IP_PTB->PDIR)
    };
    static const unsigned char EPortNum[] = {
    		3,
			2,
			1,
			16,
			0,
			1,
			1,
			0
    };

	getargs(&ep, 3, ",");
	if((argc & 1) != 1) error("Argument count");
	trig = getint(argv[0], 1, 4)-1;
	if(argc == 3)
		echo = getint(argv[2], 1, 8)-1;
	else
		echo = trig;                                                // they are the same if it is a 3-pin device (not yet supported/tested due to lack of required HW)
	*CPortDDR[trig] |= 1<<CPortNum[trig];							// drive trigger channel LOW
	uSec(100);
	*CPortDDR[trig] &= ~(1<<CPortNum[trig]);						// drive it high
	uSec(50);
	PauseTimer = 0;                                                 // this counter counts in 1ms steps
	while(!((*EPortPDIR[echo])&(1<<EPortNum[echo])))				// wait for rising edge detected (start of pulse)
	{
		if(PauseTimer>50)
		{
			fret=-1;
			goto exitfun;
		}
	}
	S32_SysTick->CVR = 0;											// reset counter
	while((*EPortPDIR[echo])&(1<<EPortNum[echo]))					// wait for falling edge detect (end of pulse)
	{
		if(PauseTimer>100)
		{
			fret=-2;
			goto exitfun;
		}
	}
	counter=S32_SysTick->CVR;										// store counter value
	// Now calulate the distance in mm
	// pulse length reflects echo time and echo means, we send something and receive something -> 2 times the distance and therefore, we need to divide by 2
	// @25oC we have a speed of the pulse at 343.3m/s through the air but we want the distance in mm => 343300mm/s
	//
	// => Distance=(343300 * Pulse)/2
	// Pulse = (0xffffff - counter)/CounterFrequency
	// 0xffffff is the initial counter value and it counts down @ MCU core frequency, which is 80MHz
	//
	// Pulse = (343300 * (0xffffff-counter))/(2*80e6) = (0xffffff-counter)*343300/160000000 = (0xffffff-counter)*3433/1600000
	// Now we adapt the division to a binary friendly instruction (shift right): 2097152/1600000 = 1.31072 => 3433*1,31072 = 4500
	//
	// finally we get the formula:
	fret=(MMFLOAT)((0xffffff-counter)*4500/2097152);	// divide by 2097152 (=1*2^21) means just a shift of the register content to the right by 21 bits
	exitfun:
	targ = T_NBR;
}

/*
 * Here we start with the MCU specific part of this project.
 * It's mainly setup/initialization of the required modules
 */
void Init_Ports(void)
{
	volatile unsigned int CNT;

	// Port initialization stuff
	IP_PCC->PCCn[PCC_PORTA_INDEX] |= PCC_PCCn_CGC_MASK;	/* Enable clock for PORTA */
	IP_PCC->PCCn[PCC_PORTB_INDEX] |= PCC_PCCn_CGC_MASK;	/* Enable clock for PORTB */
	IP_PCC->PCCn[PCC_PORTC_INDEX] |= PCC_PCCn_CGC_MASK;	/* Enable clock for PORTC */
	IP_PCC->PCCn[PCC_PORTD_INDEX] |= PCC_PCCn_CGC_MASK;	/* Enable clock for PORTD */
	IP_PCC->PCCn[PCC_PORTE_INDEX] |= PCC_PCCn_CGC_MASK;	/* Enable clock for PORTE */

	// before configuring UART/CAN I have to read in mode setting
	IP_PORTC->PCR[PTC7] = PORT_PCR_MUX(1)
					 | PORT_PCR_PE(1)		// enable pull resistor
					 | PORT_PCR_PS(1)		// pull is a pull-up resistor
					 ;
	IP_PORTE->PCR[PTE5] = PORT_PCR_MUX(1)
					 | PORT_PCR_PE(1)		// enable pull resistor
					 | PORT_PCR_PS(1)		// pull is a pull-up resistor
					 ;

	// pin configuration for LED
	IP_PORTE->PCR[LED] = PORT_PCR_MUX(1)	// configure pin to GPIO usage
					| PORT_PCR_LK(1);	// and lock config until next reset
	IP_PTE->PCOR = 1<<LED;		// set pin = LED on
	IP_PTE->PDDR |= 1<<LED;	// pin is output
	CNT=640000;
	while(CNT>0)
	{
		CNT--;
	}
	LEDOff;
	CNT=640000;
	while(CNT>0)
	{
		CNT--;
	}
	LEDOn;

	ModeSetting = ((IP_PTC->PDIR & (1<<PTC7))>>(PTC7-1)) | ((IP_PTE->PDIR & (1<<PTE5))>>PTE5);

	// map PTC[7:6] to LPUART1
	IP_PORTC->PCR[PTC6] = PORT_PCR_MUX(2)		// RXD
					 | PORT_PCR_LK(1);		// and lock configuration until next reset
	IP_PORTC->PCR[PTC7] = PORT_PCR_MUX(2)		// TXD
					 | PORT_PCR_LK(1);		// and lock configuration until next reset
	//IP_PTC->PDDR |= 1<<PTC7;
	IP_PORTB->PCR[PTB0] = PORT_PCR_MUX(1);

	// map PTD[16:15] and PTB[5:4] to LPSPI1
	IP_PORTB->PCR[PTB4] = PORT_PCR_MUX(3)		// SDO
					 | PORT_PCR_LK(1);		// and lock configuration until next reset

#ifdef CLKOUT
	IP_PORTB->PCR[PTB5] = PORT_PCR_MUX(5)		// CLKOUT
					 | PORT_PCR_LK(1);		// and lock configuration until next reset
	IP_SCG->CLKOUTCNFG = SCG_CLKOUTCNFG_CLKOUTSEL(6);	// select PLL clock
	SIM->CHIPCTL |= SIM_CHIPCTL_CLKOUTDIV(7)|SIM_CHIPCTL_CLKOUTEN(1);
#else
	IP_PORTB->PCR[PTB5] = PORT_PCR_MUX(4);		// CE/PCS0
#endif

	IP_PORTD->PCR[PTD15] = PORT_PCR_MUX(4)		// SCK
					  | PORT_PCR_LK(1);			// and lock configuration until next reset

	IP_PORTD->PCR[PTD16] = PORT_PCR_MUX(4)		// SDI
					  | PORT_PCR_LK(1);			// and lock configuration until next reset

	// map PTA[3:2] to LPI2C0
	IP_PORTA->PCR[PTA2] = PORT_PCR_MUX(3)		// SDA
					 | PORT_PCR_LK(1);			// and lock configuration until next reset

	IP_PORTA->PCR[PTA3] = PORT_PCR_MUX(3)		// SCL
					 | PORT_PCR_LK(1);			// and lock configuration until next reset

	// map PTE[5:4] to CAN0
	IP_PORTE->PCR[PTE4] = PORT_PCR_MUX(5)		// RX
					 | PORT_PCR_LK(1);			// and lock configuration until next reset

	IP_PORTE->PCR[PTE5] = PORT_PCR_MUX(5)		// TX
					 | PORT_PCR_LK(1);			// and lock configuration until next reset

	IP_PORTE->PCR[PTE10] = PORT_PCR_MUX(1)		// RTC_IRQ (Either input to get notified by an alarm or output to wake up the Raspberry Pi)
					 | PORT_PCR_LK(1);			// and lock configuration until next reset
	IP_PTE->PDDR &= ~(1<<PTE10);				// Configure PTE10 to be input
	IP_PTE->PDOR &= ~(1<<PTE10);				// Set output value to "0" once PDDR for PTE10 is set to "1"

	// Initialize GPIOs for H-Bridges and set them all to low
	IP_PTA->PDDR |= (1<<PTA10)|(1<<PTA11)|(1<<PTA12)|(1<<PTA13);
	IP_PTB->PDDR |= (1<<PTB2)|(1<<PTB3);
	IP_PTC->PDDR |= (1<<PTC14)|(1<<PTC15);
	IP_PTA->PCOR = (1<<PTA10)|(1<<PTA11)|(1<<PTA12)|(1<<PTA13);
	IP_PTB->PCOR = (1<<PTB2)|(1<<PTB3);
	IP_PTC->PCOR = (1<<PTC14)|(1<<PTC15);
	//HB_Mode(GPIO);

	// configure fault pins for H-Bridges
	IP_PORTC->PCR[FaultPin1] =   PORT_PCR_MUX(1)	// enable pin as GPIO
							| PORT_PCR_PE(1)	// enable pull resistor
							| PORT_PCR_PS(1)	// configure pull as pull-up
							| PORT_PCR_LK(1);	// and lock configuration until next reset
	IP_PORTC->PCR[FaultPin2] =   PORT_PCR_MUX(1)	// enable pin as GPIO
							| PORT_PCR_PE(1)	// enable pull resistor
							| PORT_PCR_PS(1)	// configure pull as pull-up
							| PORT_PCR_LK(1);	// and lock configuration until next reset

	// Configure pins for DIP switch
	// PTC[5]:PTD[3:0]
	IP_PTD->PDDR &= ~((1<<PTD0)|(1<<PTD1)|(1<<PTD2)|(1<<PTD3));	// set PTD[3:0] to inputs
	IP_PTC->PDDR &= ~(1<<PTC5);				// set PTC5 to input

	IP_PORTD->GPCLR = (((1<<PTD0)|(1<<PTD1)|(1<<PTD2)|(1<<PTD3))<<16)	// select PTD[3:0] at once
				 | PORT_PCR_MUX(1)		// GPIO functionality
				 | PORT_PCR_PE(1)		// enable pull resistor
				 | PORT_PCR_PS(1)		// configure pull as pull-up
				 | PORT_PCR_LK(1);		// and lock configuration until next reset

	IP_PORTC->PCR[PTC5] = PORT_PCR_MUX(1)		// GPIO functionality
					 | PORT_PCR_PE(1)		// enable pull resistor
					 | PORT_PCR_PS(1)		// configure pull as pull-up
					 | PORT_PCR_LK(1);		// and lock configuration until next reset

	// OSC pins
	IP_PORTB->PCR[PTB6] = PORT_PCR_MUX(0)			// set port to disable
					 | PORT_PCR_LK(1);			// and lock configuration until next reset
	IP_PORTB->PCR[PTB7] = PORT_PCR_MUX(0)			// set port to disable
					 | PORT_PCR_LK(1);			// and lock configuration until next reset

	// Configure PTA6, PTB13, PTD5 and PTE8 pin to GPIO mode and enable eDMA request on falling edge
    IP_PORTA->PCR[PTA6] = PORT_PCR_MUX(1)
					 | PORT_PCR_PE(1)			// enable pull resistor
					 | PORT_PCR_PS(1)			// configure pull as pull-up
    				 | PORT_PCR_IRQC(2)
					 | PORT_PCR_LK(1);
    IP_PORTB->PCR[PTB13] = PORT_PCR_MUX(1)
					 | PORT_PCR_PE(1)			// enable pull resistor
					 | PORT_PCR_PS(1)			// configure pull as pull-up
    				 | PORT_PCR_IRQC(2)
					 | PORT_PCR_LK(1);
    IP_PORTD->PCR[PTD5] = PORT_PCR_MUX(1)
					 | PORT_PCR_PE(1)			// enable pull resistor
					 | PORT_PCR_PS(1)			// configure pull as pull-up
    				 | PORT_PCR_IRQC(2)
					 | PORT_PCR_LK(1);
    IP_PORTE->PCR[PTE8] = PORT_PCR_MUX(1)
					 | PORT_PCR_PE(1)			// enable pull resistor
					 | PORT_PCR_PS(1)			// configure pull as pull-up
    				 | PORT_PCR_IRQC(2)
					 | PORT_PCR_LK(1);

    // control H-Bridge pins with FTM1
    IP_PORTA->GPCLR = (((1<<PTA10)|(1<<PTA11)|(1<<PTA12)|(1<<PTA13))<<16)
                  | (PORT_PCR_MUX(2));
    IP_PORTB->GPCLR = (((1<<PTB2)|(1<<PTB3))<<16)
                  | (PORT_PCR_MUX(2));
    IP_PORTC->GPCLR = (((1<<PTC14)|(1<<PTC15))<<16)
                  | (PORT_PCR_MUX(2));

	IP_PORTC->GPCLR = (((1<<PTC1)|(1<<PTC2)|(1<<PTC3))<<16)	// select PTC[3:1] at once
				 | PORT_PCR_MUX(1)		// GPIO functionality (A/D will use these pins as well)
				 | PORT_PCR_LK(1);		// and lock configuration until next reset
	IP_PORTC->PCR[16] = PORT_PCR_MUX(1)		// GPIO functionality (A/D will use these pins as well)
				   | PORT_PCR_LK(1);		// and lock configuration until next reset
	IP_PORTA->GPCLR = (((1<<PTA0)|(1<<PTA1))<<16)	// select PTA[1:0] at once
				 | PORT_PCR_MUX(1)		// GPIO functionality (A/D will use these pins as well)
				 | PORT_PCR_LK(1);		// and lock configuration until next reset
	IP_PORTB->GPCLR = (((1<<PTB0)|(1<<PTB1))<<16)	// select PTB[1:0] at once
				 | PORT_PCR_MUX(1)		// GPIO functionality (A/D will use these pins as well)
				 | PORT_PCR_LK(1);		// and lock configuration until next reset
}

void DMAMUX_Init()
{
	/* Enable clock for DMAMUX */
	IP_PCC->PCCn[PCC_DMAMUX_INDEX] = PCC_PCCn_CGC_MASK;

	/* DMA Channel Source Select - PORTE and DMA Enable 53*/
	IP_DMAMUX->CHCFG[0] = DMAMUX_CHCFG_SOURCE(53) | DMAMUX_CHCFG_ENBL_MASK;
	/* DMA Channel Source Select - PORTD and DMA Enable 52*/
	IP_DMAMUX->CHCFG[1] = DMAMUX_CHCFG_SOURCE(52) | DMAMUX_CHCFG_ENBL_MASK;
	/* DMA Channel Source Select - PORTA and DMA Enable 49*/
	IP_DMAMUX->CHCFG[2] = DMAMUX_CHCFG_SOURCE(49) | DMAMUX_CHCFG_ENBL_MASK;
	/* DMA Channel Source Select - PORTB and DMA Enable 50*/
	IP_DMAMUX->CHCFG[3] = DMAMUX_CHCFG_SOURCE(50) | DMAMUX_CHCFG_ENBL_MASK;
}

void DMA0to3_Init()
{
	unsigned int LoopCounter;

	for(LoopCounter=0; LoopCounter<=3; LoopCounter++)
	{
		/* Source Configuration */
		IP_DMA->TCD[LoopCounter].SADDR = DMA_TCD_SADDR_SADDR(&dummy[LoopCounter]);
		IP_DMA->TCD[LoopCounter].ATTR = DMA_TCD_ATTR_SSIZE(0)|DMA_TCD_ATTR_DSIZE(0); // Source and destination size is 8-bit
		IP_DMA->TCD[LoopCounter].SOFF = 0; // no address shift after each transfer
		IP_DMA->TCD[LoopCounter].SLAST = 0;

		/* Destination Configuration */
		IP_DMA->TCD[LoopCounter].DADDR = DMA_TCD_DADDR_DADDR(&dummy[LoopCounter]);
		IP_DMA->TCD[LoopCounter].DOFF = 0; // no address shift after each transfer
		IP_DMA->TCD[LoopCounter].DLASTSGA = 0;

		/* Set Citer and Biter to Maximum Value */
		IP_DMA->TCD[LoopCounter].CITER.ELINKNO = DMA_TCD_CITER_ELINKNO_CITER_MASK;
		IP_DMA->TCD[LoopCounter].BITER.ELINKNO = DMA_TCD_BITER_ELINKNO_BITER_MASK;
		IP_DMA->TCD[LoopCounter].NBYTES.MLNO = 1; // transfer one byte on each trigger arrived
	}
}

void DMA_Enable(void)
{

	/* Start Transfer for Channel0 */
	//IP_DMA->SERQ = DMA_SERQ_SERQ(0);
	//IP_DMA->SERQ = DMA_SERQ_SERQ(1);
	//IP_DMA->SERQ = DMA_SERQ_SERQ(2);
	//IP_DMA->SERQ = DMA_SERQ_SERQ(3);
	IP_DMA->ERQ = 0x0f;				// enable DMA channel 0-3
}

void Init_ADC(void)
{
    IP_PCC->PCCn[PCC_ADC0_INDEX] = PCC_PCCn_PCS(3); /* PCS = 3: Select FIRCDIV2 for ALTCLK1 */
    IP_PCC->PCCn[PCC_ADC0_INDEX] |= PCC_PCCn_CGC_MASK; /* Enable bus clock in ADC */

    // set up clock, data width and sampling time
    IP_ADC0->CFG1 = ADC_CFG1_ADIV(3) | ADC_CFG1_MODE(1); /* ADIV = 3: Divide ratio = 8 -> 6MHz */
    /* MODE = 'b01: 12-bit conversion */
    IP_ADC0->CFG2 = ADC_CFG2_SMPLTS(207); /* SMPLTS = 207: sample time is 208 ADC clks (~35us@6MHz) */
    /*
     * ADC conversion time:
     * (SMPLTS+1) + 1 (fixed) + 28 (10-bit: 24; 8-bit: 20; 12-bit 28) + 10 (fixed) =
     *    208 + 1 + 28 + 10 = 247cycles @6MHz -> 41.2us -> 8 channels finished after 330us
     */

    // Calibration first
    IP_ADC0->SC3 = ADC_SC3_CAL_MASK /* CAL = 1: Start calibration sequence */
    | ADC_SC3_AVGE_MASK /* AVGE = 1: Enable hardware average */
    | ADC_SC3_AVGS(3); /* AVGS = 11b: 32 samples averaged */
    /* Wait for completion */
    while(((IP_ADC0->SC1[0] & ADC_SC1_COCO_MASK)>>ADC_SC1_COCO_SHIFT) == 0);

    IP_ADC0->SC2 = ADC_SC2_ADTRG(1);		/* ADTRG = 1: HW trigger */
    IP_ADC0->SC3 = ADC_SC3_ADCO(0)			/* ADCO = 0: One conversion performed */
              | ADC_SC3_AVGE(0)			/* AVGE,AVGS = 0: HW average function disabled */
			  | ADC_SC3_AVGS(0);

    IP_ADC0->SC1[0] = ADC_SC1_ADCH(11);    /* External channel 11 as I1 */
    IP_ADC0->SC1[1] = ADC_SC1_ADCH(10);    /* External channel 10 as I2 */
    IP_ADC0->SC1[2] = ADC_SC1_ADCH(9);		/* External channel 9 as I3 */
    IP_ADC0->SC1[3] = ADC_SC1_ADCH(14);    /* External channel 14 as I4 */
    IP_ADC0->SC1[4] = ADC_SC1_ADCH(0);     /* External channel 0 as I5 */
    IP_ADC0->SC1[5] = ADC_SC1_ADCH(1);     /* External channel 1 as I6 */
    IP_ADC0->SC1[6] = ADC_SC1_ADCH(5);     /* External channel 5 as I7 */
    IP_ADC0->SC1[7] = ADC_SC1_ADCH(4);     /* External channel 4 as I8 */
    IP_ADC0->SC1[8] = ADC_SC1_ADCH(3);     /* External channel 3 as Poti */
}

void Init_PDB(void)
{
    IP_PCC->PCCn[PCC_PDB0_INDEX] |= PCC_PCCn_CGC_MASK; 	/* Enable sysclock (80MHz) for PDB */
    IP_PDB0->SC = PDB_SC_PRESCALER(4) 					/* PRESCALER = 4: clk divided by (16 x Mult factor) */
    | PDB_SC_TRGSEL(15)									/* TRGSEL = 15: Software trigger selected */
    | PDB_SC_MULT(1); 									/* MULT = 1: Multiplication factor is 10 */

    /* PDB time base = 1/(80MHz/(16 * 10)) = 2us per CNT tick */
    IP_PDB0->MOD = 200; 								// One PDB cycle is 400us

    /* PDB Period = (System Clock / (Prescaler x Mult factor)) / Modulus */
    /* PDB Period = (80 MHz / (16 x 10)) / 200 */
    /* PDB Period = (80 MHz) / (32000) = 2.5kHz (400us per PDB cycle) */
    IP_PDB0->CH[0].C1 = (PDB_C1_BB(0x7f) 				/* BB = 7fh: Back-to-back for pre-triggers [6:0] */
    | PDB_C1_TOS(0x00) 									/* TOS = ffh: all pre-trigger assert B2B w/o delay */
    | PDB_C1_EN(0xff));									/* EN = Ffh: all pre-trigger enabled */
    IP_PDB0->CH[1].C1 = (PDB_C1_BB(0x00) 				/* no B2B capability (just 1 channel) */
    | PDB_C1_TOS(0x01) 									/* TOS = 01h: Pre-trigger 0 start with dly match */
    | PDB_C1_EN(0x01)); 								/* EN = 01h: Pre-trigger 0 enabled */
    IP_PDB0->CH[1].DLY[0] = 175; 						/* Start CH1 delayed after 350us (CH0 has finished after ~330us as each conversion requires ~41.2us */

    IP_PDB0->SC |= PDB_SC_PDBEN_MASK | PDB_SC_LDOK_MASK; /* Enable PDB. Load MOD and DLY */
}

// Initializing LPIT which is used for stepper motor control
void Init_LPIT(void)
{
	IP_PCC->PCCn[PCC_LPIT_INDEX] = 0;						// Disable clock to change PCS
	IP_PCC->PCCn[PCC_LPIT_INDEX] = PCC_PCCn_PCS(1);		// PCS = 1: Select SOSCDIV2 (8MHz)
	IP_PCC->PCCn[PCC_LPIT_INDEX] |= PCC_PCCn_CGC_MASK;		// Enable clock LPIT

	IP_LPIT0->MCR = LPIT_MCR_M_CEN(1);						// Enable module
    IP_LPIT0->MIER = LPIT_MIER_TIE0(1)|LPIT_MIER_TIE1(1);	// enable IRQ for channel 0&1
	S32_NVIC->ISER[(LPIT0_Ch0_IRQn>>5)] |= (uint32_t)(1U << ((uint32_t)(LPIT0_Ch0_IRQn) & (uint32_t)0x1FU));
	S32_NVIC->ISER[(LPIT0_Ch1_IRQn>>5)] |= (uint32_t)(1U << ((uint32_t)(LPIT0_Ch1_IRQn) & (uint32_t)0x1FU));
}

// Initialize FTM1 used for PWM mode, driving the H-Bridges
void Init_FTM1(void)
{
//	IP_FTM1->SC = 0;							// switch FTM off
	IP_PCC->PCCn[PCC_FTM1_INDEX] = 0;						// Disable clock to change PCS
	IP_PCC->PCCn[PCC_FTM1_INDEX] = PCC_PCCn_PCS(3);		// PCS = 3: Select FIRCDIV1 (24MHz)
	IP_PCC->PCCn[PCC_FTM1_INDEX] |= PCC_PCCn_CGC_MASK;		// Enable clock FTM1

	IP_FTM1->SC = FTM_SC_PS(1)							// divide by 2 of FIRCDIV1 -> 24MHz/2 = 12MHz
					|  FTM_SC_PWMEN0(1)
					|  FTM_SC_PWMEN1(1)
					|  FTM_SC_PWMEN2(1)
					|  FTM_SC_PWMEN3(1)
					|  FTM_SC_PWMEN4(1)
					|  FTM_SC_PWMEN5(1)
					|  FTM_SC_PWMEN6(1)
					|  FTM_SC_PWMEN7(1);

	IP_FTM1->CONTROLS[0].CnSC = FTM_CnSC_MSB(1)			// First 4 channels for HB1
						   | FTM_CnSC_MSA(0)
						   | FTM_CnSC_ELSB(1)
						   | FTM_CnSC_ELSA(0);			// configure channel 0 PTB2 (Q1) for edge-aligned PWM
	IP_FTM1->CONTROLS[1].CnSC = FTM_CnSC_MSB(1)
						   | FTM_CnSC_MSA(0)
						   | FTM_CnSC_ELSB(1)
						   | FTM_CnSC_ELSA(0);			// configure channel 1 PTB3 (Q2) for edge-aligned PWM
	IP_FTM1->CONTROLS[2].CnSC = FTM_CnSC_MSB(1)
						   | FTM_CnSC_MSA(0)
						   | FTM_CnSC_ELSB(1)
						   | FTM_CnSC_ELSA(0);			// configure channel 2 PTC14 (Q3) for edge-aligned PWM
	IP_FTM1->CONTROLS[3].CnSC = FTM_CnSC_MSB(1)
						   | FTM_CnSC_MSA(0)
						   | FTM_CnSC_ELSB(1)
						   | FTM_CnSC_ELSA(0);			// configure channel 3 PTC15 (Q4) for edge-aligned PWM
	IP_FTM1->CONTROLS[4].CnSC = FTM_CnSC_MSB(1)			// Following channels are HB2
						   | FTM_CnSC_MSA(0)
						   | FTM_CnSC_ELSB(1)
						   | FTM_CnSC_ELSA(0);			// configure channel 4 PTA10 (Q1) for edge-aligned PWM
	IP_FTM1->CONTROLS[5].CnSC = FTM_CnSC_MSB(1)
						   | FTM_CnSC_MSA(0)
						   | FTM_CnSC_ELSB(1)
						   | FTM_CnSC_ELSA(0);			// configure channel 5 PTA11 (Q2) for edge-aligned PWM
	IP_FTM1->CONTROLS[6].CnSC = FTM_CnSC_MSB(1)
						   | FTM_CnSC_MSA(0)
						   | FTM_CnSC_ELSB(1)
						   | FTM_CnSC_ELSA(0);			// configure channel 6 PTA12 (Q3) for edge-aligned PWM
	IP_FTM1->CONTROLS[7].CnSC = FTM_CnSC_MSB(1)
						   | FTM_CnSC_MSA(0)
						   | FTM_CnSC_ELSB(1)
						   | FTM_CnSC_ELSA(0);			// configure channel 7 PTA13 (Q4) for edge-aligned PWM

	IP_FTM1->MOD = MaxPwmPeriod;						// PWM period is 57142 ticks => 210Hz
	IP_FTM1->CNT = 0;
	IP_FTM1->CONTROLS[0].CnV = 0;			// off (this is the period for channel 0)
	IP_FTM1->CONTROLS[1].CnV = 0;			// off (this is the period for channel 1)
	IP_FTM1->CONTROLS[2].CnV = 0;			// off (this is the period for channel 2)
	IP_FTM1->CONTROLS[3].CnV = 0;			// off (this is the period for channel 3)

	IP_FTM1->SC |= FTM_SC_CLKS(3);			// enable timer with external clock as source clock (FIRCDIV1)
}

void Init_LPTimer(void){
	// Enable clock to LP-Timer
	IP_PCC->PCCn[PCC_LPTMR0_INDEX] = PCC_PCCn_PCS(1)|PCC_PCCn_PCD(7); /* PCS = 1: Select SOSCDIV2 div 8 = 1MHz */
	IP_PCC->PCCn[PCC_LPTMR0_INDEX] |= PCC_PCCn_CGC_MASK;	// Enable clock for LPTMR0
	IP_LPTMR0->CMR = 1000;									// timebase set to 1ms (1MHz/1000)
	IP_LPTMR0->PSR = LPTMR_PSR_PCS(3)|LPTMR_PSR_PBYP(1);	// input #3 and prescaler bypass
}

int ClockSetup(void) {

	int ClockSetupError=10000;

	// FIRCDIV1 (24MHz) is used by
	// FTM1 for H-Bridge motor control
	//
	// FIRCDIV2 (48MHz) is used by
	// LPUART1 for console
	// ADC0 for E_Input sampling
	//
	// SOSCDIV2 (8MHz) is used by
	// LPTMR to generate system interrupt (1ms)
	// LPIT0 driving the stepper motors
	// I2C0
	//
	// SPLLDIV1 (80MHz) is used by
	// nothing...yet
	//
	// Used blocks running on SYS_CLK (80MHz) beside the standard blocks like Core, Flash, RAM, GPIO etc.
	// PDB (used for ADC trigger)
	// DMA (used for counter implementation 2)

	// FXOSC setup to work with 16MHz crystal attached
	IP_SCG->SOSCCFG = SCG_SOSCCFG_EREFS(1)		// select internal oscillator
				 | SCG_SOSCCFG_RANGE(3);	// with range of 8-40MHz crystal
	IP_SCG->SOSCCSR = SCG_SOSCCSR_SOSCEN(1)	// enable oscillator
				 | SCG_SOSCCSR_SOSCCM(1);	// enable OSC clock monitor
	while(!(IP_SCG->SOSCCSR & SCG_SOSCCSR_SOSCVLD_MASK)			// wait until clock is valid
			&& !(ClockSetupError--));
	if (!ClockSetupError) return CAUSE_CLOCK;
	IP_SCG->SOSCDIV = SCG_SOSCDIV_SOSCDIV1(5)|SCG_SOSCDIV_SOSCDIV2(2);		// SOSCDIV1 = 16MHz/16 = 1MHz | SOSCDIV2 = 16MHz/2 = 8MHz

	// Configure PLL with SOSC as source div 2 and multiply by 20 => (16MHz/2)*20 = 160 => 80MHz system clock
	IP_SCG->SPLLCFG = SCG_SPLLCFG_PREDIV(1)|SCG_SPLLCFG_MULT(4);
	IP_SCG->SPLLCSR = SCG_SPLLCSR_SPLLEN(1);				// enable PLL
	while(!(IP_SCG->SPLLCSR & SCG_SPLLCSR_SPLLVLD_MASK)	// wait for clock being stable
			&& !(ClockSetupError--));
	if (!ClockSetupError) return CAUSE_CLOCK;
	//IP_SCG->SPLLDIV = SCG_SPLLDIV_SPLLDIV1(1);				// SPLLDIV1 = 80MHz

	// System clock initialization
	IP_SCG->RCCR = SCG_RCCR_SCS(6)|SCG_RCCR_DIVCORE(0)|SCG_RCCR_DIVBUS(1)|SCG_RCCR_DIVSLOW(3); // core running @80MHz, bus @40MHz and Flash @20MHz
	IP_SCG->FIRCDIV = SCG_FIRCDIV_FIRCDIV1(2) | SCG_FIRCDIV_FIRCDIV2(1);	 // FIRCDIV1 div 2 = 24MHz; FIRCDIV2 no division = 48MHz
	IP_SCG->SIRCDIV = SCG_SIRCDIV_SIRCDIV1(2);								// div 2 -> SIRCDIV1 = 4MHz

	return 1;
}
void CallBootLoader(void)
{
	uint32_t *pSrc, *pDest;

	/* First, we have to copy the BootLoader code/vars/vectors into the RAM */
	pSrc  = &__BootLoader_Flash_Start__;
	pDest = &__BootLoader_Ram_Start__;
	while (pSrc < &__BootLoader_Flash_End__)
	{
		*pDest = *pSrc;
		pSrc++;
		pDest++;
	}

	BootLoader();	/* Now call the BootLoader in the RAM */
}

void HardFault_Handler(void) {
	// We get here by accessing something which is NOT in the memory map
	// most likely wrong firmware is installed and RAM is assumed where non is, so let's go to the boot loader
	CallBootLoader();
}

void InitProcessor(void) {

	ClockSetup();

    // setup the CPU
    BusSpeed = 80000000;                 // System config performance

	// Initialize used I/O pins
    Init_Ports();

	// Load MMBasic options
    LoadOptions();                                                  // populate the Option struct from flash

	// Now, all which is needed to do BootLoader, check on request for it
	if(ModeSetting==0)
	{
		CallBootLoader();
	}

   	// Initialize ADC
	Init_ADC();
	Init_PDB();

	// configure DMA for pulse counting
	DMAMUX_Init();
	DMA0to3_Init();

	// Initialize FTM1 for PWM on H-Bridge inputs
	Init_FTM1();

	// Initialize LPIT for stepper
	Init_LPIT();

	// Initialize LPTimer for H-Bridge fault handling
	Init_LPTimer();

    // set the base of the usable memory
    RAMBase = (void *)&__MMBASIC_RAM_BASE__;							// This is a PAGESIZE aligned memory location where MMBASIC starts to use RAM

    initConsole();                                                  // Initialize the UART used for the console
    InitHeap();                                                     // Initialize memory allocation
    initTimers();                                                   // Initialize and startup the timers

	// enable SysTick timer as free running timer @80MHz => max time to reach 0 = 209.7ms
    // This one is used for uSec function in main.c
	S32_SysTick->RVR = 0xffffff;									// define max value
	S32_SysTick->CSR = S32_SysTick_CSR_ENABLE_MASK;					// enable SysTick timer


	ENABLE_INTERRUPTS();
	DMA_Enable();

	// enable LPI2C0 IRQ as it's required to initialize the H-Bridges
	S32_NVIC->ISER[(LPI2C0_Master_IRQn>>5)] |= (uint32_t)(1U << ((uint32_t)(LPI2C0_Master_IRQn) & (uint32_t)0x1FU));
	// Now everything's up and running, initialize peripherals
	Init_HBridges();

	// and enable Cache to speed up the things...at least a bit ;-)
    IP_LMEM->PCCCR = LMEM_PCCCR_INVW0(1) | LMEM_PCCCR_INVW1(1) | LMEM_PCCCR_GO(1) | LMEM_PCCCR_ENCACHE(1);		// enable cache
}


