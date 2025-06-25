
/************************************************************************************************************************
fischertechnik-Raspberry Pi-Stand-Alone Controller

ft-rpi-sa_Controller.h

Contains all the required board specific defines of the ft-RPI-sa Controller

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

#ifndef FT_RPI_SA_CONTROLLER_FT_RPI_SA_CONTROLLER_H_
#define FT_RPI_SA_CONTROLLER_FT_RPI_SA_CONTROLLER_H_

enum {
	PTA0=0,
	PTA1,
	PTA2,
	PTA3,
	PTA4,
	PTA5,
	PTA6,
	PTA7,
	PTA8_NC,
	PTA9_NC,
	PTA10,
	PTA11,
	PTA12,
	PTA13
};

enum {
	PTB0=0,
	PTB1,
	PTB2,
	PTB3,
	PTB4,
	PTB5,
	PTB6,
	PTB7,
	PTB8_NC,
	PTB9_NC,
	PTB10_NC,
	PTB11_NC,
	PTB12_NC,
	PTB13
};

enum {
	PTC0_NC=0,
	PTC1,
	PTC2,
	PTC3,
	PTC4,
	PTC5,
	PTC6,
	PTC7,
	PTC8,
	PTC9,
	PTC10_NC,
	PTC11_NC,
	PTC12_NC,
	PTC13_NC,
	PTC14,
	PTC15,
	PTC16,
	PTC17_NC
};

enum {
	PTD0=0,
	PTD1,
	PTD2,
	PTD3,
	PTD4,
	PTD5,
	PTD6_NC,
	PTD7_NC,
	PTD8_NC,
	PTD9_NC,
	PTD10_NC,
	PTD11_NC,
	PTD12_NC,
	PTD13_NC,
	PTD14_NC,
	PTD15,
	PTD16,
	PTD17_NC
};

enum {
	PTE0_NC=0,
	PTE1_NC,
	PTE2_NC,
	PTE3_NC,
	PTE4,
	PTE5,
	PTE6_NC,
	PTE7_NC,
	PTE8,
	PTE9,
	PTE10,
	PTE11
};

// output parameters
enum {
	LOW=0,
	HIGH,
};

enum {
	FALSE=0,
	TRUE
};

enum {
	STOP=0,
	LEFT,
	RIGHT
};

enum {
    NONE=0,
    GPIO,
    PWM
};

enum {
	MOTOR=0,
	LIGHTS
};

// error/success definitions
enum {
	SUCCESS=0,
	ERROR
};

// structure used for stepper control
struct StepperStruct {
	unsigned int Position;
	unsigned int Direction;
	unsigned int StepsToGo;
	unsigned int Mode;
};

// Simple pin config macro for the H-Bridge control pins
// Mode can be either NONE, GPIO or PWM
#define PinConfig(port, pin, mode) port->PCR[pin] = PORT_PCR_MUX(mode)

// This is the period of a PWM cycle (210Hz@12MHz FTM base clock)
#define MaxPwmPeriod 0xDF36

// DIP Switch [4:0]=[PTC[5:4],PTA[3:1]]
#define DIP1 PTD0
#define DIP2 PTD1
#define DIP3 PTD2
#define DIP4 PTD3
#define DIP5 PTC5
//#define DIP_Bits (1<<DIP5|1<<DIP4|1<<DIP3|1<<DIP2|1<<DIP1)

// E Inputs
#define E1 PTC3
#define E2 PTC2
#define E3 PTC1
#define E4 PTC16
#define E5 PTA0
#define E6 PTA1
#define E7 PTB1
#define E8 PTB0
#define Ex_Bits (1<<E1|1<<E2|1<<E3|1<<E4|1<<E5|1<<E6|1<<E7|1<<E8)

#define Eingang(x) (E_Input&(1<<(x-1)))
#define PosEdgeOnInput(x) (DigitalInputPosEdges&(1<<(x-1)))
#define NegEdgeOnInput(x) (DigitalInputNegEdges&(1<<(x-1)))
#define EdgeOnRawInput(x) (RawInputEdges&(1<<(x-1)))

#define POTI	PTA7
#define POTIHIGH (E_input_adc[8] >= 0x810)
#define POTILOW (E_input_adc[8] <= 0x7f0)

// Control outputs for H-Bridge
#define PORT_IN12 PTB
#define PORT_IN34 PTC

// H-Bridge 1
#define IN1 (PORTB->PCR[PTB2])
#define IN2 (PORTB->PCR[PTB3])
#define IN3 (PORTC->PCR[PTC14])
#define IN4 (PORTC->PCR[PTC15])

#define IN1_PIN	PTB2
#define IN2_PIN PTB3
#define IN3_PIN PTC14
#define IN4_PIN PTC15

// H-Bridge 2
#define IN5_PIN	PTA10
#define IN6_PIN PTA11
#define IN7_PIN PTA12
#define IN8_PIN PTA13

#define LED PTE9
#define LED_Bit (1<<LED)
#define LEDToggle	IP_PTE->PTOR = LED_Bit;
#define LEDOff		IP_PTE->PSOR = LED_Bit;
#define LEDOn 		IP_PTE->PCOR = LED_Bit;

#define FaultPin1 PTC9
#define FaultPin2 PTC8
#define FaultPin1_Bit	(1<<FaultPin1)
#define FaultPin2_Bit	(1<<FaultPin2)

#define EnableRTC 	IP_LPTMR0->CSR = LPTMR_CSR_TEN_MASK; 	// enable LPTMR0
#define DisableRTC	IP_LPTMR0->CSR = 0x00;					// disbale LPTMR0
#define RTCTimeout	(IP_LPTMR0->CSR & LPTMR_CSR_TCF_MASK)
#define ClearRTC	(IP_LPTMR0->CSR |= LPTMR_CSR_TCF_MASK);	// clear compare flag

#define PrepareTimeoutMS(x)	SetTimeout=x
#define DisableTimeoutMS	SetTimeout=0
#define Timeout (SetTimeout==0)
//#define TimeoutOccurred (SetTimeout==0)

// From iic.h
// DRV8847
#define HB_Address_ALL		0x60
#define HB_Address_HB1		0x65
#define HB_Address_HB2		0x6a
#define HB_SLAVE_ADDR_REG	0x00
#define HB_IC1_CON_REG		0x01
#define HB_IC2_CON_REG		0x02
#define HB_SLR_STATUS1_REG	0x03
#define HB_STATUS2_REG		0x04
#define HB_4PIN_MODE		0x00
#define HB_INDEP_MODE		0x03

// TCA6408
#define PT_Addr				0x20
#define PT_InputReg			0x00
#define PT_OutputReg		0x01
#define PT_PolarityReg		0x02
#define PT_ConfigReg		0x03

// PCF2127
#define RTC_Addr			0x51
#define RTC_SecReg			0x03

#endif /* FT_RPI_SA_CONTROLLER_FT_RPI_SA_CONTROLLER_H_ */
