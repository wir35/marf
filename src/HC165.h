#ifndef __HC165_H_
#define __HC165_H_

#include <stdint.h>

#include "delays.h"

//

#define HC165_GPIO_PIN_DAT  GPIO_Pin_0
#define HC165_GPIO_PIN_CP   GPIO_Pin_1
#define HC165_GPIO_PIN_CE   GPIO_Pin_2
#define HC165_GPIO_PIN_PL   GPIO_Pin_3

// CP
#define HC165_CLOCK_HIGH  GPIO_SetBits(GPIOC, GPIO_Pin_1)
#define HC165_CLOCK_LOW   GPIO_ResetBits(GPIOC, GPIO_Pin_1)
// PL
#define HC165_SS_HIGH   GPIO_SetBits(GPIOC, GPIO_Pin_3)
#define HC165_SS_LOW    GPIO_ResetBits(GPIOC, GPIO_Pin_3)
// CE
#define HC165_CE_HIGH   GPIO_SetBits(GPIOC, GPIO_Pin_2)
#define HC165_CE_LOW    GPIO_ResetBits(GPIOC, GPIO_Pin_2)


// Struct that defines the order of all switches as they come in on the shift registers
typedef union
{
	struct {
		unsigned char ClearUp:1;
		unsigned char ClearDown:1;
		unsigned char StepLeft:1;
		unsigned char StepRight:1;
		unsigned char OutputQuantize:1;
		unsigned char OutputContinuous:1;
		unsigned char IntegrationSloped:1;
		unsigned char IntegrationStepped:1;

		unsigned char FullRangeOn:1;
		unsigned char NU__FullRangeOff:1;
		unsigned char SourceExternal:1;
		unsigned char SourceInternal:1;		
		unsigned char Voltage0:1;
		unsigned char Voltage2:1;
		unsigned char Voltage4:1;
		unsigned char Voltage6:1;

		unsigned char Voltage8:1;
		unsigned char StopOn:1;
		unsigned char StopOff:1;
		unsigned char SustainOn:1;
		unsigned char SustainOff:1;		
		unsigned char EnableOn:1;
		unsigned char EnableOff:1;		
		unsigned char FirstOn:1;
		
		unsigned char StageAddress2External:1;
		unsigned char StageAddress2PulseSelect:1;
		unsigned char StageAddress2ContiniousSelect:1;
		unsigned char Empty2:1;
		unsigned char Pulse1On:1;
		unsigned char Pulse1Off:1;
		unsigned char Pulse2On:1;
		unsigned char Pulse2Off:1;
		
		unsigned char TimeSourceExternal:1;
		unsigned char TimeSourceInternal:1;
		unsigned char StageAddress2StrobeBanana:1;
		unsigned char Empty3:1;
		unsigned char Empty5:1;		
		unsigned char StageAddress2Reset:1;
		unsigned char StageAddress2Display:1;
		unsigned char StageAddress2Internal:1;

		unsigned char StageAddress1Advance:1;
		unsigned char StageAddress1Reset:1;
		unsigned char StageAddress1Display:1;
		unsigned char StageAddress2Advance:1;
		unsigned char StageAddress1PulseSelect:1;		
		unsigned char StageAddress1ContiniousSelect:1;
		unsigned char StageAddress1Internal:1;
		unsigned char StageAddress1External:1;
		
		unsigned char FirstOff:1;
		unsigned char LastOn:1;
		unsigned char LastOff:1;
		unsigned char TimeRange1:1;
		unsigned char TimeRange2:1;
		unsigned char TimeRange3:1;
		unsigned char TimeRange4:1;
		unsigned char StageAddress1StrobeSelect:1;

		unsigned char NU__Seq1StartBanana:1;
		unsigned char NU__Seq2StartBanana:1;
		unsigned char Empty4:1;
		unsigned char StageAddress1StrobeBanana:1;		
		unsigned char Empty1:1;
		unsigned char Empty6:1;		
		unsigned char NU_Seq2StopBanana:1;
		unsigned char StageAddress1StrobePulse:1;
	} b;
	uint64_t value;
} uButtons;


void HC165_InitializeGPIO(void);

uint64_t HC165_ReadSwitches(void);

#endif
