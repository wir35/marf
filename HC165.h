#ifndef HC165_H_
#define HC165_H_
//Union which allows to control each switch on panel separately
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
	unsigned long long int value;
} uButtons;

void init_HC165(void);
void HC165_LatchUp(void);
unsigned char HC165_GetByte(void);
unsigned long int HC165_GetDWord(void);
unsigned long int HC165_GetDWord1(void);
unsigned long long int GetButton(void);
extern void delay_ms(unsigned int ms);
extern void delay_us(unsigned int us);


#endif /* HC165_H_ */
