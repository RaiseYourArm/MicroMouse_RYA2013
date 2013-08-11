#include "include.h"
#include "driverlib/eeprom.h"

extern PIDType PIDVerLeft, PIDVerRight, PIDPosLeft, PIDPosRight, PIDWallRight, PIDWallLeft, PIDWallFL, PIDWallFR;
extern uint8_t ControlFlag;
extern uint8_t Stage;
extern int32_t ADCResOn[], ADCResOff[], ADCResDelta[];
extern int32_t MaxIRBase[];
extern int32_t BuzzerTick;
extern uint8_t FollowSel;
extern uint8_t AvailDirection, RequestTurn;

uint32_t Readeeprom[4];

void Button1ISR(void)
{
	//Do nothing
	//Add your code here
}

void Button2ISR(void)
{
	//Do nothing
	//Add your code here
}

void InitPID(void)
{
	//Replace x.x with correct value
	PIDVerLeft.Kp = x.x;
	PIDVerLeft.Ki = x.x;
	PIDVerLeft.Kd = x.x;
	PIDVerLeft.Enable = x.x;

	PIDVerRight.Kp = x.x;
	PIDVerRight.Ki = x.x;
	PIDVerRight.Kd = x.x;
	PIDVerRight.Enable = x.x;

	PIDPosLeft.Kp = x.x;
	PIDPosLeft.Ki = x.x;
	PIDPosLeft.Kd = x.x;
	PIDPosLeft.Enable = x.x;

	PIDPosRight.Kp = x.x;
	PIDPosRight.Ki = x.x;
	PIDPosRight.Kd = x.x;
	PIDPosRight.Enable = x.x;

	PIDWallRight.Kp = x.x;
	PIDWallRight.Ki = x.x;
	PIDWallRight.Kd = x.x;

	PIDWallLeft.Kp = x.x;
	PIDWallLeft.Ki = x.x;
	PIDWallLeft.Kd = x.x;
}

void main (void)
{
	ConfigSystem();
	ConfigADCTimer();		//Set ADC timer interval: 20ms
	ConfigPIDTimer(10,2);	//10 ms speed, 10*2 = 20ms position
	EnableBuzzer();			//Enable Buzzer before CongifPWM
	ConfigPWM();
	ConfigIRDetectors();	
	ConfigEncoder();
	ConfigIRTransmitter();
	ConfigBattSense();		//Enable battery sensing
	ConfigButtons(&Button1ISR, &Button2ISR);	//Config User buttons
	SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);	//Enable EEPROM
	EEPROMInit();
	SetPWM(TIMER1_BASE, TIMER_A, DEFAULT, 0);
	SetPWM(TIMER0_BASE, TIMER_A, DEFAULT, 0);
	IntMasterEnable();
	InitPID();

	EEPROMWrite((uint32_t *)ADCResDelta, 0x7f0,4);	//write data to eeprom
	EEPROMReadWords((uint32_t *)Readeeprom, 0x7f0, 4);	//Read data from eeprom
	
	//Select one:
	FollowSel = FOLLOW_LEFT;	//Enable follow left wall
	//PIDSpeedSet(&PIDVerLeft, 50);		//Set left-motor speed at 50
	//PIDPositionSet(&PIDPosRight, 2000);	//
	//Move (3000, 5000);				//Move forward: left-motor 3000 pulses - right-motor 5000 pulses
	//FollowSel = FOLLOW_DISABLE;	//Disable wall-follow
	
	while(1)
	{
		//Write your code here
	}
}



