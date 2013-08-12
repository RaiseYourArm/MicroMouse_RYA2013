//*****************************************************************************
//
// Raise Your Arm 2013_ Micro Mouse robot.
//
// InterruptHandler.c - All interrupt function of MCU.
//
// This is part of revision 1.2 of the RYA Micro Mouse Library.
//      Happy coding.
//           Support Team RYA!
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup InterruptHandler_api
//! @{
//
//*****************************************************************************
#include "include.h"

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern PIDType PIDVerLeft, PIDVerRight, PIDPosLeft, PIDPosRight, PIDWallRight, PIDWallLeft, PIDWallFL, PIDWallFR;
extern int16_t avrSpeed;
extern uint8_t PIDFlag;
extern uint8_t FollowSel;
int8_t Direction;
extern void (*v_USRBT1)(void), (*v_USRBT2)(void);
extern uint32_t PIDVerLoop;

int32_t ADCResOn[4], ADCResOff[4], ADCResDelta[4];
int32_t MaxIRBase[4];
int32_t ADCResult[4];
int32_t PosLeftCount = 0, PosRightCount = 0;
int32_t EncLeftCount, EncRightCount;
int8_t ControlFlag = 1;
int32_t BuzzerTick = 0;
int8_t x = 0, y = 0;
uint8_t AvailDirection, RequestTurn = 0;

uint8_t Stage = IDLE;
uint8_t ADCStatus = 0;
uint8_t ADCError = 0;

int8_t BasePoint[2] = {0, 0};

//*****************************************************************************
//
//! GPIO FALLING_EDGE Interrupt Service Routine  for the right encoder in the
//! pin PC5 (phase A). Phase B is in the pin PC6
//!
//!
//! \param None
//!
//! \return None
//
//*****************************************************************************
void EncRightISR(void)
{
	GPIOIntClear(GPIO_PORTC_BASE, GPIO_PIN_5);
		if (HWREG(GPIO_PORTC_BASE + 0x0100) & 0x40)
		{
			EncRightCount--;
			PosRightCount--;
		}
		else
		{
			EncRightCount++;
			PosRightCount++;
		}
}

//*****************************************************************************
//
//! GPIO FALLING_EDGE Interrupt Service Routine  for the left encoder in the
//! pin PD6 (phase A). Phase B is in the pin PD7.
//!
//!
//! \param None
//!
//! \return None
//
//*****************************************************************************
void EncLeftISR(void)
{
	GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_6);
		if (HWREG(GPIO_PORTD_BASE + 0x0200) & 0x80)
		{
			EncLeftCount--;
			PosLeftCount--;
		}
		else
		{
			EncLeftCount++;
			PosLeftCount++;
		}
}

//*****************************************************************************
//
//! GPIO FALLING_EDGE Interrupt Service Routine  for two button of main board.
//!
//! \param None
//!
//! \return None
//
//*****************************************************************************
void ButtonsISR(void)
{
	uint32_t ui32Status;
	ui32Status = GPIOIntStatus(GPIO_PORTA_BASE,true);
	GPIOIntClear(GPIO_PORTA_BASE, ui32Status);
	if (ui32Status & GPIO_PIN_2)
	{
		(*v_USRBT1)();
	}
	else if (ui32Status & GPIO_PIN_3)
	{
		(*v_USRBT2)();
	}
}

//*****************************************************************************
//
//! Timer5 Interrupt Service Routine : PID speed
//!                                    PID position
//! TimerIntervalms is the interrupt time of timer5
//!
//! \param None
//!
//! \return None
//
//*****************************************************************************
void Timer5ISR(void)
{
	static unsigned char NumSpdSet = 0;
	TimerIntClear(TIMER5_BASE, TIMER_TIMA_TIMEOUT);

	NumSpdSet++;

	if (NumSpdSet == PIDVerLoop)	//PID position
	{
		NumSpdSet = 0;
		if (PIDPosLeft.Enable)
		{
			PIDPosCalc(&PIDPosLeft, PosLeftCount, avrSpeed);
			PIDSpeedSet(&PIDVerLeft, (long)PIDPosLeft.PIDResult);
			EncLeftCount = 0;
		}
		if (PIDPosRight.Enable)
		{
			PIDPosCalc(&PIDPosRight, PosRightCount, avrSpeed);
			PIDSpeedSet(&PIDVerRight, (long)PIDPosRight.PIDResult);
			EncRightCount = 0;
		}

		if (PIDFlag)
		{
			PIDFlag = 0;
		}
	}

	if (PIDVerLeft.Enable)
	{
		PIDVerCalc(&PIDVerLeft, &EncLeftCount, 90);
		SetPWM(TIMER1_BASE, TIMER_A, DEFAULT, (long)PIDVerLeft.PIDResult);
	}
	if (PIDVerRight.Enable)
	{
		PIDVerCalc(&PIDVerRight, &EncRightCount, 90);
		SetPWM(TIMER0_BASE, TIMER_A, DEFAULT, (long)PIDVerRight.PIDResult);
	}
}

//*****************************************************************************
//
//! Timer4 Interrupt Service Routine : on/ off IRTransmitter, read ADC IRDetectors
//! 1ms is the interrupt time of timer4
//!
//! \param None
//!
//! \return None
//
//*****************************************************************************
void Timer4ISR(void)
{
	TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
	ADCStatus++;
	ADCStatus %= 20;

	if (BuzzerTick > 0)
	{
		BuzzerTick--;
		TimerMatchSet(TIMER1_BASE, TIMER_B, TimerLoadGet(TIMER1_BASE, TIMER_B) / 2);
	}
	else
	{
		TimerMatchSet(TIMER1_BASE, TIMER_B, 0);
	}

	switch (ADCStatus % 10)
	{
		case 0:
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0x10);
			break;
		case 1:
			ADCProcessorTrigger(ADC0_BASE, 2);
			break;
		case 2:
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0x20);
			break;
		case 3:
			ADCProcessorTrigger(ADC0_BASE, 2);
			break;
		case 4:
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0x40);
			break;
		case 5:
			ADCProcessorTrigger(ADC0_BASE, 2);
			break;
		case 6:
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0x80);
			break;
		case 7:
			ADCProcessorTrigger(ADC0_BASE, 2);
			break;
		case 8:
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0x00);
			break;
		case 9:
			ADCProcessorTrigger(ADC0_BASE, 2);
			break;
	}
}

//*****************************************************************************
//
//! ADC Interrupt Service Routine: get ADC value, calculate the distance from
//! robot to the wall. Get available direction and run PID wall follow.
//!
//! \param None
//!
//! \return None
//
//*****************************************************************************
void ADCIsr(void)
{
	uint8_t temp;
	static uint8_t DirectionTemp;
	ADCIntClear(ADC0_BASE, 2);
	ADCSequenceDataGet(ADC0_BASE, 2, &ADCResult[0]);
	switch (ADCStatus)
	{
		case 1:
			ADCResOn[0] = ADCResult[0];
			break;
		case 3:
			ADCResOn[1] = ADCResult[1];
			break;
		case 5:
			ADCResOn[2] = ADCResult[2];
			break;
		case 7:
			ADCResOn[3] = ADCResult[3];
			break;
		case 9:
			for (temp = 0; temp < 4; temp++)
			{
				ADCResDelta[temp] = ADCResOn[temp] - ADCResult[temp];
			}
			AvailDirection = GetAvailDir();
			if (AvailDirection != DirectionTemp)
			{
				RequestTurn = 1;
			}
			DirectionTemp = AvailDirection;
			if (FollowSel != FOLLOW_DISABLE)
				WallFollow(AvailDirection);
			break;
		default:
			ADCError = 1;
			break;
	}
}

//*****************************************************************************
//
//! Base on ADC value, calculate the available direction, which direction have
//! no wall.
//!
//! \param None
//!
//! \return: 1 byte, each bit describe one available direction.
//!    AVAIL_LEFT		0x01
//!    AVAIL_FL		    0x02
//!    AVAIL_FR	    	0x04
//!    AVAIL_RIGHT		0x08
//!    NOT_AVAIL		0xFE
//
//*****************************************************************************
uint8_t GetAvailDir(void)
{
	uint8_t AvailDir = 0;
	if (ADCResDelta[0] > PIDWallFL.SetPoint)
	{
		AvailDir |= AVAIL_FL;
	}
	if (ADCResDelta[1] > MaxIRBase[1])
	{
		AvailDir |= AVAIL_LEFT;
	}
	if (ADCResDelta[2] > MaxIRBase[2])
	{
		AvailDir |= AVAIL_RIGHT;
	}
	if (ADCResDelta[3] > PIDWallFR.SetPoint)
	{
		AvailDir |= AVAIL_FR;
	}
	return (AvailDir);
}

//*****************************************************************************
//
//! Base on ADC value of BATTERY VOLTAGE SENSING module, disable power module,
//! disable all interrupt, make the buzzer scream out "beep beep beep..."
//!
//! \param None.
//!
//! \return: None.
//
//*****************************************************************************
void BattSenseISR(void)
{
	uint8_t temp;
	uint32_t BattResult;
	static uint32_t avrBattResult = 2700;
	ADCIntClear(ADC1_BASE, 3);
	ADCSequenceDataGet(ADC1_BASE, 3, &BattResult);

	avrBattResult = (4 * avrBattResult + BattResult) / 5;
	if (avrBattResult < 2630)
	{
		ControlFlag = 1;
		GPIOPinWrite(ENABLE_PORT, ENA_LEFT_PIN | ENA_RIGHT_PIN, 0x00);
		SetPWM(TIMER1_BASE, TIMER_A, DEFAULT, 0);
		SetPWM(TIMER0_BASE, TIMER_A, DEFAULT, 0);
		for (temp = 0; temp < 15; temp ++)
		{
			TimerMatchSet(TIMER1_BASE, TIMER_B, TimerLoadGet(TIMER1_BASE, TIMER_B) / 2);
			SysCtlDelay(SysCtlClockGet() / 100);
			TimerMatchSet(TIMER1_BASE, TIMER_B, 0);
			SysCtlDelay(SysCtlClockGet() / 80);
		}
		IntMasterDisable();
		BoostDisable();
	}
}

//*****************************************************************************
//
//! Create trigger pulse for ADC BATTERY VOLTAGE SENSING module
//!
//! \param None.
//!
//! \return: None.
//
//*****************************************************************************
void TimerTickBatt(void)
{
	TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
	ADCProcessorTrigger(ADC1_BASE, 3);
}

//*****************************************************************************
//
//! Calculate the position for the robot base on the total pulse of two encoder
//! If the distance which robot moved is more than the length of one cell,
//! the co-ordinate will be update.
//!
//! \param Direction: is the direction of robot
//! \param Delta: is number off cell which robot moved .
//!                                        (total pulse/the length of one cell)
//!
//!
//! \return 0 (zero).
//
//*****************************************************************************
uint8_t CalcPosition(uint8_t Direction, int8_t Delta)
{
	static int8_t DeltaTemp = 0, DirTemp = 0, x_temp, y_temp;
	if ((DeltaTemp != Delta) || (DirTemp != Direction) || (x_temp != x) || (y_temp |= y))
	{
		DeltaTemp = Delta;
		DirTemp = Direction;
		x_temp = x;
		y_temp = y;
		switch (Direction)
		{
			case 0:
				y = BasePoint[1] + Delta;
				break;
			case 1:
				x = BasePoint[0] + Delta;
				break;
			case 2:
				y = BasePoint[1] - Delta;
				break;
			case 3:
				x = BasePoint[0] - Delta;
				break;
		}
	}
	return (0);
}

