//*****************************************************************************
//
// Raise Your Arm 2013_ Micro Mouse robot.
//
// control.c - Control the robot's action.
//
// This is part of revision 1.2 of the RYA Micro Mouse Library.
//      Happy coding.
//           Support Team RYA!
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup Control_api
//! @{
//
//*****************************************************************************

#include "include.h"

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************

extern int8_t x, y;
extern uint8_t ControlFlag;
extern int32_t BuzzerTick;
extern int32_t DeltaEnc;
extern uint8_t AvailDirection, RequestTurn;
extern PIDType PIDVerLeft, PIDVerRight, PIDPosLeft, PIDPosRight, PIDWallRight, PIDWallLeft, PIDWallFL, PIDWallFR;
extern int32_t PosLeftCount, PosRightCount;
extern int16_t avrSpeed;
extern int8_t BasePoint[];
extern int8_t Direction;
extern int32_t ADCResOn[], ADCResOff[], ADCResDelta[];

uint8_t PIDFlag = 0;
uint8_t FollowSel = FOLLOW_DISABLE;

//*****************************************************************************
//
//! Enable Boost board in the pin PB2
//!
//!
//! \param None
//!
//! \return None
//
//*****************************************************************************

void BoostEnable(void)
{
	GPIOPinWrite(ENABLE_PORT, BOOST_EN_PIN, BOOST_EN_PIN);
}

//*****************************************************************************
//
//! Disables Boost board in the pin PB2
//!
//! \param None
//!
//! \return None.
//
//*****************************************************************************
void BoostDisable(void)
{
	GPIOPinWrite(ENABLE_PORT, BOOST_EN_PIN, 0);
}

//*****************************************************************************
//
//! Print the direction of the Micro Mouse by Uart0
//!
//! \param Direction is the direction of robot 0 1 2 or 3
//!
//! \return None.
//
//*****************************************************************************
void PrintDir(uint8_t Direction)
{
	switch(Direction)
	{
		case 0:
			UARTPuts(UART0_BASE, "UP");
			UARTCharPut(UART0_BASE, '\n');
			UARTCharPut(UART0_BASE, '\r');
			break;
		case 1:
			UARTPuts(UART0_BASE, "RIGHT");
			UARTCharPut(UART0_BASE, '\n');
			UARTCharPut(UART0_BASE, '\r');
			break;
		case 2:
			UARTPuts(UART0_BASE, "DOWN");
			UARTCharPut(UART0_BASE, '\n');
			UARTCharPut(UART0_BASE, '\r');
			break;
		case 3:
			UARTPuts(UART0_BASE, "LEFT");
			UARTCharPut(UART0_BASE, '\n');
			UARTCharPut(UART0_BASE, '\r');
			break;
	}

}

//*****************************************************************************
//
//! Control two motor to make robot turn right 90 degree
//!
//! \param DeltaEnc is the distance robot will go straight before turn right
//!, the robot will stand between the next cell of maze.
//! \param SpeedLeft is the speed of left motor.
//! \param SpeedRight is the speed of left motor.
//! \param NumPulse is the total pulse of two encoder after turn
//!
//!
//! \return 0 (zero).
//
//*****************************************************************************
uint8_t TurnRight(int32_t DeltaEnc, int32_t SpeedLeft, int32_t SpeedRight, uint32_t NumPulse)
{
	uint32_t EncTemp, avrSpeedtemp;

	BuzzerTick = 100;

	avrSpeedtemp = avrSpeed;
	EncTemp = PosLeftCount;
	while(abs(PosLeftCount - EncTemp) < DeltaEnc)
	{
		avrSpeed = ((abs(DeltaEnc + EncTemp - PosLeftCount) / (DeltaEnc / avrSpeedtemp)) / 2) + (abs(SpeedLeft) + abs(SpeedRight)) / 2;
	}
	avrSpeed = avrSpeedtemp;
	CalcPosition(Direction, (int8_t)((PosLeftCount + PosRightCount + 5500) / 6800));
	UpdateBasePoint(x, y);

	BuzzerTick = 100;

	FollowSel = FOLLOW_DISABLE;
	ClearPosition();
	PIDSpeedSet(&PIDVerLeft, SpeedLeft);
	PIDSpeedSet(&PIDVerRight, SpeedRight);

	while ((abs(PosLeftCount) + abs(PosRightCount) < NumPulse)||(!(AvailDirection & (AVAIL_FR | AVAIL_FL))));

	BuzzerTick = 100;

	ClearPosition();

	PIDWallLeft.iPart = 0;
	PIDWallLeft.PIDResult = 0;
	PIDWallRight.iPart = 0;
	PIDWallRight.PIDResult = 0;
	FollowSel = FOLLOW_AUTO_SELECT;
	return (0);
}

//*****************************************************************************
//
//! Control two motor to make robot turn left 90 degree
//!
//! \param DeltaEnc is the distance robot will go straight before turn right
//!, the robot will stand between the next cell of maze.
//! \param SpeedLeft is the speed of left motor.
//! \param SpeedRight is the speed of left motor.
//! \param NumPulse is the total pulse of two encoder after turn
//!
//!
//! \return 0 (zero).
//
//*****************************************************************************

uint8_t TurnLeft(int32_t DeltaEnc, int32_t SpeedLeft, int32_t SpeedRight, uint32_t NumPulse)
{
	uint32_t EncTemp, avrSpeedtemp;

	BuzzerTick = 100;

	avrSpeedtemp = avrSpeed;
	EncTemp = PosLeftCount;
	while(abs(PosLeftCount - EncTemp) < DeltaEnc)
	{
		avrSpeed = ((abs(DeltaEnc + EncTemp - PosLeftCount) / (DeltaEnc / avrSpeedtemp)) / 2) + (abs(SpeedLeft) + abs(SpeedRight)) / 2;
	}
	avrSpeed = avrSpeedtemp;
	CalcPosition(Direction, (int8_t)((PosLeftCount + PosRightCount + 5500) / 6800));
	UpdateBasePoint(x, y);

	BuzzerTick = 100;

	FollowSel = FOLLOW_DISABLE;
	ClearPosition();
	PIDSpeedSet(&PIDVerLeft, SpeedLeft);
	PIDSpeedSet(&PIDVerRight, SpeedRight);

	while ((abs(PosLeftCount) + abs(PosRightCount) < NumPulse)||(!(AvailDirection & (AVAIL_FR | AVAIL_FL))));

	BuzzerTick = 100;

	ClearPosition();

	PIDWallLeft.iPart = 0;
	PIDWallLeft.PIDResult = 0;
	PIDWallRight.iPart = 0;
	PIDWallRight.PIDResult = 0;
	FollowSel = FOLLOW_AUTO_SELECT;
	return (0);
}

//*****************************************************************************
//
//! Control two motor to make robot turn back 180 degree.
//!
//! \param DeltaEnc is the distance robot will go straight before turn right
//!, the robot will stand between the next cell of maze.
//! \param SpeedLeft is the speed of left motor.
//! \param SpeedRight is the speed of left motor.
//! \param NumPulse is the total pulse of two encoder after turn
//!
//!
//! \return 0 (zero).
//
//*****************************************************************************

uint8_t TurnBack(int32_t DeltaEnc, int32_t SpeedLeft, int32_t SpeedRight, uint32_t NumPulse)
{
	uint32_t EncTemp, avrSpeedtemp;
	BuzzerTick = 100;

	avrSpeedtemp = avrSpeed;
	EncTemp = PosLeftCount;
	while(abs(PosLeftCount - EncTemp) < DeltaEnc)
	{
		avrSpeed = ((abs(DeltaEnc + EncTemp - PosLeftCount) / (DeltaEnc / avrSpeedtemp)) / 2) + (abs(SpeedLeft) + abs(SpeedRight)) / 2;
	}
	avrSpeed = avrSpeedtemp;

	PIDPosLeft.Enable = 0;
	PIDPosRight.Enable = 0;
	PIDPosLeft.PIDResult = 0;
	PIDPosRight.PIDResult = 0;

	CalcPosition(Direction, (int8_t)((PosLeftCount + PosRightCount + 5500) / 6800));
	UpdateBasePoint(x, y);

	BuzzerTick = 100;

	FollowSel = FOLLOW_DISABLE;

	ClearPosition();
	PIDSpeedSet(&PIDVerLeft, SpeedLeft);
	PIDSpeedSet(&PIDVerRight, SpeedRight);

	while (((GetAvailDir() & (AVAIL_FR | AVAIL_FL)) != (AVAIL_FR | AVAIL_FL)) || (((abs(PosLeftCount) + abs(PosRightCount)) < NumPulse)));

	BuzzerTick = 100;

	ClearPosition();

	RequestTurn = 0;
	PIDWallLeft.iPart = 0;
	PIDWallLeft.PIDResult = 0;
	PIDWallRight.iPart = 0;
	PIDWallRight.PIDResult = 0;
	FollowSel = FOLLOW_AUTO_SELECT;
	return (0);
}

//*****************************************************************************
//
//! Clear the  total pulse counter of two encoder
//!
//! \param None
//!
//!
//! \return None.
//
//*****************************************************************************
void ClearPosition(void)
{
	PosLeftCount = 0;
	PosRightCount = 0;
}

//*****************************************************************************
//
//! Update the base point of maze, usually update after robot turn left, right
//!
//! \param x the horizontal coordinates.
//! \param y the vertical coordinates.
//!
//!
//! \return None.
//
//*****************************************************************************

void UpdateBasePoint(int8_t x, int8_t y)
{
	BasePoint[0] = x;
	BasePoint[1] = y;
}

//*****************************************************************************
//
//! Stop the robot by disable 2 H bridge.
//!
//! \param x the horizontal coordinates.
//! \param y the vertical coordinates.
//!
//!
//! \return None.
//
//*****************************************************************************

void Stop(void)
{
	BuzzerTick = 2000;
	ControlFlag = 1;
	GPIOPinWrite(ENABLE_PORT, ENA_LEFT_PIN | ENA_RIGHT_PIN, 0x00);
}

