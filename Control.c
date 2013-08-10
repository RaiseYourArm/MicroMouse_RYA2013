#include "include.h"

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
extern uint8_t Maze[33][33];
extern int32_t ADCResOn[], ADCResOff[], ADCResDelta[];

uint8_t Array[3];

uint8_t PIDFlag = 0;
uint8_t FollowSel = FOLLOW_DISABLE;

void BoostEnable(void)
{
	GPIOPinWrite(ENABLE_PORT, BOOST_EN_PIN, BOOST_EN_PIN);
}

void BoostDisable(void)
{
	GPIOPinWrite(ENABLE_PORT, BOOST_EN_PIN, 0);
}


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
	FollowSel = FOLLOW_LEFT;
	return (0);
}

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
	FollowSel = FOLLOW_LEFT;
	return (0);
}

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
	FollowSel = FOLLOW_LEFT;
	return (0);
}

void ClearPosition(void)
{
	PosLeftCount = 0;
	PosRightCount = 0;
}

void UpdateBasePoint(int8_t x, int8_t y)
{
	BasePoint[0] = x;
	BasePoint[1] = y;
}

void Stop(void)
{
	BuzzerTick = 2000;
	ControlFlag = 1;
	GPIOPinWrite(ENABLE_PORT, ENA_LEFT_PIN | ENA_RIGHT_PIN, 0x00);
}
