#include "include.h"

extern int32_t ADCResOn[], ADCResOff[], ADCResDelta[];
extern int32_t PosLeftCount, PosRightCount;
extern int32_t EncLeftCount, EncRightCount;
PIDType PIDVerLeft, PIDVerRight, PIDPosLeft, PIDPosRight, PIDWallRight, PIDWallLeft, PIDWallFL, PIDWallFR;
extern uint8_t FollowSel;

int16_t avrSpeed = 80;	//115

void PIDSpeedSet(PIDType *p_PIDVer, int32_t SpeedSet)
{
	(*p_PIDVer).Enable = 1;
	(*p_PIDVer).SetPoint = SpeedSet;
	(*p_PIDVer).iPart = 0;
}

void PIDPositionSet(PIDType *p_PID, int32_t SetPoint)
{
	(*p_PID).Enable = 1;
	(*p_PID).iPart = 0;
	(*p_PID).PIDErrorTemp1 = 0;
	(*p_PID).SetPoint = SetPoint;
	if (p_PID == &PIDPosLeft)
	{
		PosLeftCount = 0;
	}
	else
	{
		PosRightCount = 0;
	}
}

void PIDPosCalc(PIDType *p_PIDPos, int32_t Position, int32_t MaxResponse)
{
	(*p_PIDPos).PIDError = (*p_PIDPos).SetPoint - Position;
	(*p_PIDPos).pPart = (*p_PIDPos).Kp * (*p_PIDPos).PIDError;
	(*p_PIDPos).iPart += (*p_PIDPos).Ki * (*p_PIDPos).PIDError;
	(*p_PIDPos).dPart = (*p_PIDPos).Kd * ((*p_PIDPos).PIDError - (*p_PIDPos).PIDErrorTemp1);
	/*
	//Uncomment to enable iPart-limit
	if ((*p_PIDPos).iPart > 40)
		(*p_PIDPos).iPart = 40;
	else if ((*p_PIDPos).iPart < -40)
		(*p_PIDPos).iPart = -40;
	*/
	(*p_PIDPos).PIDResult = (*p_PIDPos).pPart + (*p_PIDPos).iPart + (*p_PIDPos).dPart;
	if ((*p_PIDPos).PIDResult > MaxResponse)
		(*p_PIDPos).PIDResult = (double)(MaxResponse);
	if ((*p_PIDPos).PIDResult < -1 * MaxResponse)
		(*p_PIDPos).PIDResult = (double)(-1 * MaxResponse);
}

void PIDWallCalc(PIDType *p_PIDWall, int32_t Distance, int32_t MaxResponse)
{
	(*p_PIDWall).PIDError = (*p_PIDWall).SetPoint - Distance;
	(*p_PIDWall).pPart = (*p_PIDWall).Kp * (*p_PIDWall).PIDError;
	(*p_PIDWall).iPart += (*p_PIDWall).Ki * (*p_PIDWall).PIDError;
	(*p_PIDWall).dPart = (*p_PIDWall).Kd * ((*p_PIDWall).PIDError - (*p_PIDWall).PIDErrorTemp1);

	//Uncomment to enable iPart-limit
	if ((*p_PIDWall).iPart > 40)
		(*p_PIDWall).iPart = 40;
	else if ((*p_PIDWall).iPart < -40)
		(*p_PIDWall).iPart = -40;

	(*p_PIDWall).PIDResult = (*p_PIDWall).pPart + (*p_PIDWall).iPart + (*p_PIDWall).dPart;
	if ((*p_PIDWall).PIDResult > MaxResponse)
		(*p_PIDWall).PIDResult = (double)(MaxResponse);
	if ((*p_PIDWall).PIDResult < -1 * MaxResponse)
		(*p_PIDWall).PIDResult = (double)(-1 * MaxResponse);
}

//MaxResponse: Max Duty Cycle
void PIDVerCalc(PIDType *p_PIDVer, int32_t *Speed, int32_t MaxResponse)
{
	(*p_PIDVer).PIDError = (*p_PIDVer).SetPoint - (*Speed);
	*Speed = 0;

	(*p_PIDVer).pPart = (*p_PIDVer).Kp * (*p_PIDVer).PIDError;
	(*p_PIDVer).iPart += (*p_PIDVer).Ki * (*p_PIDVer).PIDError * 0.01;
	(*p_PIDVer).dPart = (*p_PIDVer).Kd * ((*p_PIDVer).PIDError - (*p_PIDVer).PIDErrorTemp1)/0.01;

	(*p_PIDVer).PIDResult += ((*p_PIDVer).pPart + (*p_PIDVer).iPart + (*p_PIDVer).dPart) ;

	if ((*p_PIDVer).PIDResult > MaxResponse)
		(*p_PIDVer).PIDResult = (double)MaxResponse;
	if ((*p_PIDVer).PIDResult < -1 * MaxResponse)
		(*p_PIDVer).PIDResult = (double)(-1 * (MaxResponse));
	(*p_PIDVer).PIDErrorTemp1 = (*p_PIDVer).PIDError;
}

void WallFollow(uint8_t AvailDirection)
{
	int32_t DistanceIR[2];
	DistanceIR[0] = ADCResDelta[1] / 5;
	DistanceIR[1] = ADCResDelta[2] / 5;

	switch(FollowSel)
	{
		case FOLLOW_AUTO_SELECT:
			if (!(AvailDirection & AVAIL_RIGHT))
			{
				PIDWallCalc(&PIDWallRight, DistanceIR[1], 100);

				PIDSpeedSet(&PIDVerLeft, (int32_t)(avrSpeed) - (int32_t)(PIDWallRight.PIDResult / 2));
				PIDSpeedSet(&PIDVerRight, (int32_t)(avrSpeed) + (int32_t)(PIDWallRight.PIDResult / 2));
			}
			else if (!(AvailDirection & AVAIL_LEFT))
			{
				PIDWallCalc(&PIDWallLeft, DistanceIR[0], 100);

				PIDSpeedSet(&PIDVerLeft, (int32_t)(avrSpeed) + (int32_t)(PIDWallLeft.PIDResult / 2));
				PIDSpeedSet(&PIDVerRight, (int32_t)(avrSpeed) - (int32_t)(PIDWallLeft.PIDResult / 2));
			}
			else
			{
				PIDSpeedSet(&PIDVerLeft, avrSpeed / 2);
				PIDSpeedSet(&PIDVerRight, avrSpeed / 2);
			}
			break;
		case FOLLOW_LEFT:
			PIDPosCalc(&PIDWallLeft, DistanceIR[0], 100);
			PIDSpeedSet(&PIDVerLeft, (int32_t)(avrSpeed) + (int32_t)(PIDWallLeft.PIDResult / 2));
			PIDSpeedSet(&PIDVerRight, (int32_t)(avrSpeed) - (int32_t)(PIDWallLeft.PIDResult / 2));
			break;
		case FOLLOW_RIGHT:
			PIDPosCalc(&PIDWallRight, DistanceIR[1], 100);
			PIDSpeedSet(&PIDVerLeft, (int32_t)(avrSpeed) - (int32_t)(PIDWallRight.PIDResult / 2));
			PIDSpeedSet(&PIDVerRight, (int32_t)(avrSpeed) + (int32_t)(PIDWallRight.PIDResult / 2));
			break;
	}
}

void Move(int32_t PositionLeft, int32_t PositionRight)
{
	PIDPositionSet(&PIDPosLeft, PositionLeft);
	PIDPositionSet(&PIDPosRight, PositionRight);
}
