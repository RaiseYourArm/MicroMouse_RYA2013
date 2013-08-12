//*****************************************************************************
//
// Raise Your Arm 2013_ Micro Mouse robot.
//
// PID.c - PID calculator
//
// This is part of revision 1.2 of the RYA Micro Mouse Library.
//      Happy coding.
//           Support Team RYA!
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup PID_api
//! @{
//
//*****************************************************************************
#include "include.h"

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern int32_t ADCResOn[], ADCResOff[], ADCResDelta[];
extern int32_t PosLeftCount, PosRightCount;
extern int32_t EncLeftCount, EncRightCount;
PIDType PIDVerLeft, PIDVerRight, PIDPosLeft, PIDPosRight, PIDWallRight, PIDWallLeft, PIDWallFL, PIDWallFR;
extern uint8_t FollowSel;

int16_t avrSpeed = 80;	//115

//*****************************************************************************
//
//! Write speed set point to PID structure.
//!
//!
//! \param *p_PIDVer is the pointer of PID structure (velocity left or right).
//! \param SpeedSet is the speed set point.
//!
//! \return None.
//
//*****************************************************************************
void PIDSpeedSet(PIDType *p_PIDVer, int32_t SpeedSet)
{
	(*p_PIDVer).Enable = 1;
	(*p_PIDVer).SetPoint = SpeedSet;
	(*p_PIDVer).iPart = 0;
}

//*****************************************************************************
//
//! Write Position set point to PID structure
//!
//!
//! \param *p_PID is the pointer of PID structure (position left or right).
//! \param SetPoint is the position set point.
//!
//! \return None.
//
//*****************************************************************************
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

//*****************************************************************************
//
//! PID Position Calculate control signal (.result) base on set point and real value
//! with Kp Ki Kd constant.
//!
//! \param *p_PIDPos is the pointer of PID structure (position left or right).
//! \param Position is the real position.
//! \param MaxResponse is the maximum result of PID.
//!
//! \return None.
//
//*****************************************************************************
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

//*****************************************************************************
//
//! PID Distance (to the wall) Calculate control signal (.result) base on set
//! point and real value with Kp Ki Kd constant.
//!
//! \param *p_PIDWall is the pointer of PID structure (wall left or right).
//! \param Distance is the real distance measurement by IR.
//! \param MaxResponse is the maximum result of PID.
//!
//! \return None.
//
//*****************************************************************************
void PIDWallCalc(PIDType *p_PIDWall, int32_t Distance, int32_t MaxResponse)
{
	(*p_PIDWall).PIDError = (*p_PIDWall).SetPoint - Distance;
	(*p_PIDWall).pPart = (*p_PIDWall).Kp * (*p_PIDWall).PIDError;
	(*p_PIDWall).iPart += (*p_PIDWall).Ki * (*p_PIDWall).PIDError;
	(*p_PIDWall).dPart = (*p_PIDWall).Kd * ((*p_PIDWall).PIDError - (*p_PIDWall).PIDErrorTemp1);
	/*
	//Uncomment to enable iPart-limit
	if ((*p_PIDWall).iPart > 40)
		(*p_PIDWall).iPart = 40;
	else if ((*p_PIDWall).iPart < -40)
		(*p_PIDWall).iPart = -40;
	*/
	(*p_PIDWall).PIDResult = (*p_PIDWall).pPart + (*p_PIDWall).iPart + (*p_PIDWall).dPart;
	if ((*p_PIDWall).PIDResult > MaxResponse)
		(*p_PIDWall).PIDResult = (double)(MaxResponse);
	if ((*p_PIDWall).PIDResult < -1 * MaxResponse)
		(*p_PIDWall).PIDResult = (double)(-1 * MaxResponse);
}

//*****************************************************************************
//
//! PID velocity (of motor) Calculate control signal (.result) base on set
//! point and real value with Kp Ki Kd constant.
//!
//! \param *p_PIDVer is the pointer of PID structure (velocity left or right).
//! \param *Speed is the real speed of robot measurement by encoder
//! (EncRightCount or EncLeftCount)
//! \param MaxResponse is the maximum Duty Cycle.
//!
//! \return None.
//
//*****************************************************************************
void PIDVerCalc(PIDType *p_PIDVer, int32_t *Speed, int32_t MaxResponse)
{
	(*p_PIDVer).PIDError = (*p_PIDVer).SetPoint - (*Speed);
	*Speed = 0;

	(*p_PIDVer).pPart = (*p_PIDVer).Kp * (*p_PIDVer).PIDError;
	(*p_PIDVer).iPart += (*p_PIDVer).Ki * (*p_PIDVer).PIDError;
	(*p_PIDVer).dPart = (*p_PIDVer).Kd * ((*p_PIDVer).PIDError - (*p_PIDVer).PIDErrorTemp1);

	(*p_PIDVer).PIDResult += ((*p_PIDVer).pPart + (*p_PIDVer).iPart + (*p_PIDVer).dPart) ;

	if ((*p_PIDVer).PIDResult > MaxResponse)
		(*p_PIDVer).PIDResult = (double)MaxResponse;
	if ((*p_PIDVer).PIDResult < -1 * MaxResponse)
		(*p_PIDVer).PIDResult = (double)(-1 * (MaxResponse));
	(*p_PIDVer).PIDErrorTemp1 = (*p_PIDVer).PIDError;
}

//*****************************************************************************
//
//! Make robot follow the wall: left, right or auto by detecting available
//! direction.
//!
//! \param AvailDirection is the direction have no wall.
//! \return None.
//
//*****************************************************************************
void WallFollow(uint8_t AvailDirection)
{
	int32_t DistanceIR[2];
	DistanceIR[0] = ADCResDelta[1];
	DistanceIR[1] = ADCResDelta[2];

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
			PIDWallCalc(&PIDWallLeft, DistanceIR[0], 100);
			PIDSpeedSet(&PIDVerLeft, (int32_t)(avrSpeed) + (int32_t)(PIDWallLeft.PIDResult / 2));
			PIDSpeedSet(&PIDVerRight, (int32_t)(avrSpeed) - (int32_t)(PIDWallLeft.PIDResult / 2));
			break;
		case FOLLOW_RIGHT:
			PIDWallCalc(&PIDWallRight, DistanceIR[1], 100);
			PIDSpeedSet(&PIDVerLeft, (int32_t)(avrSpeed) - (int32_t)(PIDWallRight.PIDResult / 2));
			PIDSpeedSet(&PIDVerRight, (int32_t)(avrSpeed) + (int32_t)(PIDWallRight.PIDResult / 2));
			break;
	}
}

//*****************************************************************************
//
//! Make robot move a little bit by set some pulses to two encoder.
//!
//! \param PositionLeft is pulse of left encoder.
//! \param PositionRight is pulse of right encoder.
//! \return None.
//
//*****************************************************************************
void Move(int32_t PositionLeft, int32_t PositionRight)
{
	PIDPositionSet(&PIDPosLeft, PositionLeft);
	PIDPositionSet(&PIDPosRight, PositionRight);
}
