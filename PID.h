//*****************************************************************************
//
// Raise Your Arm 2013_ Micro Mouse robot.
//
// MYUART.h - Prototypes for InterruptHandler functions.
//
// This is part of revision 1.2 of the RYA Micro Mouse Library.
//      Happy coding.
//           Support Team RYA!
//*****************************************************************************
#ifndef PID_H_
#define PID_H_

//*****************************************************************************
//
// Structure of all PID type: speed, position, wall...
//
//*****************************************************************************
typedef struct
{
		double Kp;
		double Ki;
		double Kd;
		double pPart;
		double iPart;
		double dPart;
		int32_t SetPoint;
		double PIDResult;
		double PIDResultTemp;
		int32_t PIDError;
		int32_t PIDErrorTemp1;
		int32_t PIDErrorTemp2;
		uint8_t Enable;
} PIDType;

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
void PIDSpeedSet(PIDType *p_PIDVer, int32_t SpeedSet);
void PIDPositionSet(PIDType *p_PID, int32_t SetPoint);
void PIDPosCalc(PIDType *p_PIDPos, int32_t Position, int32_t MaxResponse);
void PIDWallCalc(PIDType *p_PIDWall, int32_t Distance, int32_t MaxResponse);
void PIDVerCalc(PIDType *p_PIDVer, int32_t *Speed, int32_t MaxResponse);
void WallFollow(uint8_t AvailDirection);
void Move(int32_t PositionLeft, int32_t PositionRight);

#endif /* PID_H_ */
