//*****************************************************************************
//
// Raise Your Arm 2013_ Micro Mouse robot.
//
// Control.h - Prototypes for the Control functions.
//
// This is part of revision 1.2 of the RYA Micro Mouse Library.
//      Happy coding.
//           Support Team RYA!
//*****************************************************************************

#ifndef CONTROL_H_
#define CONTROL_H_

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
void BoostEnable(void);
void BoostDisable(void);
void PrintDir(uint8_t Direction);
uint8_t TurnRight(int32_t DeltaEnc, int32_t SpeedLeft, int32_t SpeedRight, uint32_t NumPulse);
uint8_t TurnLeft(int32_t DeltaEnc, int32_t SpeedLeft, int32_t SpeedRight, uint32_t NumPulse);
uint8_t TurnBack(int32_t DeltaEnc, int32_t SpeedLeft, int32_t SpeedRight, uint32_t NumPulse);
void ClearPosition(void);
void UpdateBasePoint(int8_t x, int8_t y);
void Stop(void);

#endif /* CONTROL_H_ */

