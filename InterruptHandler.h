//*****************************************************************************
//
// Raise Your Arm 2013_ Micro Mouse robot.
//
// InterruptHandler.h - Prototypes for InterruptHandler functions.
//
// This is part of revision 1.2 of the RYA Micro Mouse Library.
//      Happy coding.
//           Support Team RYA!
//*****************************************************************************
#ifndef INTERRUPTHANDLER_H_
#define INTERRUPTHANDLER_H_

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
void EncRightISR(void);
void EncLeftISR(void);
void ButtonsISR(void);
void Timer5ISR(void);
void Timer4ISR(void);
void ADCIsr(void);
uint8_t GetAvailDir(void);
void BattSenseISR(void);
void TimerTickBatt(void);
uint8_t CalcPosition(uint8_t Direction, int8_t Delta);

#endif /* INTERRUPTHANDLER_H_ */
