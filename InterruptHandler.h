#ifndef INTERRUPTHANDLER_H_
#define INTERRUPTHANDLER_H_

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
