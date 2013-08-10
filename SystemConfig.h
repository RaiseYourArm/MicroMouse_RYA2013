#ifndef SYSTEMCONFIG_H_
#define SYSTEMCONFIG_H_

void ConfigPIDTimer(uint32_t TimerIntervalms, uint32_t PIDVerlocityLoop);
void ConfigADCTimer(void);
void ConfigSystem(void);
void ConfigPWM(void);
void EnableBuzzer(void);
void ConfigIRDetectors(void);
void ConfigNetwork(void);
void SetPWM(uint32_t ulBaseAddr, uint32_t ulTimer, uint32_t ulFrequency, int32_t ucDutyCycle);
void ConfigEncoder(void);
void ConfigIRTransmitter(void);
void ConfigBattSense(void);
void ConfigButtons(void (*Button1ISR)(void), void Button2ISR(void));

#endif /* SYSTEMCONFIG_H_ */
