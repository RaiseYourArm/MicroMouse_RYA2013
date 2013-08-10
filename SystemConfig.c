#include "include.h"
#include "inc/hw_gpio.h"
//*****************************************************************************
//
// External declaration for the interrupt handler used by the application.
//
//*****************************************************************************
extern void ADCIsr(void);				//ADC Interrupt Routine - Used for 
										//read IR Detectors value
extern void NetworkIntHandler(void);	//Network-Interface Interrupt Routine
extern void Timer4ISR(void);			//Timer 4 - used to control IR transmitters - detectors
extern void Timer5ISR(void);			//Timer 5 - used for PID control
extern void EncRightISR(void);			//Encoder
extern void EncLeftISR(void);
extern void BattSenseISR(void);			//Under-Voltage Interrupt Routine
extern void TimerTickBatt(void);
extern void ButtonsISR(void);			//User-buttons Interrupt Routine

extern PIDType PIDVerLeft, PIDVerRight, PIDPosLeft, PIDPosRight, PIDWallRight, PIDWallLeft, PIDWallFL, PIDWallFR;

UARTType Network;

static uint8_t BuzzerEnable = 0;
uint32_t PIDVerLoop = 0;

void (*v_USRBT1)(void), (*v_USRBT2)(void);

void ConfigPIDTimer(uint32_t TimerIntervalms, uint32_t PIDVerlocityLoop)
{
	PIDVerLoop = PIDVerlocityLoop;
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
	TimerConfigure(TIMER5_BASE, TIMER_CFG_PERIODIC);
	TimerLoadSet(TIMER5_BASE, TIMER_A, (SysCtlClockGet() / 1000) * TimerIntervalms);	//Interval: //1:150
	TimerIntRegister(TIMER5_BASE, TIMER_A, &Timer5ISR);
	IntEnable(INT_TIMER5A);
	TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
	TimerIntClear(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
	TimerEnable(TIMER5_BASE, TIMER_A);
}

void ConfigADCTimer(void)
{
	//Wall Follow
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
	TimerConfigure(TIMER4_BASE, TIMER_CFG_PERIODIC);
	TimerLoadSet(TIMER4_BASE, TIMER_A, SysCtlClockGet() / 1000);	//Interval: 1ms
	TimerIntRegister(TIMER4_BASE, TIMER_A, &Timer4ISR);
	IntEnable(INT_TIMER4A);
	TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
	TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
	TimerEnable(TIMER4_BASE, TIMER_A);
}

void ConfigSystem(void)
{
	// Config clock
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	//Boost Converter Control
	SysCtlPeripheralEnable(ENAPORT_PERIPHERAL);
	GPIOPinTypeGPIOOutput(ENABLE_PORT, BOOST_EN_PIN);
	GPIOPinWrite(ENABLE_PORT, BOOST_EN_PIN, 0);
	//H-Bridges Control
	SysCtlPeripheralEnable(ENAPORT_PERIPHERAL);
	GPIOPinTypeGPIOOutput(ENABLE_PORT, ENA_LEFT_PIN | ENA_RIGHT_PIN);
	GPIOPinWrite(ENABLE_PORT, ENA_LEFT_PIN | ENA_RIGHT_PIN, 0x00);
}

void ConfigPWM(void)
{
	// Configure PF1 as T0CCP1
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	GPIOPinConfigure(GPIO_PB6_T0CCP0);		//Right
	GPIOPinConfigure(GPIO_PB4_T1CCP0);		//Left
	GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_6);
	

	// Configure timer
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);


	TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM);
	TimerLoadSet(TIMER0_BASE, TIMER_A, DEFAULT);
	TimerMatchSet(TIMER0_BASE, TIMER_A, DEFAULT); // PWM
	TimerControlLevel(TIMER0_BASE, TIMER_A, false);
	TimerEnable(TIMER0_BASE, TIMER_A);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
	if (BuzzerEnable)
	{
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
		GPIOPinConfigure(GPIO_PF3_T1CCP1);		//Buzzer
		GPIOPinTypeTimer(GPIO_PORTF_BASE, GPIO_PIN_3);
		TimerConfigure(TIMER1_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM);
		TimerMatchSet(TIMER1_BASE, TIMER_B, 0); // PWM
		TimerControlLevel(TIMER1_BASE, TIMER_B, true);//load=>match//buzzer1b
	}
	else
	{
		TimerConfigure(TIMER1_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM);
	}
	TimerLoadSet(TIMER1_BASE, TIMER_A, DEFAULT);
	TimerMatchSet(TIMER1_BASE, TIMER_A, DEFAULT); // PWM
	TimerControlLevel(TIMER1_BASE, TIMER_A, false);
	if (BuzzerEnable)
		TimerEnable(TIMER1_BASE, TIMER_BOTH);
	else
		TimerEnable(TIMER1_BASE, TIMER_A);
}

void EnableBuzzer(void)		//Call before ConfigPWM() function
{
	BuzzerEnable = 1;
}

void ConfigIRDetectors(void)
{
 	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
 	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
 	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);
 	ADCHardwareOversampleConfigure(ADC0_BASE, 32);

 	ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);
 	ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH1);//led 1
 	ADCSequenceStepConfigure(ADC0_BASE, 2, 1, ADC_CTL_CH0);//2
 	ADCSequenceStepConfigure(ADC0_BASE, 2, 2, ADC_CTL_CH9);//3
 	ADCSequenceStepConfigure(ADC0_BASE, 2, 3, ADC_CTL_END | ADC_CTL_CH8 | ADC_CTL_IE);//4
 	ADCSequenceEnable(ADC0_BASE, 2);//
 	ADCIntRegister(ADC0_BASE, 2, &ADCIsr);
 	ADCIntEnable(ADC0_BASE, 2);
}

void SetPWM(uint32_t ulBaseAddr, uint32_t ulTimer, uint32_t ulFrequency, int32_t ucDutyCycle)
{
	uint32_t ulPeriod;
	ulPeriod = SysCtlClockGet() / ulFrequency;
	TimerLoadSet(ulBaseAddr, ulTimer, ulPeriod);
	if (ucDutyCycle > 90)
		ucDutyCycle = 90;
	else if (ucDutyCycle < -90)
		ucDutyCycle = -90;
	TimerMatchSet(ulBaseAddr, ulTimer, (100 + ucDutyCycle) * ulPeriod / 200 - 1);
}

void ConfigEncoder(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) = 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) &= ~0x80;

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	GPIODirModeSet (GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_6, GPIO_DIR_MODE_IN);
	GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_FALLING_EDGE);
	GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_6, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	GPIODirModeSet (GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7, GPIO_DIR_MODE_IN);
	GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_FALLING_EDGE);
	GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);

	GPIOIntRegister(GPIO_PORTC_BASE, &EncRightISR);
	IntEnable(INT_GPIOC);
//	IntPrioritySet(INT_GPIOC, 0x03);
	GPIOIntEnable(GPIO_PORTC_BASE, GPIO_PIN_5);
	GPIOIntClear(GPIO_PORTC_BASE, GPIO_PIN_5);

	GPIOIntRegister(GPIO_PORTD_BASE, &EncLeftISR);
	IntEnable(INT_GPIOD);
//	IntPrioritySet(INT_GPIOC, 0x03);
	GPIOIntEnable(GPIO_PORTD_BASE, GPIO_PIN_6);
	GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_6);
}

void ConfigIRTransmitter(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0x00);
}

void ConfigBattSense(void)
{
 	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
 	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
 	GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_3);
 	ADCHardwareOversampleConfigure(ADC1_BASE, 64);

 	ADCSequenceConfigure(ADC1_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
 	ADCSequenceStepConfigure(ADC1_BASE, 3, 0, ADC_CTL_END | ADC_CTL_CH4 | ADC_CTL_IE);
 	ADCSequenceEnable(ADC1_BASE, 3);
 	ADCIntRegister(ADC1_BASE, 3, &BattSenseISR);
 	ADCIntEnable(ADC1_BASE, 3);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);

	TimerConfigure(TIMER3_BASE, TIMER_CFG_PERIODIC);
	TimerLoadSet(TIMER3_BASE, TIMER_A, SysCtlClockGet() / 10);	//Interval: 300s
	TimerIntRegister(TIMER3_BASE, TIMER_A, &TimerTickBatt);
	IntEnable(INT_TIMER3A);
	TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
	TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
	TimerEnable(TIMER3_BASE, TIMER_A);
}

void ConfigButtons(void (*Button1ISR)(void), void (*Button2ISR)(void))
{
	v_USRBT1 = Button1ISR;
	v_USRBT2 = Button2ISR;
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIODirModeSet (GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3, GPIO_DIR_MODE_IN);
	GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3, GPIO_FALLING_EDGE);
	GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);
	GPIOIntRegister(GPIO_PORTA_BASE, &ButtonsISR);
	IntEnable(INT_GPIOA);
//	IntPrioritySet(INT_GPIOA, 0x03);
	GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3);
	GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3);
}
