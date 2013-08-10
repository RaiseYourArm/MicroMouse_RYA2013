/*
 * InterruptHandler.c
 *
 *  Created on: Jul 15, 2013
 *      Author: Admin
 */
#include "include.h"


extern PIDType PIDVerLeft, PIDVerRight, PIDPosLeft, PIDPosRight, PIDWallRight, PIDWallLeft, PIDWallFL, PIDWallFR;
extern int16_t avrSpeed;
extern double Ts;
extern uint8_t PIDFlag;
extern uint8_t FollowSel;
extern str_Network_Frame RecceiveFrame;
extern UARTType Network;
extern int8_t Direction;;

int32_t ADCResOn[4], ADCResOff[4], ADCResDelta[4];
int32_t ADCResult[4];
int32_t PosLeftCount = 0, PosRightCount = 0, EncRem = 0, DeltaEnc = 1500;	//1000	//1350
int32_t EncLeftCount, EncRightCount;
int8_t ControlFlag = 1;
int32_t MaxIRBase[4];
int32_t BuzzerTick = 0;
int8_t x = 0, y = 0;
uint8_t AvailDirection, RequestTurn = 0;

uint8_t *p_UARTBuf, UARTBuf[100];
uint8_t Stage = 0;
uint8_t ADCStatus = 0;
uint32_t set = 0;
uint8_t ADCError = 0;
uint8_t NetworkRequest = 0;

int8_t BasePoint[2] = {0, 0};

void EncRightISR(void)
{
	GPIOIntClear(GPIO_PORTC_BASE, GPIO_PIN_5);
		if (HWREG(GPIO_PORTC_BASE + 0x0100) & 0x40)
		{
			EncRightCount--;
			PosRightCount--;
		}
		else
		{
			EncRightCount++;
			PosRightCount++;
		}
//	}
}

void EncLeftISR(void)
{
	GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_6);
		if (HWREG(GPIO_PORTD_BASE + 0x0200) & 0x80)
		{
			EncLeftCount--;
			PosLeftCount--;
		}
		else
		{
			EncLeftCount++;
			PosLeftCount++;
		}

}

void ButtonsISR(void)
{
	uint32_t ui32Status;
	uint8_t count = 0;
	ui32Status = GPIOIntStatus(GPIO_PORTA_BASE,true);
	GPIOIntClear(GPIO_PORTA_BASE, ui32Status);
	if (ui32Status & GPIO_PIN_2)
	{
		while ((GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2) & GPIO_PIN_2) == 0)
		{
			SysCtlDelay(SysCtlClockGet() / 100);
			count++;
			if (count > 30)
			{
				BoostDisable();
				BuzzerTick = 350;

				break;
			}
		}
	}
}

void Timer5ISR(void)
{
	static unsigned char NumSpdSet = 0;
	TimerIntClear(TIMER5_BASE, TIMER_TIMA_TIMEOUT);


	if (BuzzerTick > 0)
	{
		BuzzerTick--;
		TimerMatchSet(TIMER1_BASE, TIMER_B, TimerLoadGet(TIMER1_BASE, TIMER_B) / 2);
	}
	else if (BuzzerTick == 0)
	{
		BuzzerTick--;
		TimerMatchSet(TIMER1_BASE, TIMER_B, 0);
	}

	NumSpdSet++;

	if (NumSpdSet == 20)	//PID position
	{
		NumSpdSet = 0;

#ifdef PID_POSITION
		UARTPutn(UART7_BASE, PosLeftCount);
		UARTCharPut(UART7_BASE, '\n');
#endif

		if (PIDPosLeft.Enable)
		{
			PIDPosCalc(&PIDPosLeft, PosLeftCount, avrSpeed);
			PIDSpeedSet(&PIDVerLeft, (long)PIDPosLeft.PIDResult);
			EncLeftCount = 0;
		}
		if (PIDPosRight.Enable)
		{
			PIDPosCalc(&PIDPosRight, PosRightCount, avrSpeed);
			PIDSpeedSet(&PIDVerRight, (long)PIDPosRight.PIDResult);
			EncRightCount = 0;
		}

		if (PIDFlag)
		{
			PIDFlag = 0;
		}
	}

	if (NumSpdSet % 10 == 0)
	{
#ifdef PID_SPEED
		UARTPutn(UART7_BASE, EncLeftCount);
		UARTCharPut(UART7_BASE, '\n');
#endif
		if (PIDVerLeft.Enable)
		{
			PIDVerCalc(&PIDVerLeft, &EncLeftCount, 90);
			SetPWM(TIMER1_BASE, TIMER_A, DEFAULT, (long)PIDVerLeft.PIDResult);
		}
		if (PIDVerRight.Enable)
		{
			PIDVerCalc(&PIDVerRight, &EncRightCount, 90);
			SetPWM(TIMER0_BASE, TIMER_A, DEFAULT, (long)PIDVerRight.PIDResult);
		}
		if (Stage == STRAIGHT)
		{
//			CalcPosition(Direction, (PosLeftCount + PosRightCount + 4000) / 7000);
			CalcPosition(Direction, (int8_t)((PosLeftCount + PosRightCount + 5500) / 6800));
		}
	}
}

void Timer4ISR(void)
{
	TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
	ADCStatus++;
	ADCStatus %= 20;

	switch (ADCStatus % 10)
	{
		case 0:
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0x10);
			break;
		case 1:
			ADCProcessorTrigger(ADC0_BASE, 2);
			break;
		case 2:
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0x20);
			break;
		case 3:
			ADCProcessorTrigger(ADC0_BASE, 2);
			break;
		case 4:
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0x40);
			break;
		case 5:
			ADCProcessorTrigger(ADC0_BASE, 2);
			break;
		case 6:
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0x80);
			break;
		case 7:
			ADCProcessorTrigger(ADC0_BASE, 2);
			break;
		case 8:
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0x00);
			break;
		case 9:
			ADCProcessorTrigger(ADC0_BASE, 2);
			break;
	}
}

void NetworkIntHandler(void)
{
	static uint8_t *p_NWByte, ByteCount;
	if (ByteCount == 0)
		p_NWByte = (uint8_t *)&RecceiveFrame;
	UARTIntClear(Network.PortName, UARTIntStatus(Network.PortName, true));
	while(UARTCharsAvail(Network.PortName))
	{
		*p_NWByte++ = (uint8_t)(UARTCharGetNonBlocking(Network.PortName));
		ByteCount++;
		if ((p_NWByte[-1]) == '\n')
		{
			NetworkRequest = 1;
			if (ByteCount != FrameLength + 1)
				NetworkRequest = 0;
			ByteCount = 0;
		}
	}
}

void BluetoothIntHandler(void)
{
//	uint8_t temp;
	unsigned long ulStatus;
	static unsigned char count = 0;
	ulStatus = UARTIntStatus(UART7_BASE, true);
	UARTIntClear(UART7_BASE, ulStatus);
	while(UARTCharsAvail(UART7_BASE))
	{
//		UARTCharGetNonBlocking(UART7_BASE);
		Stop();
#ifdef SET_PID
		ControlFlag = 1;
		*p_UARTBuf++ = (unsigned char)(UARTCharGetNonBlocking(UART7_BASE));
		if (p_UARTBuf[-1] == '\n')
		{
			p_UARTBuf = &UARTBuf[0];

			PIDVerLeft.Enable = 0;
			PIDVerRight.Enable = 0;
			GPIOPinWrite(ENABLE_PORT, ENA_LEFT_PIN | ENA_RIGHT_PIN, 0x00);
			FollowSel = FOLLOW_DISABLE;
			set = 0;
			count = 0;
		}
		else if (p_UARTBuf[-1] == '\r')
		{
			switch (count)
			{
				case 0:
					avrSpeed = (uint32_t)set / 10000;
					PIDVerLeft.SetPoint = avrSpeed;
					PIDPosLeft.SetPoint = avrSpeed;
					PIDPosRight.SetPoint = avrSpeed;
					break;
#ifdef PID_SPEED
				case 1:
					PIDVerLeft.Kp = (double)set / 10000;
					break;
				case 2:
					PIDVerLeft.Ki = (double)set / 10000;
					break;
				case 3:
					PIDVerLeft.Kd = (double)set / 10000;
					break;
#endif
#ifdef PID_POSITION
				case 1:
					PIDPosLeft.Kp = (double)set / 10000;
					PIDPosRight.Kp = (double)set / 10000;
					break;
				case 2:
					PIDPosLeft.Ki = (double)set / 10000;
					PIDPosRight.Ki = (double)set / 10000;
					break;
				case 3:
					PIDPosLeft.Kd = (double)set / 10000;
					PIDPosRight.Kd = (double)set / 10000;
					break;
#endif
#ifdef PID_WALL
//				case 1:
//					DeltaEnc = (double)set / 10000;
//					break;

				case 1:
					PIDWallRight.Kp = (double)set / 10000;
					break;
				case 2:
					PIDWallRight.Ki = (double)set / 10000;
					break;
				case 3:
					PIDWallRight.Kd = (double)set / 10000;
					break;
#endif
			}

			set = 0;
			count++;
		}
		else
		{
			set = set * 10 + (p_UARTBuf[-1] - 48);
		}
#else
		return;
#endif
	}
}

void ADCIsr(void)
{
	uint8_t temp;
	static uint8_t DirectionTemp;
//	uint8_t ui8RequestDir;
	ADCIntClear(ADC0_BASE, 2);
	ADCSequenceDataGet(ADC0_BASE, 2, &ADCResult[0]);
	switch (ADCStatus)
	{
		case 1:
			ADCResOn[0] = ADCResult[0];
			break;
		case 3:
			ADCResOn[1] = ADCResult[1];
			break;
		case 5:
			ADCResOn[2] = ADCResult[2];
			break;
		case 7:
			ADCResOn[3] = ADCResult[3];
			break;
		case 9:
			for (temp = 0; temp < 4; temp++)
			{
				ADCResDelta[temp] = ADCResOn[temp] - ADCResult[temp];
			}
			AvailDirection = GetAvailDir();
			if (AvailDirection != DirectionTemp)
			{
				RequestTurn = 1;
				DirectionTemp = AvailDirection;
			}
			if (FollowSel != FOLLOW_DISABLE)
				WallFollow(AvailDirection);
			break;
		default:
			ADCError = 1;
			break;
	}
}

uint8_t GetAvailDir(void)
{
	uint8_t AvailDir = 0;
	if (ADCResDelta[0] / 5 > PIDWallFL.SetPoint)
	{
		AvailDir |= AVAIL_FL;
	}
	if (ADCResDelta[1] / 5 > MaxIRBase[1])
	{
		AvailDir |= AVAIL_LEFT;
	}
	if (ADCResDelta[2] / 5 > MaxIRBase[2])
	{
		AvailDir |= AVAIL_RIGHT;
	}
	if (ADCResDelta[3] / 5 > PIDWallFR.SetPoint)
	{
		AvailDir |= AVAIL_FR;
	}
	return (AvailDir);
}

void BattSenseISR(void)
{
	uint8_t temp;
	uint32_t BattResult;
	static uint32_t avrBattResult = 2700;
	ADCIntClear(ADC1_BASE, 3);
	ADCSequenceDataGet(ADC1_BASE, 3, &BattResult);

	avrBattResult = (4 * avrBattResult + BattResult) / 5;
	if (avrBattResult < 2630)
	{
		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, 0x01);
		ControlFlag = 1;
		GPIOPinWrite(ENABLE_PORT, ENA_LEFT_PIN | ENA_RIGHT_PIN, 0x00);

		SetPWM(TIMER1_BASE, TIMER_A, DEFAULT, 0);
		SetPWM(TIMER0_BASE, TIMER_A, DEFAULT, 0);
		for (temp = 0; temp < 15; temp ++)
		{
			TimerMatchSet(TIMER1_BASE, TIMER_B, TimerLoadGet(TIMER1_BASE, TIMER_B) / 2);
			SysCtlDelay(SysCtlClockGet() / 100);
			TimerMatchSet(TIMER1_BASE, TIMER_B, 0);
			SysCtlDelay(SysCtlClockGet() / 80);
		}
		IntMasterDisable();
		BoostDisable();
	}
}

void TimerTickBatt(void)
{
	TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
	ADCProcessorTrigger(ADC1_BASE, 3);
}

uint8_t CalcPosition(uint8_t Direction, int8_t Delta)
{
	static int8_t DeltaTemp = 0;
	if (DeltaTemp != Delta)
	{
		//note:
		//
		DeltaTemp = Delta;
		switch (Direction)
		{
			case 0:
				y = BasePoint[1] + Delta;
				break;
			case 1:
				x = BasePoint[0] + Delta;
				break;
			case 2:
				y = BasePoint[1] - Delta;
				break;
			case 3:
				x = BasePoint[0] - Delta;
				break;
		}
		UARTPutn(UART0_BASE, x);
		UARTCharPut(UART0_BASE, ' ');
		UARTPutn(UART0_BASE, y);
		UARTCharPut(UART0_BASE, '\n');
		UARTCharPut(UART0_BASE, '\r');

		UARTPutn(UART7_BASE, x);
		UARTCharPut(UART7_BASE, ' ');
		UARTPutn(UART7_BASE, y);
		UARTCharPut(UART7_BASE, '\n');
		if (Stage == STRAIGHT)
			UpdateMap(x, y, Direction);
	if ((x == 5) && (y == 5))
	{
		BuzzerTick = 1000;
		ControlFlag = 1;
		GPIOPinWrite(ENABLE_PORT, ENA_LEFT_PIN | ENA_RIGHT_PIN, 0x00);
		PIDWallRight.Enable = 0;
		PIDWallLeft.Enable = 0;
		SetPWM(TIMER1_BASE, TIMER_A, DEFAULT, 0);
		SetPWM(TIMER0_BASE, TIMER_A, DEFAULT, 0);
		PIDPosLeft.Enable = 0;
		PIDPosRight.Enable = 0;
		BoostDisable();
		BasePoint[0] = 0;
		BasePoint[1] = 0;
		x = 0;
		y = 0;
		return (1);
	}
	}
	return (0);
}
