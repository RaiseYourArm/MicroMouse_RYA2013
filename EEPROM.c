/*
 * EEPROM.c
 *
 *  Created on: Aug 6, 2013
 *      Author: Admin
 */
#include "include.h"
#include "driverlib/eeprom.h"

uint32_t EEPROMWrite(uint32_t *pui32_Data, uint32_t ui32WordAddress, uint32_t NumOfWords)
{
	uint32_t Address;
	if (ui32WordAddress > 0x7ff)
	{
		return (INVALID_ADDRESS);
	}
	if ((ui32WordAddress + NumOfWords) > 0x7ff)
	{
		return (INVALID_NUM_OF_WORDS);
	}
	Address = ui32WordAddress << 2;
	return (EEPROMProgram(pui32_Data, Address, NumOfWords << 2));
}

uint32_t EEPROMReadWords(uint32_t *pui32_Data, uint32_t ui32WordAddress, uint32_t NumOfWords)
{
	uint32_t Address;
	if (ui32WordAddress > 0x7ff)
	{
		return (INVALID_ADDRESS);
	}
	if ((ui32WordAddress + NumOfWords) > 0x7ff)
	{
		return (INVALID_NUM_OF_WORDS);
	}
	Address = ui32WordAddress << 2;
	EEPROMRead(pui32_Data, Address, NumOfWords << 2);
	return (0);
}

