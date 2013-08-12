//*****************************************************************************
//
// Raise Your Arm 2013_ Micro Mouse robot.
//
// EEPROM.c - Function to read and write EEPROM
//
// This is part of revision 1.2 of the RYA Micro Mouse Library.
//      Happy coding.
//           Support Team RYA!
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup EEPROM_api
//! @{
//
//*****************************************************************************
#include "include.h"
#include "driverlib/eeprom.h"

//*****************************************************************************
//
//! Write data to EEPROM: address of word 0x000 - 0x7ff, total 2KB
//! One word combine 4 bytes
//!
//! \param *pui32_Data is pointer of data which you want to write
//! \param ui32WordAddress is address of EEPROM where you want to write
//! \param NumOfWords is number of word you want to write
//!
//!
//! \return Returns 0 on success or non-zero values on failure. Failure codes
//! are logical OR combi-nations ofEEPROM_RC_INVPL, EEPROM_RC_WRBUSY,
//! EEPROM_RC_NOPERM, EEP-ROM_RC_WKCOPY, EEPROM_RC_WKERASE,
//! and EEPROM_RC_WORKING.
//
//*****************************************************************************
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

//*****************************************************************************
//
//! Read data to EEPROM: address of word 0x000 - 0x7ff, total 2KB
//! One word combine 4 bytes
//! This function may be called to read a number of words of data from a
//! word-aligned address within the EEPROM. Data read is copied
//! into the buffer pointed to by thepui32Dataparameter.
//!
//! \param *pui32_Data is pointer of data which you want to read
//! \param ui32WordAddress is address of EEPROM where you want to read
//! \param NumOfWords is number of word you want to read
//!
//!
//! \return None
//
//*****************************************************************************
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


