#ifndef EEPROM_H_
#define EEPROM_H_

uint32_t EEPROMWrite(uint32_t *pui32_Data, uint32_t ui32WordAddress, uint32_t NumOfWords);
uint32_t EEPROMReadWords(uint32_t *pui32_Data, uint32_t ui32WordAddress, uint32_t NumOfWords);

#endif /* EEPROM_H_ */
