#ifndef __EEPROM_H__
#define __EEPROM_H__



uint8_t eeprom_read( uint16_t address );
void eeprom_write( uint16_t address, uint8_t data );



#endif // __EEPROM_H__
