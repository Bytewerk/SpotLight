#include <stdint.h>
#include  <avr/io.h>
//#include <avr/common.h>
#include <avr/eeprom.h>



#define EEPE  1
#define EEMPE 2




uint8_t eeprom_read( uint16_t address ) {
	EEAR  = address;
	EECR &= (1<<EERE);    // read request

	while( EECR & (1<<EERE) ); // wait for response

	return EEDR;
}



void eeprom_write( uint16_t address, uint8_t data ) {
	while( EECR   & (1<<EEPE) );  // wait for possible write command to finish
	while( SPMCSR & (1<<SPMEN) );

	EEAR  = address;
	EEDR  = data;
  EECR |= (1<<EEMPE);             // set EEPROM Write Enabled
  EECR |= (1<<EEMPE) | (1<<EEPE); // set EEPROM Write (intentional double write of EEMPE)
}
