#include <stdint.h>
#include <avr/io.h>
#include <avr/eeprom.h>

#include "setup.h"
#include "eeprom.h"



// the eeprom registers of the ATMEGA16M1
// are not fully described in the corresponding
// header files
#define EEPE  1
#define EEMPE 2



extern config_t config;



void eeprom_readConfig( void ) {
	uint8_t eeprom_data = eeprom_read_byte((uint8_t *)EEPROM_CONFIG_ADDR);
	if( eeprom_data == 0x01 ) {
		// config in eeprom detected
		for(uint8_t i=0; i < sizeof(config); ++i ) {
			config.raw[i] = eeprom_read_byte((uint8_t *)EEPROM_CONFIG_ADDR + i);
    }
  }
	else {
		// no config in eeprom. Use defaults
		config.data.hwId = 0;
		config.data.inUseMarker = 0;
		config.data.pMin = 0x05AA;
		config.data.pMax = 0x0FAA;
		config.data.yMin = 0x04AA;
		config.data.yMax = 0x10AA;
	}

}


void eeprom_writeConfig( void ) {
	// mark eeprom as flashed
	config.data.inUseMarker = 0x01;

	for(uint8_t i=0; i < sizeof(config); ++i ) {
		eeprom_write_byte((uint8_t *)(EEPROM_CONFIG_ADDR + i), config.raw[i] );
	}
}



void eeprom_erase( void ) {
	for(uint8_t i=0; i < sizeof(config); ++i ) {
		eeprom_update_byte(  (uint8_t *)(EEPROM_CONFIG_ADDR + i), 0xFF );
	//	eeprom_update_byte( (uint8_t *)(EEPROM_CONFIG_ADDR + i), 0xFF );
	}
}
