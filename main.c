/*
 * "THE BEER-WARE LICENSE" (Revision 42):
 * Stefan Rupp <struppi@struppi.name> and
 * Daniel Steuer <daniel.steuer@bingo-ev.de>
 * wrote this file. As long as you retain this
 * notice you can do whatever you want with this stuff. If we meet some day,
 * and you think this stuff is worth it, you can buy me a beer in return.
 * (c) 2013 Stefan Rupp and Daniel Steuer
*/
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>

#include "can/can.h"
#include "setup.h"
#include "byteworker.h"
#include "timer.h"



void can_parse_msgs( can_t *msg );



typedef struct {
	// polar coordinates
	uint8_t r;
	uint8_t p;
} status_t;



config_t config;



int main( void ) {
	uint16_t i=0x40;
	uint8_t dir=0;
	can_t msgRx;
	can_t msgTx;

	wdt_disable();
	timer_init();
	bw_can_init( 500 /*kbit*/ );
	sei();

	bw_led_set( 1, 0 );
	bw_led_set( 0, 1 );

	DDRD |= (1<<PD2);
	wdt_enable( WDTO_250MS );

//	eeprom_init();

	// DELME
	msgTx.id = 0x55;
	msgTx.length = 0;
	msgTx.flags.rtr = 0;
	msgTx.flags.extended = 0;

	while( 1 ) {
		wdt_reset();

		if( can_check_message() ) {
			can_get_message( &msgRx );
			can_parse_msgs( &msgRx );
		}

		bw_led_toggle( 0 );
		bw_led_toggle( 1 );

		OCR1A = i;
		if(!dir) {
			i++;
		}
		else {
			i--;
		}

		if( i > 0x92 ) {
			dir=1;
		}
		else if ( i < 0x20 ) {
			dir=0;
		}

		timer_wait( 10 );

//		can_send_message( &msgTx );
	}
}



void can_parse_msgs( can_t *msgRx ) {

	switch( msgRx->id ) {
		case eCanIdReset:
			while(1); // wait for watchdog to reset
		break;

		case eCanIdSetAngles:
		break;

		default:
		break;
	}
}



retCode_e servo_setR( uint16_t requestedR ) {
	if( requestedR < config.rMin ) {
		return eErrorInvalidInput;
	}
	if( requestedR > config.rMax ) {
		return eErrorInvalidInput;
	}

	return eOK;
}
