/*
 * "THE BEER-WARE LICENSE" (Revision 42):
 * Daniel Steuer <daniel.steuer@bingo-ev.de>
 * wrote this file. As long as you retain this
 * notice you can do whatever you want with this stuff. If we meet some day,
 * and you think this stuff is worth it, you can buy me a beer in return.
 * (c) 2015 Daniel Steuer
*/
#include <stdint.h>
#include <avr/io.h>
#include "can/can.h"
#include "byteworker.h"



//
// I/O
//
int8_t bw_led_set( uint8_t led, uint8_t state ) {
	if( led == 0 ) {
		DDRC |= (1<<PC5);

		if( state )
			PORTC |= (1<<PC5);
		else
			PORTC &= ~(1<<PC5);
	}
	else if( led == 1 ) {
		DDRC |= (1<<PC6);

		if( state )
			PORTC |= (1<<PC6);
		else
			PORTC &= ~(1<<PC6);
	}

	return 0;
}



int8_t bw_led_toggle( uint8_t led ) {
	if( led == 0 ) {
		DDRC  |= (1<<PC5);
		PORTC ^= (1<<PC5);
	}
	else if( led == 1 ) {
		DDRC  |= (1<<PC6);
		PORTC ^= (1<<PC6);
	}

	return 0;
}



//
// CAN
//
int8_t bw_can_init( uint16_t bitrate /* kbits */ ) {
	switch( bitrate ) {
		case 125:
			can_init( BITRATE_125_KBPS );
		break;

		case 500:
			can_init( BITRATE_500_KBPS );
		break;

		case 1000:
			can_init( BITRATE_1_MBPS );
		break;

		default:
			return -1;
		break;
	}

	can_filter_t can_filter = { .id = 0, .mask = 0, .flags = { .rtr = 0, .extended = 0 } };
	can_set_filter( 0, &can_filter );
	can_set_filter( 1, &can_filter );
	can_set_filter( 2, &can_filter );

	// This is the TJA1043's standby and enable line.
	DDRB |= (1<<PB3) | (1<<PB4); // !STB! | EN
	PORTB |= (1<<PB3) | (1<<PB4); // ready for sending data

	return 0;
}
