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
#include "servo.h"



void can_parse_msgs( can_t *msg );
void send_heartbeat( void );



typedef struct {
	uint32_t hwId;
	uint16_t yawRaw;
	uint16_t pitchRaw;
	uint8_t  yaw;
	uint8_t  pitch;
	uint8_t  brightness;
} status_t;




config_t config;
status_t state;


int main( void ) {
	can_t msgRx;
	uint32_t now;
	static uint32_t lastHeartbeat = 0;

	wdt_disable();
	timer_init();
	servo_init();
	bw_can_init( 500 /*kbit*/ );
	sei();

	bw_led_set( 1, 0 );
	bw_led_set( 0, 1 );

	DDRD |= (1<<PD2);
	wdt_enable( WDTO_250MS );

	while( 1 ) {
		wdt_reset();
		now = timer_getMs();

		while( can_check_message() ) {
			can_get_message( &msgRx );
			can_parse_msgs( &msgRx );
			bw_led_toggle( 0 );
		}


		if( lastHeartbeat + 100 < now ) {
			lastHeartbeat = now;
			send_heartbeat();
			bw_led_toggle( 1 );
		}
	}
}



void can_parse_msgs( can_t *msgRx ) {
	switch( msgRx->id ) {
		case eCanIdReset: {
			while(1); // wait for watchdog to reset
			break;
		}

		case eCanIdSetAddress: { // <HWId(32le)>
			state.hwId = ((uint32_t)msgRx->data[3]<<24) + ((uint32_t)msgRx->data[2]<<16) + ((uint32_t)msgRx->data[1]<<8) + msgRx->data[0];
			break;
		}

		case eCanIdSetPosRaw: { // <pitch(16)><yaw(16)><brightness(8)>
			state.pitchRaw   = (msgRx->data[1]<<8) + msgRx->data[0];
			state.yawRaw     = (msgRx->data[2]<<8) + msgRx->data[3];
			state.brightness = msgRx->data[4];
			break;
		}

		case eCanIdCalibrateUpperLimit: { // no data
			break;
		}

		case eCanIdCalibrateLowerLimit: { // no data
			break;
		}

		case eCanIdSetPos: { // <pitch(8)><yaw(8)><brightness(8)>
			state.pitch      = msgRx->data[0];
			state.yaw        = msgRx->data[1];
			state.brightness = msgRx->data[2];
			break;
		}

		default: {
			break;
		}
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



void send_heartbeat( void ) {
	can_t msgTx;

	msgTx.id = 0x200;
	msgTx.length = 0; // no data
	msgTx.flags.rtr = 0;
	msgTx.flags.extended = 0;

	can_send_message( &msgTx );
}
