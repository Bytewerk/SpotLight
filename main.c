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
void updateLampState( void );
void lamp_setBrightness( uint8_t brightness );

typedef struct {
	uint32_t hwId;
	uint8_t  yaw;
	uint8_t  pitch;
	uint8_t  brightness;
} status_t;



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

	wdt_enable( WDTO_250MS );

	bw_led_set( 1, 0 );
	bw_led_set( 0, 0 );

	state.pitch      = 0;
	state.yaw        = 0;
	state.brightness = 0;

	while( 1 ) {
		wdt_reset();
		now = timer_getMs();

		while( can_check_message() ) {
			can_get_message( &msgRx );
			can_parse_msgs( &msgRx );
		}


		if( lastHeartbeat + 100 < now ) {
			lastHeartbeat = now;
			send_heartbeat();
			bw_led_toggle( 0 );
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
			uint16_t pitch = ((uint16_t)msgRx->data[0]<<8) + msgRx->data[1];
			uint16_t yaw   = ((uint16_t)msgRx->data[2]<<8) + msgRx->data[3];
			state.brightness = msgRx->data[4];

			servo_setRawValue( ePitch, pitch );
			servo_setRawValue( eYaw,   yaw );

			lamp_setBrightness( state.brightness );
			bw_led_toggle( 1 );
			break;
		}

		case eCanIdCalibrateUpperLimit: { // no data
			servo_calibrateUpperLimit();
			break;
		}

		case eCanIdCalibrateLowerLimit: { // no data
			servo_calibrateLowerLimit();
			break;
		}

		case eCanIdSetPos: { // <pitch(8)><yaw(8)><brightness(8)>
			state.pitch      = msgRx->data[0];
			state.yaw        = msgRx->data[1];
			state.brightness = msgRx->data[2];

			updateLampState();
			break;
		}

		default: {
			break;
		}
	}
}



void updateLampState( void ) {
	servo_setValue( ePitch, state.pitch );
	servo_setValue( eYaw,   state.yaw );
	lamp_setBrightness( state.brightness ); // NOTE: only ON/OFF supported yet
}



void lamp_setBrightness( uint8_t brightness ) {
	DDRD  |= (1<<PD7);
	if( brightness ) {
		PORTD |= (1<<PD7);
	}
	else {
		PORTD &= ~(1<<PD7);
	}
}



void send_heartbeat( void ) {
	can_t msgTx;
	uint16_t p = servo_getRawValue( ePitch );
	uint16_t y = servo_getRawValue( eYaw );


	msgTx.id = 0x200;
	msgTx.length = 8;
	msgTx.flags.rtr = 0;
	msgTx.flags.extended = 0;

	msgTx.data[0] = state.pitch;
	msgTx.data[1] = state.yaw;
	msgTx.data[2] = state.brightness;
	msgTx.data[3] = 0;
	msgTx.data[4] = (p >> 8) & 0xFF;
	msgTx.data[5] = p & 0xFF;
	msgTx.data[6] = (y >> 8) & 0xFF;
	msgTx.data[7] = y & 0xFF;

	can_send_message( &msgTx );
}
