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
#include "eeprom.h"



#define VERSION_MAJOR (1)
#define VERSION_MINOR (2)



void can_parse_msgs( can_t *msg );
void send_heartbeat( void );
void updateLampState( void );
void lamp_setBrightness( uint8_t brightness );
void send_responseCode( uint8_t responseCode );
void send_config( void );

typedef struct {
	uint32_t lastChangedPosition;
	uint8_t  servoPwmActive;
	uint8_t  yaw;
	uint8_t  pitch;
	uint8_t  brightness;
	uint8_t  lastYaw;
	uint8_t  lastPitch;
	uint8_t  lastBrightness;
} status_t;



status_t state;
config_t config;


int main( void ) {
	can_t msgRx;
	uint32_t now;
	uint32_t lastHeartbeat = 0;

	wdt_disable();

	timer_init();
	servo_init();
	bw_can_init( 500 /*kbit*/ );
	sei();

	wdt_enable( WDTO_250MS );

	bw_led_set( eNetworkLed,   0 );
	bw_led_set( eHeartBeatLed, 0 );

	eeprom_readConfig();

	state.pitch      = 0;
	state.yaw        = 0;
	state.brightness = 0;
	state.lastPitch  = 0;
	state.lastYaw    = 0;
	state.lastBrightness = 0;
	state.lastChangedPosition = 0;
	state.servoPwmActive = 1;

	send_config(); // tx config from eeprom for debugging

	while( 1 ) {
		wdt_reset();
		now = timer_getMs();

		while( can_check_message() ) {
			can_get_message( &msgRx );
			can_parse_msgs( &msgRx );
		}

		if( lastHeartbeat + 100 < now ) {
			bw_led_toggle( eHeartBeatLed );
			lastHeartbeat = now;
			send_heartbeat();
		}

		if( state.lastChangedPosition + eServoSleepDelay < now ) {
			if( state.servoPwmActive ) {
				servo_disable();
				state.servoPwmActive = 0;
				bw_led_set( eNetworkLed, 0 );
			}
		}
		else {
			if( !state.servoPwmActive ) {
				servo_enable();
				state.servoPwmActive = 1;
			}
			bw_led_set( eNetworkLed, 1 );
			updateLampState();
		}


	} // while(1)
}



void can_parse_msgs( can_t *msgRx ) {
	uint32_t now;

	uint16_t requestedId   = (msgRx->id & 0x02F0);
	uint16_t expectedId    = eCanIdHeartbeat + (config.data.hwId << 4);
	uint16_t maskedCommand = (msgRx->id & 0x000F) | eCanIdHeartbeat;

	if( requestedId != expectedId ) {
		return; // not our message
	}

	now = timer_getMs();

	switch( maskedCommand ) {
		case eCanIdReset: {
			send_responseCode( eCanIdReset & 0xFF );

			while(1); // wait for watchdog to reset
			break;
		}

		case eCanIdSetAddress: { // <HWId(8)>
			config.data.hwId = msgRx->data[0];
			send_responseCode( eCanIdSetAddress & 0xFF );
			break;
		}

		case eCanIdSetPosRaw: { // <pitch(16)><yaw(16)><brightness(8)>
			// this is for debug and calibration purposes only
			uint16_t pitch = ((uint16_t)msgRx->data[0]<<8) + msgRx->data[1];
			uint16_t yaw   = ((uint16_t)msgRx->data[2]<<8) + msgRx->data[3];
			uint8_t  brightness = msgRx->data[4];

			servo_enable();

			servo_setRawValue( ePitch, pitch );
			servo_setRawValue( eYaw,   yaw );

			lamp_setBrightness( brightness );
			send_responseCode( eCanIdSetPosRaw & 0xFF );
			break;
		}

		case eCanIdCalibrateUpperLimit: { // no data
			servo_calibrateUpperLimit();
			send_responseCode( eCanIdCalibrateUpperLimit & 0xFF );
			break;
		}

		case eCanIdCalibrateLowerLimit: { // no data
			servo_calibrateLowerLimit();
			send_responseCode( eCanIdCalibrateLowerLimit & 0xFF );
			break;
		}

		case eCanIdSetPos: { // <pitch(8)><yaw(8)><brightness(8)>
			state.pitch      = msgRx->data[0];
			state.yaw        = msgRx->data[1];
			state.brightness = msgRx->data[2];

			if( (state.lastPitch != state.pitch) ||
				(state.lastYaw != state.yaw) ||
				(state.lastBrightness != state.brightness)
			) {
				state.lastChangedPosition = now;
			}

			state.lastPitch      = state.pitch;
			state.lastYaw        = state.yaw;
			state.lastBrightness = state.brightness;
			send_responseCode( eCanIdSetPos & 0xFF );
			break;
		}

		case eCanIdStoreConfigEEPROM: {
			eeprom_writeConfig();
			send_responseCode( eCanIdStoreConfigEEPROM & 0xFF );
			break;
		}

		case eCanIdRequestConfig: {
			send_config();
			send_responseCode( eCanIdRequestConfig & 0xFF );
			break;
		}

		case eCanIdEraseEEPROM: {
			eeprom_erase();
			send_responseCode( eCanIdEraseEEPROM & 0xFF );
			break;
		}

		default: {
			send_responseCode( 0xFF );
			break;
		}
	}
}



void updateLampState( void ) {
	servo_setValue( ePitch, state.pitch );
	servo_setValue( eYaw,   state.yaw );
	lamp_setBrightness( state.brightness ); // NOTE: only ON/OFF supported
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

	msgTx.id = eCanIdHeartbeat + ((config.data.hwId)<<4);
	msgTx.length = 6;
	msgTx.flags.rtr = 0;
	msgTx.flags.extended = 0;

	msgTx.data[0] = state.pitch;
	msgTx.data[1] = state.yaw;
	msgTx.data[2] = state.brightness;
	msgTx.data[3] = 0;
	msgTx.data[4] = (VERSION_MAJOR) & 0xFF;
	msgTx.data[5] = (VERSION_MINOR) & 0xFF;

	can_send_message( &msgTx );
}



void send_responseCode( uint8_t responseCode ) {
	can_t msgTx;

	msgTx.id = eCanIdResponse + ((config.data.hwId)<<4);
	msgTx.length = 1;
	msgTx.flags.rtr = 0;
	msgTx.flags.extended = 0;

	msgTx.data[0] = responseCode;

	can_send_message( &msgTx );
}



void send_config( void ) {
	can_t msgTx;

	msgTx.id = eCanIdSendConfig + ((config.data.hwId)<<4);
	msgTx.flags.rtr = 0;
	msgTx.flags.extended = 0;

	msgTx.length = 2;
	msgTx.data[0] = (config.data.inUseMarker);
	msgTx.data[1] = (config.data.hwId);
	can_send_message( &msgTx );

	msgTx.length = 8;
	msgTx.data[0] = (config.data.pMin)>>8;
	msgTx.data[1] = (config.data.pMin);
	msgTx.data[2] = (config.data.pMax)>>8;
	msgTx.data[3] = (config.data.pMax);
	msgTx.data[4] = (config.data.yMin)>>8;
	msgTx.data[5] = (config.data.yMin);
	msgTx.data[6] = (config.data.yMax)>>8;
	msgTx.data[7] = (config.data.yMax);
	can_send_message( &msgTx );
}
