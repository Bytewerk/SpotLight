#include <stdint.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include "setup.h"
#include "servo.h"



extern config_t config;



static uint16_t buffer_OCR1A, buffer_OCR1B;



ISR( TIMER1_OVF_vect ) {
	OCR1A = buffer_OCR1A;
	OCR1B = buffer_OCR1B;
}



void servo_init( void ) {
	servo_enable();

	buffer_OCR1A = config.data.yMin + (config.data.yMax - config.data.yMin) / 2; // 1.5 ms is default
	buffer_OCR1B = config.data.pMin + (config.data.pMax - config.data.pMin) / 2; // 1.5 ms is default
}



void servo_calibrateLowerLimit( void ) {
	// make sure upper and lower limit are not inverted
	if( OCR1A > config.data.yMax ) {
		config.data.yMin = config.data.pMax;
	}
	else {
		config.data.yMin = OCR1A;
	}

	if( OCR1B > config.data.pMax ) {
		config.data.pMin = config.data.pMax;
	}
	else {
		config.data.pMin = OCR1B;
	}
}

void servo_calibrateUpperLimit( void ) {
	// make sure upper and lower limit are not inverted
	if( buffer_OCR1A < config.data.yMin ) {
		config.data.yMax = config.data.yMin;
	}
	else {
		config.data.yMax = OCR1A;
	}

	if( buffer_OCR1B < config.data.pMin ) {
		config.data.pMax = config.data.pMin;
	}
	else {
		config.data.pMax = OCR1B;
	}
}



void servo_setValue( uint8_t servoId, uint8_t value ) {
	uint16_t range;

	switch( servoId ) {
		case ePitch: {
			range = config.data.pMax - config.data.pMin;
			buffer_OCR1B = config.data.pMin + (((uint32_t)range * (uint32_t)value) / 0xff);
			break;
		}

		case eYaw: {
			range = config.data.yMax - config.data.yMin;
			buffer_OCR1A = config.data.yMin + (((uint32_t)range * (uint32_t)value) / 0xff);
			break;
		}

		default: {
			break;
		}
	}
}



void servo_setRawValue( uint8_t servoId, uint16_t value ) {
	switch( servoId ) {
		case ePitch: {
			buffer_OCR1B = value;
			break;
		}

		case eYaw: {
			buffer_OCR1A = value;
			break;
		}

		default: {
			break;
		}
	}
}



uint16_t servo_getRawValue( uint8_t servoId ) {
	uint16_t tmp = 0xFFFF;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		switch( servoId ) {
			case ePitch: {
				tmp = OCR1B;
				break;
			}

			case eYaw: {
				tmp = OCR1A;
				break;
			}

			default: {
				break;
			}
		}
	}
	return tmp;
}



void servo_enable( void ) {
	DDRD  |= (1<<PD2); // OCR1A
	DDRC  |= (1<<PC1); // OCR1B

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    TCCR1C = (1<<FOC1A) | (1<<FOC1B); // force output
    TIMSK1 = (1<<TOIE1);

    // pwm output for servos (16 bit fast PWM)
    TCCR1A  = (1<<COM1A1); // pin goes high on reset and low on compare match
    TCCR1A |= (1<<COM1B0)|(1<<COM1B1); // pin goes low on reset and high on compare match
    TCCR1A |= (1<<WGM11); // 16 bit fast PWM
    TCCR1B  = (1<<WGM12)|(1<<WGM13); // 16 bit fast PWM (continued) (ICR1=TOP)
    TCCR1B |= (1<<CS11); // clockdivider F_CPU / 8

    ICR1 = 40000; // 20 ms duty cycle
  }
}



void servo_disable( void ) {
	TCCR1B &= ~( (1<<CS12) | (1<<CS11) | (1<<CS10) ); // deactivate PWM clock

	// float both PWM pins
	DDRD   &= ~(1<<PD2);
	PORTD  &= ~(1<<PD2);

	DDRC   &= ~(1<<PC1);
	PORTC  &= ~(1<<PC1);
}
