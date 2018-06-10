#include <stdint.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include "setup.h"
#include "servo.h"



typedef struct servo_calibration {
	uint16_t lowerLimit;
	uint16_t upperLimit;
} servo_t;

servo_t servo_p, servo_y;



void servo_init( void ) {
	servo_p.lowerLimit = 0x0500;
	servo_p.upperLimit = 0x0F00;

	servo_y.lowerLimit = 0x0370;
	servo_y.upperLimit = 0x10F0;

	servo_enable();

	OCR1A = servo_p.lowerLimit + (servo_p.upperLimit - servo_p.lowerLimit) / 2; // 1.5 ms is default
	OCR1B = servo_y.lowerLimit + (servo_y.upperLimit - servo_y.lowerLimit) / 2; // 1.5 ms is default
}



void servo_calibrateLowerLimit( void ) {
	// make sure upper and lower limit are not inverted
	if( OCR1A > servo_p.upperLimit ) {
		servo_p.lowerLimit = servo_p.upperLimit;
	}
	else {
		servo_p.lowerLimit = OCR1A;
	}

	if( OCR1B > servo_y.upperLimit ) {
		servo_y.lowerLimit = servo_y.upperLimit;
	}
	else {
		servo_y.lowerLimit = OCR1B;
	}
}

void servo_calibrateUpperLimit( void ) {
	// make sure upper and lower limit are not inverted
	if( OCR1A < servo_p.lowerLimit ) {
		servo_p.upperLimit = servo_p.lowerLimit;
	}
	else {
		servo_p.upperLimit = OCR1A;
	}

	if( OCR1B < servo_y.lowerLimit ) {
		servo_y.upperLimit = servo_y.lowerLimit;
	}
	else {
		servo_y.upperLimit = OCR1B;
	}
}



void servo_setValue( uint8_t servoId, uint8_t value ) {
	uint16_t range;

	switch( servoId ) {
		case ePitch: {
			range = servo_p.upperLimit - servo_p.lowerLimit;
			OCR1B = servo_p.lowerLimit + (((uint32_t)range * (uint32_t)value) / 0xff);
			break;
		}

		case eYaw: {
			range = servo_y.upperLimit - servo_y.lowerLimit;
			OCR1A = servo_y.lowerLimit + (((uint32_t)range * (uint32_t)value) / 0xff);
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
			OCR1B = value;
			break;
		}

		case eYaw: {
			OCR1A = value;
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
			}

			case eYaw: {
				tmp = OCR1A;
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
    TCCR1C = 0; // must be 0

    TIMSK1 = (1<<FOC1A)|(1<<FOC1B);

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
