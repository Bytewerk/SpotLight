#include <avr/interrupt.h>
#include <stdint.h>
#include "servo.h"



typedef struct servo_calibration {
	uint16_t lowerLimit;
	uint16_t upperLimit;
} servo_t;

servo_t s1, s2;



void servo_init( void ) {
	s1.lowerLimit = 0;
	s2.lowerLimit = 0;
	s1.upperLimit = 0xFFFF;
	s1.upperLimit = 0xFFFF;

	// setup timer 1 for servo PWM output

	TCCR1C = 0; // must be 0

	OCR1A = 500;
	OCR1B = 500;

	TIMSK1 = 0;

	TCCR1A  = (1<<WGM11)  | (1<<WGM10);// 10 bit fast PWM
	TCCR1A |= (1<<COM1A1) | (1<<COM1B1); // clear on match, start low

	TCCR1B  = (1<<CS11) | (1<<CS10); // clockdivider F_CPU / 64
	TCCR1B |= (1<<WGM12); // 10 bit fast PWM (continued)
}



void servo_calibrateLowerLimit( void ) {
	s1.lowerLimit = OCR1A;
	s2.lowerLimit = OCR1B;
}

void servo_calibrateUpperLimit( void ) {
	s1.upperLimit = OCR1A;
	s2.upperLimit = OCR1B;
}



void servo_setValue( uint8_t servoId, uint8_t value ) {
	uint16_t range;
	uint16_t scaleFactor;

	switch( servoId ) {
		case 0: {
			range = s1.upperLimit - s1.lowerLimit;
			OCR1A = range * scaleFactor;
			break;
		}

		case 1: {
			break;
		}

		default: {
			break;
		}
	}
}
