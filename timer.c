/*
 * "THE BEER-WARE LICENSE" (Revision 42):
 * Martin Wenger <martin.wenger@arcormail.de> and Stefan Rupp <struppi@erlangen.ccc.de>
 * wrote this file. As long as you retain this notice you can do whatever you want
 * with this stuff. If we meet some day, and you think this stuff is worth it,
 * you can buy me/us a beer in return.
 * (c) 2005-2007 Martin Wenger, Stefan Rupp. Daniel Steuer
 */

#include <stdint.h>
#include <avr/io.h>
#include <util/atomic.h>
#include <avr/interrupt.h>
#include "timer.h"



/* TimerTime gets increased on every SIG_OUTPUT_COMPARE2 interrupt
	* those interrupts happen every 1ms
	*/
static volatile uint32_t timer_time;

ISR(TIMER0_COMPA_vect) {
	++timer_time;
	return;
}



/**
	* Initialize the timer
	* This function has to be called first, before calling TimerWait and/or TimerGet,
	* else TimerGet will always return 0, and TimerWait will wait forever!
	*/
void timer_init(void) {
	// Reset timer to zero
	timer_time = 0;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		// timebase: 1000 Hz
		TCCR0A = (1<<WGM01);
		TCCR0B = (1<<CS01)|(1<<CS00);
		OCR0A = 249;
		TIMSK0 |= (1<<OCIE0A);

		// pwm output for servos
		TCCR1A =  (1<<COM1A1)|(1<<COM1B1); // pin goes high on compare match
		TCCR1A |= (1<<WGM11)|(1<<WGM10); // 10 bit fast PWM

		TCCR1B =  (1<<WGM12); // 10 bit fast PWM (continued)
		TCCR1B |= (1<<CS12); // clockdivider F_CPU / 256
//		TCCR1B |= (1<<CS11)|(1<<CS10); // clockdivider F_CPU / 256
		// (16*10^6 Hz)/(256[clkdiv] * 1024[pwm top value]) = 61 Hz

		OCR1A = 0x01FF;
		OCR1B = 0x01FF;

//		TIMSK1 |= (1<<OCIE1A);//|(1<<OCIE1B);
	}
}



/**
	* Get the current time
	* \return the current time (in ms)
	*/
uint32_t timer_getMs(void) {
	uint32_t result;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		result = timer_time;
	}
	return result;
}




/**
	* Wait for (at least) the specified amount of time
	* \param delay The time to wait (in ms)
	*/

void timer_wait(uint32_t delay) {
	uint32_t end = timer_getMs() + delay +1;
	while ( end > timer_getMs() );
	return;
}