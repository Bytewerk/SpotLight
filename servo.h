#ifndef __SERVO_H__
#define __SERVO_H__

void servo_init( void );
void servo_calibrateLowerLimit( void );
void servo_calibrateUpperLimit( void );
void servo_setValue( uint8_t servoId, uint8_t value );
void servo_setRawValue( uint8_t servoId, uint16_t value );
uint16_t servo_getRawValue( uint8_t servoId );
void servo_activate( void );
void servo_deActivate( void );

#else
#error "double include of setup.h"
#endif
