/*
 * "THE BEER-WARE LICENSE" (Revision 42):
 * Martin Wenger <martin.wenger@arcormail.de> and Stefan Rupp <struppi@erlangen.ccc.de>
 * wrote this file. As long as you retain this notice you can do whatever you want
 * with this stuff. If we meet some day, and you think this stuff is worth it,
 * you can buy me/us a beer in return.
 * (c) 2005-2007 Martin Wenger, Stefan Rupp
 */
#ifndef __BYTEWORKER_H__
#define __BYTEWORKER_H__

int8_t bw_led_set( uint8_t led, uint8_t state );
int8_t bw_led_toggle( uint8_t led );
int8_t bw_can_init( uint16_t bitrate /* kbits */ );

#endif //__BYTEWORKER_H__
