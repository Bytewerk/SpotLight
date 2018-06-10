#ifndef __setup_H__
#define __setup_H__

typedef enum {
	eCanIdHeartbeat           = 0x0200,
	eCanIdSetPos              = 0x0201,
	eCanIdCalibrateUpperLimit = 0x0202,
	eCanIdCalibrateLowerLimit = 0x0203,
	eCanIdSetPosRaw           = 0x0204,
	eCanIdReset               = 0x0205,
	eCanIdSetAddress          = 0x0206
} canMsgs_e;



// these are the default settings
// they are overwritten by what's stored
// in the eeprom
typedef struct {
	uint32_t canBaseAddress;
	uint16_t rMin;
	uint16_t rMax;
	uint16_t pMin;
	uint16_t pMax;
} config_t;



typedef enum {
	ePitch        = 0, // OCR1B
	eYaw          = 1, // OCR1A
	eBrightness   = 2, // PD7
	eNetworkLed   = 1, // blue led
	eHeartBeatLed = 0  // green led
} actor_e;



typedef enum {
	eServoSleepDelay = 2000, // ms
	eCanBaseAddress  = 0x200
}
delays_e;


#else
#error "double include of setup.h"
#endif
