#ifndef __setup_H__
#define __setup_H__

typedef enum {
	eCanIdHeartbeat           = 0x0200,
	eCanIdSetPos              = 0x0201,
	eCanIdCalibrateUpperLimit = 0x0202,
	eCanIdCalibrateLowerLimit = 0x0203,
	eCanIdSetPosRaw           = 0x0204,
	eCanIdReset               = 0x0205,
	eCanIdSetAddress          = 0x0206,
	eCanIdStoreConfigEEPROM   = 0x0207,
	eCanIdResponse            = 0x0208,
	eCanIdSendConfig          = 0x0209,
	eCanIdRequestConfig       = 0x020A,
	eCanIdEraseEEPROM         = 0x020C

} canMsgs_e;



// these are the default settings
// they are overwritten by what's stored
// in the eeprom
#define EEPROM_CONFIG_ADDR (0x0080)
typedef union {
	struct {
		uint8_t  inUseMarker;
		uint8_t  hwId;
		uint16_t pMin;
		uint16_t pMax;
		uint16_t yMin;
		uint16_t yMax;
	} data;

	uint8_t raw[10];
} config_t;



typedef enum {
	ePitch        = 0, // OCR1B
	eYaw          = 1, // OCR1A
	eBrightness   = 2, // PD7
	eNetworkLed   = 1, // blue led
	eHeartBeatLed = 0  // green led
} actor_e;



typedef enum {
	eServoSleepDelay = 1000, // ms
	eCanBaseAddress  = 0x200
}
delays_e;


#else
#error "double include of setup.h"
#endif
