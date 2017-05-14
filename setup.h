#ifndef __setup_H__
#define __setup_H__

typedef enum {
	eCanIdReset     = 0x123,
	eCanIdSetAngles = 0x124
} canMsgs_e;



// these are the default settings
// they are overwritten by what's stored
// in the eeprom
typedef struct {
	uint16_t rMin;
	uint16_t rMax;
	uint16_t pMin;
	uint16_t pMax;
} config_t;



typedef enum {
	eOK                =  0,
	eErrorInvalidInput = -1
} retCode_e;


#else
#error "double include of setup.h"
#endif
