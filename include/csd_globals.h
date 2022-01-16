#ifndef __CSD_GLOBALS_H__
#define __CSD_GLOBALS_H__

#include <libopencmsis/core_cm3.h>

enum RUNMODE { STANDBY, ACTIVE, SLEEP } runMode;

struct hid_report {
	int8_t slider;
	int8_t thumb_x;
	int8_t thumb_y;
	uint8_t buttons;
	uint8_t buttons2;
} edtc_report;


#endif