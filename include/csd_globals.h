#ifndef __CSD_GLOBALS_H__
#define __CSD_GLOBALS_H__

#include <libopencmsis/core_cm3.h>

enum RUNMODE { STANDBY, ACTIVE, SLEEP } runMode;

struct hid_report {
	int8_t slider;
	int8_t thumb_x;
	int8_t thumb_y;
	uint8_t buttons;
} edtc_report;


/*
uint8_t sw1;
uint8_t sw2;
uint8_t sw1_lp;
uint8_t sw2_lp;
int16_t knob1;
uint16_t knob2;
uint16_t knob1_lp;
uint16_t knob2_lp;
*/

#endif