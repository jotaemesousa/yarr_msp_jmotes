/*
 * remote_defines.h
 *
 *  Created on: May 9, 2013
 *      Author: joao
 */

#ifndef REMOTE_DEFINES_H_
#define REMOTE_DEFINES_H_

#define START_CALIBRATION_TIME_MS 	2000
#define TIMEOUT_MS					250

typedef struct ROSpberryRemote
{
	int16_t linear;
	int16_t steer;
	uint8_t buttons;

}report_t;

typedef enum
{
	L1_BUTTON = 0x01,
	L2_BUTTON = 0x02,
	R1_BUTTON = 0x04,
	R2_BUTTON = 0x08,
	ASK_BIT = 0x10
}jmotes_buttons;

typedef struct CarParameters_
{
	int8_t steer_invert;
	int16_t steer_offset;
}CarParameters;

#endif /* REMOTE_DEFINES_H_ */
