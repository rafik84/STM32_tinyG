#include <stdio.h>
#include <stddef.h> 

#include "tinyg.h"
#include "system.h"

/*
 * sys_init() - lowest level hardware init
 */

/*
 * sys_get_id() - get a human readable signature
 *
 *	Produces a unique deviceID based on the factory calibration data. Format is:
 *		123456-ABC
 *
 *	The number part is a direct readout of the 6 digit lot number
 *	The alpha is the lo 5 bits of wafer number and XY coords in printable ASCII
 *	Refer to NVM_PROD_SIGNATURES_t in iox192a3.h for details.
 */
enum { 
	LOTNUM0=8,  // Lot Number Byte 0, ASCII 
	LOTNUM1,    // Lot Number Byte 1, ASCII 
	LOTNUM2,    // Lot Number Byte 2, ASCII 
	LOTNUM3,    // Lot Number Byte 3, ASCII 
	LOTNUM4,    // Lot Number Byte 4, ASCII 
	LOTNUM5,    // Lot Number Byte 5, ASCII 
	WAFNUM =16, // Wafer Number 
	COORDX0=18, // Wafer Coordinate X Byte 0 
	COORDX1,    // Wafer Coordinate X Byte 1 
	COORDY0,    // Wafer Coordinate Y Byte 0 
	COORDY1,    // Wafer Coordinate Y Byte 1 
}; 

void sys_get_id(char *id)
{
	char printable[10] = {"STM32F207"};
	uint8_t i;
	for (i=0; i<9; i++) {
		id[i] = printable[i];
	}
}
