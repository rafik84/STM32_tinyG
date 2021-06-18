#ifndef __EEPROM_H
#define __EEPROM_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx_hal.h"

/* Device voltage range supposed to be [2.7V to 3.6V], the operation will be done by word  */
#define VOLTAGE_RANGE			(uint8_t)FLASH_VOLTAGE_RANGE_3
/* EEPROM start address in Flash */
#define EEPROM_START_ADDRESS	((uint32_t)0x080E0000) /* EEPROM emulation start address (last sector): 384 Kb */
#define FLASH_SECTOR			FLASH_SECTOR_11

uint8_t memcpy_to_flash (uint8_t *source);
uint8_t memcpy_from_flash (uint8_t *dest);
#endif
