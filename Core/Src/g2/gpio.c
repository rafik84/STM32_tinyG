#include "tinyg.h"
#include "util.h"
#include "config.h"
#include "controller.h"
#include "hardware.h"
//#include "switch.h"
#include "gpio.h"
#include "canonical_machine.h"
#include "xio.h"						// signals

//======================== Parallel IO Functions ===============================

/*
 * IndicatorLed_set() 	- fake out for IndicatorLed.set() until we get Motate running
 * IndicatorLed_clear() - fake out for IndicatorLed.clear() until we get Motate running
 * IndicatorLed_toggle()- fake out for IndicatorLed.toggle() until we get Motate running
 */

void IndicatorLed_set(){
	cs.led_state = 1;
}

void IndicatorLed_clear(){
	cs.led_state = 0;
}

void IndicatorLed_toggle(){
	if (cs.led_state == 0) {
		cs.led_state = 1;
	} else {
		cs.led_state = 0;
	}
}

/*
 * gpio_led_on() - turn led on - assumes TinyG LED mapping
 * gpio_led_off() - turn led on - assumes TinyG LED mapping
 * gpio_led_toggle()
 */

void gpio_led_on(uint8_t led){

}

void gpio_led_off(uint8_t led){

}

void gpio_led_toggle(uint8_t led){

}

/*
 * gpio_read_bit() - return true if bit is on, false if off
 * gpio_set_bit_on() - turn bit on
 * gpio_set_bit_off() - turn bit on
 *
 *	These functions have an inner remap depending on what hardware is running
 */

uint8_t gpio_read_bit(uint8_t b)
{
	return (0);
}

void gpio_set_bit_on(uint8_t b)
{

}

void gpio_set_bit_off(uint8_t b)
{

}
