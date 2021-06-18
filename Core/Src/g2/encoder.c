#include "tinyg.h"
#include "config.h"
#include "encoder.h"

/**** Allocate Structures ****/

enEncoders_t en;


void encoder_init(){
	memset(&en, 0, sizeof(en));		// clear all values, pointers and status
	encoder_init_assertions();
}

/*
 * encoder_init_assertions() - initialize encoder assertions
 * encoder_test_assertions() - test assertions, return error code if violation exists
 */

void encoder_init_assertions(){
	en.magic_end = MAGICNUM;
	en.magic_start = MAGICNUM;
}

stat_t encoder_test_assertions(){
	if (en.magic_end   != MAGICNUM) return (STAT_ENCODER_ASSERTION_FAILURE);
	if (en.magic_start != MAGICNUM) return (STAT_ENCODER_ASSERTION_FAILURE);
	return (STAT_OK);
}

/*
 * en_set_encoder_steps() - set encoder values to a current step count
 *
 *	Sets the encoder_position steps. Takes floating point steps as input,
 *	writes integer steps. So it's not an exact representation of machine
 *	position except if the machine is at zero.
 */

void en_set_encoder_steps(uint8_t motor, float steps){
	en.en[motor].encoder_steps = (int32_t)round(steps);
}

/*
 * en_read_encoder()
 *
 *	The stepper ISR count steps into steps_run(). These values are accumulated to
 *	encoder_position during LOAD (HI interrupt level). The encoder position is
 *	therefore always stable. But be advised: the position lags target and position
 *	valaues elsewherein the system becuase the sample is taken when the steps for
 *	that segment are complete.
 */

float en_read_encoder(uint8_t motor){
	return((float)en.en[motor].encoder_steps);
}
