
/* Switch Modes
 *
 *	The switches are considered to be homing switches when machine_state is
 *	MACHINE_HOMING. At all other times they are treated as limit switches:
 *	  - Hitting a homing switch puts the current move into feedhold
 *	  - Hitting a limit switch causes the machine to shut down and go into lockdown until reset
 *
 * 	The normally open switch modes (NO) trigger an interrupt on the falling edge
 *	and lockout subsequent interrupts for the defined lockout period. This approach
 *	beats doing debouncing as an integration as switches fire immediately.
 *
 * 	The normally closed switch modes (NC) trigger an interrupt on the rising edge
 *	and lockout subsequent interrupts for the defined lockout period. Ditto on the method.
 */

#include "tinyg.h"
#include "config.h"
#include "switch.h"
#include "hardware.h"
#include "canonical_machine.h"
#include "text_parser.h"

/*
 * switch_init() - initialize homing/limit switches
 *
 *	This function assumes sys_init() and st_init() have been run previously to
 *	bind the ports and set bit IO directions, repsectively. See system.h for details
 */
/* Note: v7 boards have external strong pullups on GPIO2 pins (2.7K ohm).
 *	v6 and earlier use internal pullups only. Internal pullups are set
 *	regardless of board type but are extraneous for v7 boards.
 */
//#define PIN_MODE PORT_OPC_TOTEM_gc			// alternate pin mode for v7 boards

void switch_init(void)
{

}

/*
 * Switch closure processing routines
 *
 * ISRs 				 - switch interrupt handler vectors
 * _isr_helper()		 - common code for all switch ISRs
 * switch_rtc_callback() - called from RTC for each RTC tick.
 *
 *	These functions interact with each other to process switch closures and firing.
 *	Each switch has a counter which is initially set to negative SW_DEGLITCH_TICKS.
 *	When a switch closure is DETECTED the count increments for each RTC tick.
 *	When the count reaches zero the switch is tripped and action occurs.
 *	The counter continues to increment positive until the lockout is exceeded.
 */

void switch_rtc_callback(void){
	for (uint8_t i=0; i < NUM_SWITCHES; i++) {
		if (sw.mode[i] == SW_MODE_DISABLED || sw.debounce[i] == SW_IDLE)
            continue;

		if (++sw.count[i] == SW_LOCKOUT_TICKS) {		// state is either lockout or deglitching
			sw.debounce[i] = SW_IDLE;
            // check if the state has changed while we were in lockout...
            uint8_t old_state = sw.state[i];
            if(old_state != read_switch(i)) {
                sw.debounce[i] = SW_DEGLITCHING;
                sw.count[i] = -SW_DEGLITCH_TICKS;
            }
            continue;
		}
		if (sw.count[i] == 0) {							// trigger point
			sw.sw_num_thrown = i;						// record number of thrown switch
			sw.debounce[i] = SW_LOCKOUT;
//			sw_show_switch();							// only called if __DEBUG enabled

			if ((cm.cycle_state == CYCLE_HOMING) || (cm.cycle_state == CYCLE_PROBE)) {		// regardless of switch type
				cm_request_feedhold();
			} else if (sw.mode[i] & SW_LIMIT_BIT) {		// should be a limit switch, so fire it.
				sw.limit_flag = true;					// triggers an emergency shutdown
			}
		}
	}
}

/*
 * get_switch_mode()  - return switch mode setting
 * get_limit_thrown() - return true if a limit was tripped
 * get_switch_num()   - return switch number most recently thrown
 */

uint8_t get_switch_mode(uint8_t sw_num) { return (sw.mode[sw_num]);}
uint8_t get_limit_switch_thrown(void) { return(sw.limit_flag);}
uint8_t get_switch_thrown(void) { return(sw.sw_num_thrown);}


// global switch type
void set_switch_type( uint8_t switch_type ) { sw.switch_type = switch_type; }
uint8_t get_switch_type() { return sw.switch_type; }

/*
 * reset_switches() - reset all switches and reset limit flag
 */

void reset_switches()
{
	for (uint8_t i=0; i < NUM_SWITCHES; i++) {
		sw.debounce[i] = SW_IDLE;
        read_switch(i);
	}
	sw.limit_flag = false;
}

/*
 * read_switch() - read a switch directly with no interrupts or deglitching
 */
uint8_t read_switch(uint8_t sw_num)
{
	return 0 ;
}

/*
 * _show_switch() - simple display routine
 */
/*
void sw_show_switch(void)
{
	fprintf_P(stderr, PSTR("Limit Switch Thrown Xmin %d Xmax %d  Ymin %d Ymax %d  \
		Zmin %d Zmax %d Amin %d Amax %d\n"),
		sw.state[SW_MIN_X], sw.state[SW_MAX_X],
		sw.state[SW_MIN_Y], sw.state[SW_MAX_Y],
		sw.state[SW_MIN_Z], sw.state[SW_MAX_Z],
		sw.state[SW_MIN_A], sw.state[SW_MAX_A]);
}
*/

/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 * These functions are not part of the NIST defined functions
 ***********************************************************************************/

stat_t sw_set_st(nvObj_t *nv)			// switch type (global)
{
	set_01(nv);
	switch_init();
	return (STAT_OK);
}

stat_t sw_set_sw(nvObj_t *nv)			// switch setting
{
	if (nv->value > SW_MODE_MAX_VALUE)
        return (STAT_INPUT_VALUE_RANGE_ERROR);
	set_ui8(nv);
	switch_init();
	return (STAT_OK);
}

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

static const char fmt_st[] PROGMEM = "[st]  switch type%18d [0=NO,1=NC]\n";
void sw_print_st(nvObj_t *nv) { text_print_ui8(nv, fmt_st);}

//static const char fmt_ss[] PROGMEM = "Switch %s state:     %d\n";
//void sw_print_ss(nvObj_t *nv) { fprintf(stderr, fmt_ss, nv->token, (uint8_t)nv->value);}

/*
static const char msg_sw0[] PROGMEM = "Disabled";
static const char msg_sw1[] PROGMEM = "NO homing";
static const char msg_sw2[] PROGMEM = "NO homing & limit";
static const char msg_sw3[] PROGMEM = "NC homing";
static const char msg_sw4[] PROGMEM = "NC homing & limit";
static const char *const msg_sw[] PROGMEM = { msg_sw0, msg_sw1, msg_sw2, msg_sw3, msg_sw4 };
*/


#endif

/*============== G2 switch code - completely different, for now ===================

#include "tinyg2.h"
#include "switch.h"
#include "hardware.h"
#include "canonical_machine.h"

// Allocate switch array structure
switches_t sw;

static void _no_action(switch_t *s);
static void _led_on(switch_t *s);
static void _led_off(switch_t *s);
static void _trigger_feedhold(switch_t *s);
static void _trigger_cycle_start(switch_t *s);

 *
 * switch_init() - initialize homing/limit switches
 *
 *	This function assumes all Motate pins have been set up and that
 *	SW_PAIRS and SW_POSITIONS is accurate
 *
 *	Note: `type` and `mode` are not initialized as they should be set from configuration
 *

void switch_init(void)
{
//	sw.type = SW_NORMALLY_OPEN;				// set from config

	switch_t *s;	// shorthand

	for (uint8_t axis=0; axis<SW_PAIRS; axis++) {
		for (uint8_t position=0; position<SW_POSITIONS; position++) {
			s = &sw.s[axis][position];

			s->type = sw.type;				// propagate type from global type
//			s->mode = SW_MODE_DISABLED;		// set from config
			s->state = false;
			s->edge = SW_NO_EDGE;
			s->debounce_ticks = SW_LOCKOUT_TICKS;
			s->debounce_timeout = 0;

			// functions bound to each switch
			s->when_open = _no_action;
			s->when_closed = _no_action;
			s->on_leading = _trigger_feedhold;
			s->on_trailing = _trigger_cycle_start;
		}
	}
	// functions bound to individual switches
	// <none>
	// sw.s[AXIS_X][SW_MIN].when_open = _led_off;
	// sw.s[AXIS_X][SW_MIN].when_closed = _led_on;
}

static void _no_action(switch_t *s) { return; }
static void _led_on(switch_t *s) { IndicatorLed.clear(); }
static void _led_off(switch_t *s) { IndicatorLed.set(); }

 *
 * poll_switches() - run a polling cycle on all switches
 *

stat_t poll_switches()
{
	read_switch(&sw.s[AXIS_X][SW_MIN], axis_X_min_pin);
	read_switch(&sw.s[AXIS_X][SW_MAX], axis_X_max_pin);
	read_switch(&sw.s[AXIS_Y][SW_MIN], axis_Y_min_pin);
	read_switch(&sw.s[AXIS_Y][SW_MAX], axis_Y_max_pin);
	read_switch(&sw.s[AXIS_Z][SW_MIN], axis_Z_min_pin);
	read_switch(&sw.s[AXIS_Z][SW_MAX], axis_Z_max_pin);
	read_switch(&sw.s[AXIS_A][SW_MIN], axis_A_min_pin);
	read_switch(&sw.s[AXIS_A][SW_MAX], axis_A_max_pin);
	read_switch(&sw.s[AXIS_B][SW_MIN], axis_B_min_pin);
	read_switch(&sw.s[AXIS_B][SW_MAX], axis_B_max_pin);
	read_switch(&sw.s[AXIS_C][SW_MIN], axis_C_min_pin);
	read_switch(&sw.s[AXIS_C][SW_MAX], axis_C_max_pin);
	return (STAT_OK);
}

 *
 * read_switch() - read switch with NO/NC, debouncing and edge detection
 *
 *	Returns true if switch state changed - e.g. leading or falling edge detected
 *	Assumes pin_value input = 1 means open, 0 is closed. Pin sense is adjusted to mean:
 *	  0 = open for both NO and NC switches
 *	  1 = closed for both NO and NC switches
 *
uint8_t read_switch(switch_t *s, uint8_t pin_value)
{
	// instant return conditions: switch disabled or in a lockout period
	if (s->mode == SW_MODE_DISABLED) {
		return (false);
	}
	if (s->debounce_timeout > GetTickCount()) {
		return (false);
	}
	// return if no change in state
	uint8_t pin_sense_corrected = (pin_value ^ (s->type ^ 1));	// correct for NO or NC mode
  	if ( s->state == pin_sense_corrected) {
		s->edge = SW_NO_EDGE;
		if (s->state == SW_OPEN) {
			s->when_open(s);
		} else {
			s->when_closed(s);
		}
		return (false);
	}
	// the switch legitimately changed state - process edges
	if ((s->state = pin_sense_corrected) == SW_OPEN) {
			s->edge = SW_TRAILING;
			s->on_trailing(s);
		} else {
			s->edge = SW_LEADING;
			s->on_leading(s);
	}
	s->debounce_timeout = (GetTickCount() + s->debounce_ticks);
	return (true);
}

static void _trigger_feedhold(switch_t *s)
{
	IndicatorLed.toggle();
	cm_request_feedhold();

//	if (cm.cycle_state == CYCLE_HOMING) {		// regardless of switch type
//		cm.request_feedhold = true;
//	} else if (s->mode & SW_LIMIT_BIT) {		// set flag if it's a limit switch
//		cm.limit_tripped_flag = true;
//	}

}

static void _trigger_cycle_start(switch_t *s)
{
	IndicatorLed.toggle();
	cm_request_cycle_start();
}

 *
 * switch_get_switch_mode()  - return switch mode setting
 * switch_get_limit_thrown() - return true if a limit was tripped
 * switch_get_sw_num()  	 - return switch number most recently thrown
 *

uint8_t get_switch_mode(uint8_t sw_num) { return (0);}	// ++++

*/
