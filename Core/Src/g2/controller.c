#include "tinyg.h"				// #1
#include "config.h"				// #2
#include "controller.h"
#include "json_parser.h"
#include "text_parser.h"
#include "gcode_parser.h"
#include "canonical_machine.h"
#include "plan_arc.h"
#include "planner.h"
#include "stepper.h"

#include "encoder.h"
#include "hardware.h"
#include "switch.h"
#include "gpio.h"
#include "report.h"
#include "util.h"
#include "xio.h"

/***********************************************************************************
 **** STRUCTURE ALLOCATIONS *********************************************************
 ***********************************************************************************/

controller_t cs;		// controller state structure

/***********************************************************************************
 **** STATICS AND LOCALS ***********************************************************
 ***********************************************************************************/

static void _controller_HSM(void);
static stat_t _shutdown_idler(void);
static stat_t _limit_switch_handler(void);
static stat_t _system_assertions(void);
static stat_t _sync_to_planner(void);
static stat_t _sync_to_tx_buffer(void);
static stat_t _command_dispatch(void);
/***********************************************************************************
 **** CODE *************************************************************************
 ***********************************************************************************/
/*
 * controller_init() - controller init
 */

void controller_init(){
	memset(&cs, 0, sizeof(controller_t));			// clear all values, job_id's, pointers and status
	controller_init_assertions();

	cs.fw_build 	= TINYG_FIRMWARE_BUILD;
	cs.fw_version 	= TINYG_FIRMWARE_VERSION;
	cs.hw_platform 	= TINYG_HARDWARE_PLATFORM;		// NB: HW version is set from EEPROM
	cs.state 		= CONTROLLER_STARTUP;			// ready to run startup lines
}

/*
 * controller_init_assertions()
 * controller_test_assertions() - check memory integrity of controller
 */

void controller_init_assertions(){
	cs.magic_start = MAGICNUM;
	cs.magic_end = MAGICNUM;
}

stat_t controller_test_assertions(){
	if ((cs.magic_start != MAGICNUM) || (cs.magic_end != MAGICNUM)) return (STAT_CONTROLLER_ASSERTION_FAILURE);
	return (STAT_OK);
}

/*
 * controller_run() - MAIN LOOP - top-level controller
 *
 * The order of the dispatched tasks is very important.
 * Tasks are ordered by increasing dependency (blocking hierarchy).
 * Tasks that are dependent on completion of lower-level tasks must be
 * later in the list than the task(s) they are dependent upon.
 *
 * Tasks must be written as continuations as they will be called repeatedly,
 * and are called even if they are not currently active.
 *
 * The DISPATCH macro calls the function and returns to the controller parent
 * if not finished (STAT_EAGAIN), preventing later routines from running
 * (they remain blocked). Any other condition - OK or ERR - drops through
 * and runs the next routine in the list.
 *
 * A routine that had no action (i.e. is OFF or idle) should return STAT_NOOP
 */

void controller_run()
{
	while (true) {
		_controller_HSM();
	}
}

#define	DISPATCH(func) if (func == STAT_EAGAIN) return;
static void _controller_HSM()
{
	DISPATCH(_shutdown_idler());				// 3. idle in shutdown state
//	DISPATCH( poll_switches());					// 4. run a switch polling cycle
	DISPATCH(_limit_switch_handler());			// 5. limit switch has been thrown

	DISPATCH(cm_feedhold_sequencing_callback());// 6a. feedhold state machine runner
	DISPATCH(mp_plan_hold_callback());			// 6b. plan a feedhold from line runtime
	DISPATCH(_system_assertions());				// 7. system integrity assertions
//----- planner hierarchy for gcode and cycles ---------------------------------------//
	//DISPATCH(st_motor_power_callback());		// stepper motor power sequencing
//	DISPATCH(switch_debounce_callback());		// debounce switches
	DISPATCH(sr_status_report_callback());		// conditionally send status report
	DISPATCH(qr_queue_report_callback());		// conditionally send queue report
	DISPATCH(rx_report_callback());             // conditionally send rx report
	DISPATCH(cm_arc_callback());				// arc generation runs behind lines
	DISPATCH(cm_homing_callback());				// G28.2 continuation
	DISPATCH(cm_jogging_callback());			// jog function
	DISPATCH(cm_probe_callback());				// G38.2 continuation
	DISPATCH(cm_deferred_write_callback());		// persist G10 changes when not in machining cycle
//----- command readers and parsers --------------------------------------------------//

	DISPATCH(_sync_to_planner());				// ensure there is at least one free buffer in planning queue
	DISPATCH(_sync_to_tx_buffer());				// sync with TX buffer (pseudo-blocking)
	DISPATCH(_command_dispatch());				// read and execute next command
}

/*****************************************************************************
 * _command_dispatch() - dispatch line received from active input device
 *
 *	Reads next command line and dispatches to relevant parser or action
 *	Accepts commands if the move queue has room - EAGAINS if it doesn't
 *	Manages cutback to serial input from file devices (EOF)
 *	Also responsible for prompts and for flow control
 */
#include "tcp.h"

static stat_t _command_dispatch(){
	//--stat_t status;
	memset(&cs.in_buf,0,INPUT_BUFFER_LEN);
	uint8_t c;
	uint8_t i = 0 ;
	while((c = TelnetRead()) != TCP_NO_DATA) {
		if ((c != '\n') && (c != '\r')) {
			cs.in_buf[i] = c ;i++;
		}
	}
	if(i == 0 ){
		return STAT_OK;
	}

	cs.bufp = cs.in_buf;
/**
	while (true) {
		if ((status = xio_gets(cs.primary_src, cs.in_buf, sizeof(cs.in_buf))) == STAT_OK) {
			cs.bufp = cs.in_buf;
			break;
		}
		// handle end-of-file from file devices
		if (status == STAT_EOF) {						// EOF can come from file devices only
			if (cfg.comm_mode == TEXT_MODE) {
				fprintf(stderr, PSTR("End of command file\n"));
			} else {
				rpt_exception(STAT_EOF);				// not really an exception
			}
		}
		return (status);								// Note: STAT_EAGAIN, errors, etc. will drop through
	}
**/
#ifdef __ARM
	// detect USB connection and transition to disconnected state if it disconnected
	if (SerialUSB.isConnected() == false) cs.state = CONTROLLER_NOT_CONNECTED;

	// read input line and return if not a completed line
	if (cs.state == CONTROLLER_READY) {
		if (read_line(cs.in_buf, &cs.read_index, sizeof(cs.in_buf)) != STAT_OK) {
			cs.bufp = cs.in_buf;
			return (STAT_OK);	// This is an exception: returns OK for anything NOT OK, so the idler always runs
		}
	} else if (cs.state == CONTROLLER_NOT_CONNECTED) {
		if (SerialUSB.isConnected() == false) return (STAT_OK);
		cm_request_queue_flush();
		rpt_print_system_ready_message();
		cs.state = CONTROLLER_STARTUP;

	} else if (cs.state == CONTROLLER_STARTUP) {		// run startup code
		cs.state = CONTROLLER_READY;

	} else {
		return (STAT_OK);
	}
	cs.read_index = 0;
#endif // __ARM

	// set up the buffers
	cs.linelen = strlen(cs.in_buf)+1;					// linelen only tracks primary input
	strncpy(cs.saved_buf, cs.bufp, SAVED_BUFFER_LEN-1);	// save input buffer for reporting

	// dispatch the new text line
	switch (toupper(*cs.bufp)) {						// first char
		case CHAR_RESET			: { hw_request_hard_reset(); break ;}
		case CHAR_FEEDHOLD		: { cm_request_feedhold(); break; }		// include for AVR diagnostics and ARM serial
		case CHAR_QUEUE_FLUSH   : { cm_request_queue_flush(); break; }
		case CHAR_CYCLE_START 	: { cm_request_cycle_start(); break; }

		case NUL: { 									// blank line (just a CR)
			if (cfg.comm_mode != JSON_MODE) {
				text_response(STAT_OK, cs.saved_buf);
			}
			break;
		}
		case '$': case '?': case 'H': { 				// text mode input
			cfg.comm_mode = TEXT_MODE;
			text_response(text_parser(cs.bufp), cs.saved_buf);
			break;
		}
		case '{': { 									// JSON input
			cfg.comm_mode = JSON_MODE;
			json_parser(cs.bufp);
			break;
		}
		default: {										// anything else must be Gcode
			if (cfg.comm_mode == JSON_MODE) {			// run it as JSON...
				strncpy(cs.out_buf, cs.bufp, INPUT_BUFFER_LEN -8);					// use out_buf as temp
				sprintf((char *)cs.bufp,"{\"gc\":\"%s\"}\n", (char *)cs.out_buf);	// '-8' is used for JSON chars
				json_parser(cs.bufp);
			} else {									//...or run it as text
				text_response(gc_gcode_parser(cs.bufp), cs.saved_buf);
			}
		}
	}
	return (STAT_OK);
}


/**** Local Utilities ********************************************************/
/*
 * _shutdown_idler() - blink rapidly and prevent further activity from occurring
 * _normal_idler() - blink Indicator LED slowly to show everything is OK
 *
 *	Shutdown idler flashes indicator LED rapidly to show everything is not OK.
 *	Shutdown idler returns EAGAIN causing the control loop to never advance beyond
 *	this point. It's important that the reset handler is still called so a SW reset
 *	(ctrl-x) or bootloader request can be processed.
 */

static stat_t _shutdown_idler(){
	if (cm_get_machine_state() != MACHINE_SHUTDOWN) { return (STAT_OK);}

	return (STAT_EAGAIN);	// EAGAIN prevents any lower-priority actions from running
}

/*
 * tg_reset_source() 		 - reset source to default input device (see note)
 * tg_set_primary_source() 	 - set current primary input source
 * tg_set_secondary_source() - set current primary input source
 *
 * Note: Once multiple serial devices are supported reset_source() should
 * be expanded to also set the stdout/stderr console device so the prompt
 * and other messages are sent to the active device.
 */

/*
 * _sync_to_tx_buffer() - return eagain if TX queue is backed up
 * _sync_to_planner() - return eagain if planner is not ready for a new command
 * _sync_to_time() - return eagain if planner is not ready for a new command
 */
static stat_t _sync_to_tx_buffer(){
	return (STAT_OK);
}

static stat_t _sync_to_planner(){
	if (cm_get_buffer_drain_state() == DRAIN_REQUESTED) {
		if (mp_get_planner_buffers_available() < PLANNER_BUFFER_POOL_SIZE) { // need to drain it first
			return (STAT_EAGAIN);
		}
		else {
			cm_set_buffer_drain_state(DRAIN_OFF);
			if (cfg.comm_mode == TEXT_MODE)
				text_response(STAT_OK, (char_t *)"");   // prompt
			return (STAT_OK);
		}
	}else {
		if (mp_get_planner_buffers_available() < PLANNER_BUFFER_HEADROOM) { // allow up to N planner buffers for this line
			return (STAT_EAGAIN);
		}
	}
	return (STAT_OK);
}

/*
 * _limit_switch_handler() - shut down system if limit switch fired
 */
static stat_t _limit_switch_handler(void){
	if (cm_get_machine_state() == MACHINE_ALARM) { return (STAT_NOOP);}

	if (get_limit_switch_thrown() == false) return (STAT_NOOP);
	return(cm_hard_alarm(STAT_LIMIT_SWITCH_HIT));
	return (STAT_OK);
}

/*
 * _system_assertions() - check memory integrity and other assertions
 */
#define EMERGY(a) if((status_code=a) != STAT_OK) return (cm_hard_alarm(status_code));

stat_t _system_assertions(){
	EMERGY(config_test_assertions());
	EMERGY(controller_test_assertions());
	EMERGY(canonical_machine_test_assertions());
	EMERGY(planner_test_assertions());
	EMERGY(stepper_test_assertions());
	EMERGY(encoder_test_assertions());
	EMERGY(xio_test_assertions());
	return (STAT_OK);
}
