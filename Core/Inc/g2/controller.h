#ifndef CONTROLLER_H_ONCE
#define CONTROLLER_H_ONCE

#define INPUT_BUFFER_LEN  255			// text buffer size (255 max)
#define SAVED_BUFFER_LEN  100			// saved buffer size (for reporting only)
#define OUTPUT_BUFFER_LEN 512			// text buffer size
// see also: tinyg.h MESSAGE_LEN and config.h NV_ lengths

typedef struct controllerSingleton {	// main TG controller struct
	magic_t magic_start;				// magic number to test memory integrity
	uint8_t state;						// controller state
	float null;							// dumping ground for items with no target
	float fw_build;						// tinyg firmware build number
	float fw_version;					// tinyg firmware version number
	float hw_platform;					// tinyg hardware compatibility - platform type
	float hw_version;					// tinyg hardware compatibility - platform revision

	// communications state variables
	uint8_t primary_src;				// primary input source device
	uint8_t secondary_src;				// secondary input source device
	uint8_t default_src;				// default source device
	uint8_t network_mode;				// 0=master, 1=repeater, 2=slave

	uint16_t linelen;					// length of currently processing line
	uint16_t read_index;				// length of line being read

	// system state variables
	uint8_t led_state;		// LEGACY	// 0=off, 1=on
	int32_t led_counter;	// LEGACY	// a convenience for flashing an LED
	uint32_t led_timer;					// used by idlers to flash indicator LED
	uint8_t hard_reset_requested;		// flag to perform a hard reset
	uint8_t bootloader_requested;		// flag to enter the bootloader
	uint8_t shared_buf_overrun;			// flag for shared string buffer overrun condition

//	uint8_t sync_to_time_state;
//	uint32_t sync_to_time_time;

	int32_t job_id[4];					// uuid to identify the job

	// controller serial buffers
	char_t *bufp;						// pointer to primary or secondary in buffer
	char_t in_buf[INPUT_BUFFER_LEN];	// primary input buffer
	char_t out_buf[OUTPUT_BUFFER_LEN];	// output buffer
	char_t saved_buf[SAVED_BUFFER_LEN];	// save the input buffer
	magic_t magic_end;
} controller_t;

extern controller_t cs;					// controller state structure

enum cmControllerState {				// manages startup lines
	CONTROLLER_INITIALIZING = 0,		// controller is initializing - not ready for use
	CONTROLLER_NOT_CONNECTED,			// controller has not yet detected connection to USB (or other comm channel)
	CONTROLLER_CONNECTED,				// controller has connected to USB (or other comm channel)
	CONTROLLER_STARTUP,					// controller is running startup messages and lines
	CONTROLLER_READY					// controller is active and ready for use
};

/**** function prototypes ****/

void controller_init();
void controller_init_assertions(void);
stat_t controller_test_assertions(void);
void controller_run(void);
//void controller_reset(void);

void tg_reset_source(void);
void tg_set_primary_source(uint8_t dev);
void tg_set_secondary_source(uint8_t dev);

#ifdef __cplusplus
}
#endif

#endif // End of include guard: CONTROLLER_H_ONCE
