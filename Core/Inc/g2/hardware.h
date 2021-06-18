#ifndef HARDWARE_H_ONCE
#define HARDWARE_H_ONCE
//--------------------------------------------------------------
#include "stm32f2xx.h"

#define TIM_PRESCALER		  30	// 1Mhz clock
#define TIM_CLOCK			  (SystemCoreClock/2/TIM_PRESCALER) // 1Mhz
#define F_CPU 2000000UL		  // should always precede <avr/delay.h>
//
/*--- Hardware platform enumerations ---*/
enum hwPlatform {
	HM_PLATFORM_NONE = 0,

	HW_PLATFORM_TINYG_XMEGA,	// TinyG code base on Xmega boards.
								//	hwVersion 7 = TinyG v7 and earlier
								//	hwVersion 8 = TinyG v8

	HW_PLATFORM_G2_DUE,			// G2 code base on native Arduino Due

	HW_PLATFORM_TINYG_V9		// G2 code base on v9 boards
								//  hwVersion 0 = v9c
								//  hwVersion 1 = v9d
								//  hwVersion 2 = v9f
								//  hwVersion 3 = v9h
								//  hwVersion 4 = v9i
};

#define HW_VERSION_TINYGV6		6
#define HW_VERSION_TINYGV7		7
#define HW_VERSION_TINYGV8		8

#define HW_VERSION_TINYGV9C		0
#define HW_VERSION_TINYGV9D		1
#define HW_VERSION_TINYGV9F		2
#define HW_VERSION_TINYGV9H		3
#define HW_VERSION_TINYGV9I		4

////////////////////////////
/////// AVR VERSION ////////
////////////////////////////

#include "config.h"						// needed for the stat_t typedef

// uncomment once motate Xmega port is available
//#include "motatePins.h"
//#include "motateTimers.h"				// for Motate::timer_number

/*************************
 * Global System Defines *
 *************************/
										// CPU clock - set for delays
#define MILLISECONDS_PER_TICK 1			// MS for system tick (systick * N)
#define SYS_ID_LEN 7					// length of system ID string from sys_get_id()
/*
 * Port setup - Stepper / Switch Ports:
 *	b0	(out) step			(SET is step,  CLR is rest)
 *	b1	(out) direction		(CLR = Clockwise)
 *	b2	(out) motor enable 	(CLR = Enabled)
 *	b3	(out) microstep 0
 *	b4	(out) microstep 1
 *	b5	(out) output bit for GPIO port1
 *	b6	(in) min limit switch on GPIO 2 (note: motor controls and GPIO2 port mappings are not the same)
 *	b7	(in) max limit switch on GPIO 2 (note: motor controls and GPIO2 port mappings are not the same)
 */
#define MOTOR_PORT_DIR_gm 0x3F	// dir settings: lower 6 out, upper 2 in
//#define MOTOR_PORT_DIR_gm 0x00	// dir settings: all inputs

enum cfgPortBits {			// motor control port bit positions
	STEP_BIT_bp = 0,		// bit 0
	DIRECTION_BIT_bp,		// bit 1
	MOTOR_ENABLE_BIT_bp,	// bit 2
	MICROSTEP_BIT_0_bp,		// bit 3
	MICROSTEP_BIT_1_bp,		// bit 4
	GPIO1_OUT_BIT_bp,		// bit 5 (4 gpio1 output bits; 1 from each axis)
	SW_MIN_BIT_bp,			// bit 6 (4 input bits for homing/limit switches)
	SW_MAX_BIT_bp			// bit 7 (4 input bits for homing/limit switches)
};

#define STEP_BIT_bm			(1<<STEP_BIT_bp)
#define DIRECTION_BIT_bm	(1<<DIRECTION_BIT_bp)
#define MOTOR_ENABLE_BIT_bm (1<<MOTOR_ENABLE_BIT_bp)
#define MICROSTEP_BIT_0_bm	(1<<MICROSTEP_BIT_0_bp)
#define MICROSTEP_BIT_1_bm	(1<<MICROSTEP_BIT_1_bp)
#define GPIO1_OUT_BIT_bm	(1<<GPIO1_OUT_BIT_bp)	// spindle and coolant output bits
#define SW_MIN_BIT_bm		(1<<SW_MIN_BIT_bp)		// minimum switch inputs
#define SW_MAX_BIT_bm		(1<<SW_MAX_BIT_bp)		// maximum switch inputs

/* Bit assignments for GPIO1_OUTs for spindle, PWM and coolant */

#define SPINDLE_BIT			0x08		// spindle on/off
#define SPINDLE_DIR			0x04		// spindle direction, 1=CW, 0=CCW
#define SPINDLE_PWM			0x02		// spindle PWMs output bit
#define MIST_COOLANT_BIT	0x01		// coolant on/off - these are the same due to limited ports
#define FLOOD_COOLANT_BIT	0x01		// coolant on/off

#define SPINDLE_LED			0
#define SPINDLE_DIR_LED		1
#define SPINDLE_PWM_LED		2
#define COOLANT_LED			3

#define INDICATOR_LED		SPINDLE_DIR_LED	// can use the spindle direction as an indicator LED


/* Timer setup for stepper and dwells */

#define FREQUENCY_DDA 			(float)50000	// DDA frequency in hz.
#define FREQUENCY_DWELL			(float)10000	// Dwell count frequency in hz.
#define LOAD_TIMER_PERIOD 		100				// cycles you have to shut off SW interrupt
#define EXEC_TIMER_PERIOD 		100				// cycles you have to shut off SW interrupt
#define EXEC_TIMER_PERIOD_LONG 	100				// cycles you have to shut off SW interrupt

#define STEP_TIMER_TYPE		TC0_struct 		// stepper subsybstem uses all the TC0's
#define STEP_TIMER_DISABLE 	0				// turn timer off (clock = 0 Hz)
#define STEP_TIMER_ENABLE	1				// turn timer clock on (F_CPU = 32 Mhz)
#define STEP_TIMER_WGMODE	0				// normal mode (count to TOP and rollover)

#define LOAD_TIMER_DISABLE 	0				// turn load timer off (clock = 0 Hz)
#define LOAD_TIMER_ENABLE	1				// turn load timer clock on (F_CPU = 32 Mhz)
#define LOAD_TIMER_WGMODE	0				// normal mode (count to TOP and rollover)

#define EXEC_TIMER_DISABLE 	0				// turn exec timer off (clock = 0 Hz)
#define EXEC_TIMER_ENABLE	1				// turn exec timer clock on (F_CPU = 32 Mhz)
#define EXEC_TIMER_WGMODE	0				// normal mode (count to TOP and rollover)

/**** Device singleton - global structure to allow iteration through similar devices ****/
/*
	Ports are shared between steppers and GPIO so we need a global struct.
	Each xmega port has 3 bindings; motors, switches and the output bit

	The initialization sequence is important. the order is:
		- sys_init()	binds all ports to the device struct
		- st_init() 	sets IO directions and sets stepper VPORTS and stepper specific functions
		- gpio_init()	sets up input and output functions and required interrupts

	Care needs to be taken in routines that use ports not to write to bits that are
	not assigned to the designated function - ur unpredicatable results will occur
*/

/*** function prototypes ***/

void hardware_init(void);			// master hardware init
void hw_request_hard_reset();
void hw_hard_reset(void);
stat_t hw_hard_reset_handler(void);

void hw_request_bootloader(void);
stat_t hw_bootloader_handler(void);
stat_t hw_run_boot(nvObj_t *nv);

stat_t hw_set_hv(nvObj_t *nv);
stat_t hw_get_id(nvObj_t *nv);

#ifdef __TEXT_MODE

	void hw_print_fb(nvObj_t *nv);
	void hw_print_fv(nvObj_t *nv);
	void hw_print_hp(nvObj_t *nv);
	void hw_print_hv(nvObj_t *nv);
	void hw_print_id(nvObj_t *nv);

#else

	#define hw_print_fb tx_print_stub
	#define hw_print_fv tx_print_stub
	#define hw_print_hp tx_print_stub
	#define hw_print_hv tx_print_stub
	#define hw_print_id tx_print_stub

#endif // __TEXT_MODE
//----------------------------------------------------------------------------------------------------------------------------------
#define STEP_X_Pin 			GPIO_PIN_15
#define STEP_Y_Pin 			GPIO_PIN_13
#define STEP_Y_DUAL_Pin  	GPIO_PIN_5
#define STEP_Z_Pin 			GPIO_PIN_11
#define STEP_A_Pin 			GPIO_PIN_9
#define STEP_B_Pin 			GPIO_PIN_7
#define STEP_C_Pin 			GPIO_PIN_5

#define DIR_X_Pin 			GPIO_PIN_14
#define DIR_Y_Pin 			GPIO_PIN_12
#define DIR_Z_Pin 			GPIO_PIN_10
#define DIR_A_Pin 			GPIO_PIN_8
#define DIR_B_Pin 			GPIO_PIN_6
#define DIR_C_Pin 			GPIO_PIN_4
//-----------------------------------------------
#define STEP_X_Bit 			15		// PE15
#define STEP_Y_Bit 			13		// PE13
#define STEP_Y_DUAL_Bit 	5		// PE5
#define STEP_Z_Bit 			11		// PE11
#define STEP_A_Bit 			9		// PE9
#define STEP_B_Bit 			7		// PE7
#define STEP_C_Bit 			5		// PE5
//-----------------------------------------------
#define DIR_X_Bit 			14		// PE14
#define DIR_Y_Bit 			12		// PE12
#define DIR_Z_Bit 			10		// PE10
#define DIR_A_Bit 			8		// PE8
#define DIR_B_Bit 			6		// PE6
#define DIR_C_Bit 			4		// PE4

#define STEP_MASK			((1<<STEP_X_Bit)|(1<<STEP_Y_Bit)|(1<<STEP_Y_DUAL_Bit)|(1<<STEP_Z_Bit)|(1<<STEP_A_Bit)|(1<<STEP_B_Bit)|(1<<STEP_C_Bit))
#define DIRECTION_MASK  	((1<<DIR_X_Bit)|(1<<DIR_Y_Bit)|(1<<DIR_Z_Bit)|(1<<DIR_A_Bit)|(1<<DIR_B_Bit)|(1<<DIR_C_Bit))

#define STEPPERS_PORT		GPIOE

#define EN_STEPPER_Pin 		GPIO_PIN_2
#define EN_STEPPER_Port 	GPIOI
//-------------------------------------------------------------------------------------------------

#define PROBE_PIN  			GPIO_PIN_9
#define PROBE_GPIO_Port 	GPIOD
//
#define LIMITS_PORT	GPIOD
// D
#define LIMIT_PIN_3		GPIO_PIN_10
#define LIMIT_PIN_4		GPIO_PIN_13
#define LIMIT_PIN_5		GPIO_PIN_14
#define LIMIT_PIN_6		GPIO_PIN_15
// C
#define LIMIT_PIN_7		GPIO_PIN_6
#define LIMIT_PIN_8		GPIO_PIN_7
#define LIMIT_PIN_9		GPIO_PIN_8
#define LIMIT_PIN_10	GPIO_PIN_9
// A
#define LIMIT_PIN_11	GPIO_PIN_11
#define LIMIT_PIN_12	GPIO_PIN_12

#define PORTPINDEF 		uint16_t
#define PROBE_MASK       1 // don't change

#define HAL_WRITE_PORT(GPIOx,DATA)			(GPIOx->ODR = DATA)
#define HAL_READ_PORT(GPIOx)				(GPIOx->IDR)

#endif	// end of include guard: HARDWARE_H_ONCE
