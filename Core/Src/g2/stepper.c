#include "tinyg.h"
#include "config.h"
#include "stepper.h"
#include "encoder.h"
#include "planner.h"
#include "report.h"
#include "hardware.h"
#include "text_parser.h"
#include "util.h"

TIM_HandleTypeDef tim2 ;
TIM_HandleTypeDef tim3 ;
TIM_HandleTypeDef tim4 ;
TIM_HandleTypeDef tim5 ;

void TIM2_IRQHandler(void){
  HAL_TIM_IRQHandler(&tim2);
}

void TIM3_IRQHandler(void){
  HAL_TIM_IRQHandler(&tim3);
}

void TIM4_IRQHandler(void){
  HAL_TIM_IRQHandler(&tim4);
}

void TIM5_IRQHandler(void){
  HAL_TIM_IRQHandler(&tim5);
}
#define TIM_START(TIM)		TIM->CR1 |= TIM_CR1_CEN
#define TIM_STOP(TIM)		TIM->CR1 &=~(TIM_CR1_CEN)
// Updatefrequency = TIM clk/((PSC+1)*(ARR+1))
/**** Allocate structures ****/
stConfig_t st_cfg;
stPrepSingleton_t st_pre;
static stRunSingleton_t st_run;

/**** Setup local functions ****/
void _load_move(void);
void _request_load_move(void);

// handy macro
#define _f_to_period(f) (uint16_t)((float)F_CPU / (float)f)

/************************************************************************************
 **** CODE **************************************************************************
 ************************************************************************************/
/*
 * stepper_init() - initialize stepper motor subsystem
 *
 *	Notes:
 *	  - This init requires sys_init() to be run beforehand
 * 	  - microsteps are setup during config_init()
 *	  - motor polarity is setup during config_init()
 *	  - high level interrupts must be enabled in main() once all inits are complete
 */
#define TIM_DDA		TIM2
#define TIM_DWELL	TIM3
#define TIM_LOAD	TIM4
#define TIM_EXEC	TIM5

void stepper_timers_init(){

	//--------------------------------
	__HAL_RCC_TIM2_CLK_ENABLE();	// DDA
	__HAL_RCC_TIM3_CLK_ENABLE();	// DWELL
	__HAL_RCC_TIM4_CLK_ENABLE();	// LOAD
	__HAL_RCC_TIM5_CLK_ENABLE();	// EXEC

	tim2.Instance = TIM2;
	tim2.Init.Period = 0;
	tim2.Init.Prescaler = (TIM_PRESCALER-1);
	tim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	tim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

	HAL_TIM_Base_Init(&tim2);
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    TIM2->DIER 	= TIM_DIER_UIE;
    TIM2->EGR 	= TIM_EGR_UG;
/**
    TIM_DDA->CR1 &= ~TIM_CR1_CEN;// Stop the timer
    TIM_DDA->ARR = 250000;
    TIM_DDA->CNT = 0;            // Reset the timer
    TIM_DDA->CR1 = TIM_CR1_CEN | TIM_CR1_ARPE; // Enable the timer
**/
	//----------------------------------------------------------
	tim3.Instance = TIM3;
	tim3.Init.Prescaler  = (TIM_PRESCALER-1);
	tim3.Init.Period = 0;
	tim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	tim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

	HAL_TIM_Base_Init(&tim3);
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);

    TIM3->DIER 	= TIM_DIER_UIE;
    TIM3->EGR 	= TIM_EGR_UG;
	//----------------------------------------------------------
	tim4.Instance = TIM4;
	tim4.Init.Prescaler  = (TIM_PRESCALER-1);
	tim4.Init.Period = 0;
	tim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	tim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&tim4);
    HAL_NVIC_SetPriority(TIM4_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
    TIM4->DIER 	= TIM_DIER_UIE;
    TIM4->EGR 	= TIM_EGR_UG;
	//----------------------------------------------------------
	tim5.Instance = TIM5;
	tim5.Init.Prescaler  = (TIM_PRESCALER-1);
	tim5.Init.Period = 0;
	tim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	tim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&tim5);
    HAL_NVIC_SetPriority(TIM5_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
	//HAL_TIM_Base_Start_IT(&tim5);
    TIM5->DIER 	= TIM_DIER_UIE;
    TIM5->EGR 	= TIM_EGR_UG;
    // init GPIO // |DIR_X_Pin |DIR_Y_Pin|DIR_Z_Pin
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    // PIN Steps
    GPIO_InitStruct.Pin = STEP_X_Pin|STEP_Y_Pin|STEP_Z_Pin|STEP_Y_DUAL_Pin|STEP_A_Pin|STEP_B_Pin|STEP_C_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  	HAL_GPIO_Init(STEPPERS_PORT, &GPIO_InitStruct);
  	HAL_GPIO_WritePin(STEPPERS_PORT, STEP_X_Pin|STEP_Y_Pin|STEP_Z_Pin|STEP_Y_DUAL_Pin|STEP_A_Pin|STEP_B_Pin|STEP_C_Pin , GPIO_PIN_RESET);
  	// PIN Dir
    GPIO_InitStruct.Pin = DIR_X_Pin |DIR_Y_Pin|DIR_Z_Pin|DIR_A_Pin|DIR_B_Pin|DIR_C_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  	HAL_GPIO_Init(STEPPERS_PORT, &GPIO_InitStruct);
  	HAL_GPIO_WritePin(STEPPERS_PORT, DIR_X_Pin |DIR_Y_Pin|DIR_Z_Pin|DIR_A_Pin|DIR_B_Pin|DIR_C_Pin , GPIO_PIN_RESET);
  	// EN PIN
  	GPIO_InitTypeDef GPIO_EnStruct = {0};
  	GPIO_EnStruct.Pin = EN_STEPPER_Pin ;
  	GPIO_EnStruct.Mode = GPIO_MODE_OUTPUT_PP;
  	GPIO_EnStruct.Pull = GPIO_NOPULL;
  	GPIO_EnStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  	HAL_GPIO_Init(EN_STEPPER_Port, &GPIO_EnStruct);
  	HAL_GPIO_WritePin(EN_STEPPER_Port, EN_STEPPER_Pin , GPIO_PIN_RESET);
}
//--------------------------
void stepper_init(){
	memset(&st_run, 0, sizeof(st_run));			// clear all values, pointers and status
	stepper_init_assertions();

	st_pre.buffer_state = PREP_BUFFER_OWNED_BY_EXEC;
	st_reset();									// reset steppers to known state
}

/*
 * stepper_init_assertions() - test assertions, return error code if violation exists
 * stepper_test_assertions() - test assertions, return error code if violation exists
 */

void stepper_init_assertions(){
	st_run.magic_end = MAGICNUM;
	st_run.magic_start = MAGICNUM;
	st_pre.magic_end = MAGICNUM;
	st_pre.magic_start = MAGICNUM;
}

stat_t stepper_test_assertions(){
	if (st_run.magic_end	!= MAGICNUM) return (STAT_STEPPER_ASSERTION_FAILURE);
	if (st_run.magic_start	!= MAGICNUM) return (STAT_STEPPER_ASSERTION_FAILURE);
	if (st_pre.magic_end	!= MAGICNUM) return (STAT_STEPPER_ASSERTION_FAILURE);
	if (st_pre.magic_start	!= MAGICNUM) return (STAT_STEPPER_ASSERTION_FAILURE);
	return (STAT_OK);
}

/*
 * st_runtime_isbusy() - return TRUE if runtime is busy:
 *
 *	Busy conditions:
 *	- motors are running
 *	- dwell is running
 */

uint8_t st_runtime_isbusy(){
	if (st_run.dda_ticks_downcount == 0) {

		return (false);
	}
	return (true);
}

/*
 * st_reset() - reset stepper internals
 */

void st_reset(){
    st_run.dda_ticks_downcount = 0;                     // signal the runtime is not busy
    st_pre.buffer_state = PREP_BUFFER_OWNED_BY_EXEC;    // set to EXEC or it won't restart

	for (uint8_t motor=0; motor<MOTORS; motor++) {
		st_pre.mot[motor].prev_direction = STEP_INITIAL_DIRECTION;
		st_run.mot[motor].substep_accumulator = 0;	// will become max negative during per-motor setup;
		st_pre.mot[motor].corrected_steps = 0;		// diagnostic only - no action effect
	}
	mp_set_steps_to_runtime_position();
}

/*
 * st_clc() - clear counters
 */
// clear diagnostic counters, reset stepper prep
stat_t st_clc(nvObj_t *nv){
	st_reset();
	return(STAT_OK);
}

/*
 * Motor power management functions
 *
 * _deenergize_motor()		 - remove power from a motor
 * _energize_motor()		 - apply power to a motor
 * _set_motor_power_level()	 - set the actual Vref to a specified power level
 *
 * st_energize_motors()		 - apply power to all motors
 * st_deenergize_motors()	 - remove power from all motors
 * st_motor_power_callback() - callback to manage motor power sequencing
 */

static uint8_t _motor_is_enabled(uint8_t motor){
	uint8_t port;
	switch(motor) {
		default: port = 0xff;	// defaults to disabled for bad motor input value
	}
	return ((port & MOTOR_ENABLE_BIT_bm) ? 0 : 1);	// returns 1 if motor is enabled (motor is actually active low)
}

static void _deenergize_motor(const uint8_t motor){
	st_run.mot[motor].power_state = MOTOR_OFF;
}

static void _energize_motor(const uint8_t motor){
	if (st_cfg.mot[motor].power_mode == MOTOR_DISABLED) {
		_deenergize_motor(motor);
		return;
	}
	st_run.mot[motor].power_state = MOTOR_POWER_TIMEOUT_START;
}

/*
 * _set_motor_power_level()	- applies the power level to the requested motor.
 *
 *	The power_level must be a compensated PWM value - presumably one of:
 *		st_cfg.mot[motor].power_level_scaled
 *		st_run.mot[motor].power_level_dynamic
 */


void st_energize_motors(){
	for (uint8_t motor = MOTOR_1; motor < MOTORS; motor++) {
		_energize_motor(motor);
		st_run.mot[motor].power_state = MOTOR_POWER_TIMEOUT_START;
	}
}

void st_deenergize_motors(){
	for (uint8_t motor = MOTOR_1; motor < MOTORS; motor++) {
		_deenergize_motor(motor);
	}
}

/*
 * st_motor_power_callback() - callback to manage motor power sequencing
 *
 *	Handles motor power-down timing, low-power idle, and adaptive motor power
 */
stat_t st_motor_power_callback() 	{
	// manage power for each motor individually
	for (uint8_t m = MOTOR_1; m < MOTORS; m++) {
		// de-energize motor if it's set to MOTOR_DISABLED
		if (st_cfg.mot[m].power_mode == MOTOR_DISABLED) {
			_deenergize_motor(m);
			continue;
		}
		// energize motor if it's set to MOTOR_ALWAYS_POWERED
		if (st_cfg.mot[m].power_mode == MOTOR_ALWAYS_POWERED) {
			if (! _motor_is_enabled(m)) _energize_motor(m);
			continue;
		}
		// start a countdown if MOTOR_POWERED_IN_CYCLE or MOTOR_POWERED_ONLY_WHEN_MOVING
		if (st_run.mot[m].power_state == MOTOR_POWER_TIMEOUT_START) {
			st_run.mot[m].power_state = MOTOR_POWER_TIMEOUT_COUNTDOWN;
			st_run.mot[m].power_systick = SysTickTimer_getValue() + (st_cfg.motor_power_timeout * 1000);
		}
		// do not process countdown if in a feedhold
		if (cm_get_combined_state() == COMBINED_HOLD) {
			continue;
		}
		// do not process countdown if in a feedhold
		if (cm_get_combined_state() == COMBINED_HOLD) {
			continue;
		}
		// run the countdown if you are in a countdown
		if (st_run.mot[m].power_state == MOTOR_POWER_TIMEOUT_COUNTDOWN) {
			if (SysTickTimer_getValue() > st_run.mot[m].power_systick ) {
				st_run.mot[m].power_state = MOTOR_IDLE;
				_deenergize_motor(m);
                sr_request_status_report(SR_TIMED_REQUEST);		// request a status report when motors shut down
			}
		}
	}
	return (STAT_OK);
}


/******************************
 * Interrupt Service Routines *
 ******************************/

/***** Stepper Interrupt Service Routine ************************************************
 *  ISR - DDA timer interrupt routine - service ticks from DDA timer
 *	Uses direct struct addresses and literal values for hardware devices - it's faster than
 *	using indexed timer and port accesses. I checked. Even when -0s or -03 is used.
 */
void IRQ_TIMER_DDA(){
	if ((st_run.mot[MOTOR_1].substep_accumulator += st_run.mot[MOTOR_1].substep_increment) > 0) {
		//--PORT_MOTOR_1_VPORT.OUT |= STEP_BIT_bm;		// turn step bit on
		HAL_GPIO_WritePin(STEPPERS_PORT,STEP_X_Pin, GPIO_PIN_SET);
		st_run.mot[MOTOR_1].substep_accumulator -= st_run.dda_ticks_X_substeps;
		INCREMENT_ENCODER(MOTOR_1);
	}
	if ((st_run.mot[MOTOR_2].substep_accumulator += st_run.mot[MOTOR_2].substep_increment) > 0) {
		//--PORT_MOTOR_2_VPORT.OUT |= STEP_BIT_bm;

		st_run.mot[MOTOR_2].substep_accumulator -= st_run.dda_ticks_X_substeps;
		INCREMENT_ENCODER(MOTOR_2);
	}
	if ((st_run.mot[MOTOR_3].substep_accumulator += st_run.mot[MOTOR_3].substep_increment) > 0) {
		//--PORT_MOTOR_3_VPORT.OUT |= STEP_BIT_bm;
		st_run.mot[MOTOR_3].substep_accumulator -= st_run.dda_ticks_X_substeps;
		INCREMENT_ENCODER(MOTOR_3);
	}
	if ((st_run.mot[MOTOR_4].substep_accumulator += st_run.mot[MOTOR_4].substep_increment) > 0) {
		//--PORT_MOTOR_4_VPORT.OUT |= STEP_BIT_bm;
		st_run.mot[MOTOR_4].substep_accumulator -= st_run.dda_ticks_X_substeps;
		INCREMENT_ENCODER(MOTOR_4);
	}

	// pulse stretching for using external drivers.- turn step bits off
	//--PORT_MOTOR_1_VPORT.OUT &= ~STEP_BIT_bm;				// ~ 5 uSec pulse width
	//--PORT_MOTOR_2_VPORT.OUT &= ~STEP_BIT_bm;				// ~ 4 uSec
	//--PORT_MOTOR_3_VPORT.OUT &= ~STEP_BIT_bm;				// ~ 3 uSec
	//--PORT_MOTOR_4_VPORT.OUT &= ~STEP_BIT_bm;				// ~ 2 uSec

	HAL_GPIO_WritePin(STEPPERS_PORT,STEP_X_Pin, GPIO_PIN_RESET);
	if (--st_run.dda_ticks_downcount != 0) return;
	TIM_STOP(TIM2);

	//--TIMER_DDA.CTRLA = STEP_TIMER_DISABLE;				// disable DDA timer
	_load_move();											// load the next move
}
/***** Dwell Interrupt Service Routine **************************************************
 * ISR - DDA timer interrupt routine - service ticks from DDA timer
 */

void IRQ_TIMER_DWELL() {								// DWELL timer interrupt
	if (--st_run.dda_ticks_downcount == 0) {
		TIM_STOP(TIM_DWELL);
		//--TIMER_DWELL.CTRLA = STEP_TIMER_DISABLE;			// disable DWELL timer
		_load_move();
	}
}

/****************************************************************************************
 * Exec sequencing code		- computes and prepares next load segment
 * st_request_exec_move()	- SW interrupt to request to execute a move
 * exec_timer interrupt		- interrupt handler for calling exec function
 */

void st_request_exec_move(){

	if (st_pre.buffer_state == PREP_BUFFER_OWNED_BY_EXEC) {// bother interrupting
		//printf("st_exec_move\n");
		TIM_EXEC->CR1 &=~TIM_CR1_CEN;	// Stop the timer
		TIM_EXEC->ARR = 200;
		TIM_EXEC->CNT = 0;            // Reset the timer
		TIM_EXEC->CR1 = TIM_CR1_CEN | TIM_CR1_ARPE; // Enable the timer
		//--TIMER_EXEC.PER = EXEC_TIMER_PERIOD;
		//--TIMER_EXEC.CTRLA = EXEC_TIMER_ENABLE;				// trigger a LO interrupt
	}
}

void IRQ_TIMER_EXEC() {								// exec move SW interrupt
	//--TIMER_EXEC.CTRLA = EXEC_TIMER_DISABLE;				// disable SW interrupt timer
	TIM_STOP(TIM_EXEC);
	// exec_move
	if (st_pre.buffer_state == PREP_BUFFER_OWNED_BY_EXEC) {
		if (mp_exec_move() != STAT_NOOP) {
			st_pre.buffer_state = PREP_BUFFER_OWNED_BY_LOADER; // flip it back
			//printf("load\r");
			_request_load_move();
		}
	}
}

/****************************************************************************************
 * Loader sequencing code
 * st_request_load_move() - fires a software interrupt (timer) to request to load a move
 * load_move interrupt	  - interrupt handler for running the loader
 *
 *	_load_move() can only be called be called from an ISR at the same or higher level as
 *	the DDA or dwell ISR. A software interrupt has been provided to allow a non-ISR to
 *	request a load (see st_request_load_move())
 */

void _request_load_move(){
	if (st_runtime_isbusy()) {
		return;													// don't request a load if the runtime is busy
	}

	if (st_pre.buffer_state == PREP_BUFFER_OWNED_BY_LOADER) {	// bother interrupting
		TIM_LOAD->CR1 &= ~TIM_CR1_CEN;// Stop the timer
		TIM_LOAD->ARR = 200;
		TIM_LOAD->CNT = 0;            // Reset the timer
		TIM_LOAD->CR1 |= TIM_CR1_CEN |TIM_CR1_ARPE ; // Enable the timer

		//--TIMER_LOAD.PER = LOAD_TIMER_PERIOD;
		//--TIMER_LOAD.CTRLA = LOAD_TIMER_ENABLE;					// trigger a HI interrupt
	}
}

void IRQ_TIMER_LOAD(){													// load steppers SW interrupt
	//--TIMER_LOAD.CTRLA = LOAD_TIMER_DISABLE;						// disable SW interrupt timer
	_load_move();
	TIM_STOP(TIM_LOAD);
}

/****************************************************************************************
 * _load_move() - Dequeue move and load into stepper struct
 *
 *	This routine can only be called be called from an ISR at the same or
 *	higher level as the DDA or dwell ISR. A software interrupt has been
 *	provided to allow a non-ISR to request a load (see st_request_load_move())
 *
 *	In aline() code:
 *	 - All axes must set steps and compensate for out-of-range pulse phasing.
 *	 - If axis has 0 steps the direction setting can be omitted
 *	 - If axis has 0 steps the motor must not be enabled to support power mode = 1
 */
/****** WARNING - THIS CODE IS SPECIFIC TO AVR. SEE G2 FOR ARM CODE ******/

void _load_move(){
	// Be aware that dda_ticks_downcount must equal zero for the loader to run.
	// So the initial load must also have this set to zero as part of initialization
	if (st_runtime_isbusy()) {
		return;													// exit if the runtime is busy
	}
	if (st_pre.buffer_state != PREP_BUFFER_OWNED_BY_LOADER) {	// if there are no moves to load...
//		for (uint8_t motor = MOTOR_1; motor < MOTORS; motor++) {
//			st_run.mot[motor].power_state = MOTOR_POWER_TIMEOUT_START;	// ...start motor power timeouts
//		}
		return;
	}
	// handle aline loads first (most common case)
	if (st_pre.move_type == MOVE_TYPE_ALINE) {

		//**** setup the new segment ****
		st_run.dda_ticks_downcount = st_pre.dda_ticks;
		st_run.dda_ticks_X_substeps = st_pre.dda_ticks_X_substeps;

		//**** MOTOR_1 LOAD ****
		// the following if() statement sets the runtime substep increment value or zeroes it
		if ((st_run.mot[MOTOR_1].substep_increment = st_pre.mot[MOTOR_1].substep_increment) != 0) {

			// NB: If motor has 0 steps the following is all skipped. This ensures that state comparisons
			//	   always operate on the last segment actually run by this motor, regardless of how many
			//	   segments it may have been inactive in between.

			// Apply accumulator correction if the time base has changed since previous segment
			if (st_pre.mot[MOTOR_1].accumulator_correction_flag == true) {
				st_pre.mot[MOTOR_1].accumulator_correction_flag = false;
				st_run.mot[MOTOR_1].substep_accumulator *= st_pre.mot[MOTOR_1].accumulator_correction;
			}

			// Detect direction change and if so:
			//	- Set the direction bit in hardware.
			//	- Compensate for direction change by flipping substep accumulator value about its midpoint.

			if (st_pre.mot[MOTOR_1].direction != st_pre.mot[MOTOR_1].prev_direction) {
				st_pre.mot[MOTOR_1].prev_direction = st_pre.mot[MOTOR_1].direction;
				st_run.mot[MOTOR_1].substep_accumulator = -(st_run.dda_ticks_X_substeps + st_run.mot[MOTOR_1].substep_accumulator);
				//--if (st_pre.mot[MOTOR_1].direction == DIRECTION_CW)
				//--PORT_MOTOR_1_VPORT.OUT &= ~DIRECTION_BIT_bm; else
				//--PORT_MOTOR_1_VPORT.OUT |= DIRECTION_BIT_bm;
			}
			SET_ENCODER_STEP_SIGN(MOTOR_1, st_pre.mot[MOTOR_1].step_sign);

			// Enable the stepper and start motor power management
			if (st_cfg.mot[MOTOR_1].power_mode != MOTOR_DISABLED) {
				//--PORT_MOTOR_1_VPORT.OUT &= ~MOTOR_ENABLE_BIT_bm;             // energize motor
				st_run.mot[MOTOR_1].power_state = MOTOR_POWER_TIMEOUT_START;// set power management state
			}

		} else {  // Motor has 0 steps; might need to energize motor for power mode processing
			if (st_cfg.mot[MOTOR_1].power_mode == MOTOR_POWERED_IN_CYCLE) {
				//--PORT_MOTOR_1_VPORT.OUT &= ~MOTOR_ENABLE_BIT_bm;             // energize motor
				st_run.mot[MOTOR_1].power_state = MOTOR_POWER_TIMEOUT_START;
			}
		}
		// accumulate counted steps to the step position and zero out counted steps for the segment currently being loaded
		ACCUMULATE_ENCODER(MOTOR_1);

#if (MOTORS >= 2)	//**** MOTOR_2 LOAD ****
		if ((st_run.mot[MOTOR_2].substep_increment = st_pre.mot[MOTOR_2].substep_increment) != 0) {
			if (st_pre.mot[MOTOR_2].accumulator_correction_flag == true) {
				st_pre.mot[MOTOR_2].accumulator_correction_flag = false;
				st_run.mot[MOTOR_2].substep_accumulator *= st_pre.mot[MOTOR_2].accumulator_correction;
			}
			if (st_pre.mot[MOTOR_2].direction != st_pre.mot[MOTOR_2].prev_direction) {
				st_pre.mot[MOTOR_2].prev_direction = st_pre.mot[MOTOR_2].direction;
				st_run.mot[MOTOR_2].substep_accumulator = -(st_run.dda_ticks_X_substeps + st_run.mot[MOTOR_2].substep_accumulator);
				//--if (st_pre.mot[MOTOR_2].direction == DIRECTION_CW)
				//--PORT_MOTOR_2_VPORT.OUT &= ~DIRECTION_BIT_bm; else
				//--PORT_MOTOR_2_VPORT.OUT |= DIRECTION_BIT_bm;
			}
			SET_ENCODER_STEP_SIGN(MOTOR_2, st_pre.mot[MOTOR_2].step_sign);
			if (st_cfg.mot[MOTOR_2].power_mode != MOTOR_DISABLED) {
				//--PORT_MOTOR_2_VPORT.OUT &= ~MOTOR_ENABLE_BIT_bm;
				st_run.mot[MOTOR_2].power_state = MOTOR_POWER_TIMEOUT_START;
			}
		} else {
			if (st_cfg.mot[MOTOR_2].power_mode == MOTOR_POWERED_IN_CYCLE) {
				//--PORT_MOTOR_2_VPORT.OUT &= ~MOTOR_ENABLE_BIT_bm;
				st_run.mot[MOTOR_2].power_state = MOTOR_POWER_TIMEOUT_START;
			}
		}
		ACCUMULATE_ENCODER(MOTOR_2);
#endif
#if (MOTORS >= 3)	//**** MOTOR_3 LOAD ****
		if ((st_run.mot[MOTOR_3].substep_increment = st_pre.mot[MOTOR_3].substep_increment) != 0) {
			if (st_pre.mot[MOTOR_3].accumulator_correction_flag == true) {
				st_pre.mot[MOTOR_3].accumulator_correction_flag = false;
				st_run.mot[MOTOR_3].substep_accumulator *= st_pre.mot[MOTOR_3].accumulator_correction;
			}
			if (st_pre.mot[MOTOR_3].direction != st_pre.mot[MOTOR_3].prev_direction) {
				st_pre.mot[MOTOR_3].prev_direction = st_pre.mot[MOTOR_3].direction;
				st_run.mot[MOTOR_3].substep_accumulator = -(st_run.dda_ticks_X_substeps + st_run.mot[MOTOR_3].substep_accumulator);
				//--if (st_pre.mot[MOTOR_3].direction == DIRECTION_CW)
				//--PORT_MOTOR_3_VPORT.OUT &= ~DIRECTION_BIT_bm; else
				//--PORT_MOTOR_3_VPORT.OUT |= DIRECTION_BIT_bm;
			}
			SET_ENCODER_STEP_SIGN(MOTOR_3, st_pre.mot[MOTOR_3].step_sign);
			if (st_cfg.mot[MOTOR_3].power_mode != MOTOR_DISABLED) {
				//--PORT_MOTOR_3_VPORT.OUT &= ~MOTOR_ENABLE_BIT_bm;
				st_run.mot[MOTOR_3].power_state = MOTOR_POWER_TIMEOUT_START;
			}
		} else {
			if (st_cfg.mot[MOTOR_3].power_mode == MOTOR_POWERED_IN_CYCLE) {
				//--PORT_MOTOR_3_VPORT.OUT &= ~MOTOR_ENABLE_BIT_bm;
				st_run.mot[MOTOR_3].power_state = MOTOR_POWER_TIMEOUT_START;
			}
		}
		ACCUMULATE_ENCODER(MOTOR_3);
#endif
#if (MOTORS >= 4)  //**** MOTOR_4 LOAD ****
		if ((st_run.mot[MOTOR_4].substep_increment = st_pre.mot[MOTOR_4].substep_increment) != 0) {
			if (st_pre.mot[MOTOR_4].accumulator_correction_flag == true) {
				st_pre.mot[MOTOR_4].accumulator_correction_flag = false;
				st_run.mot[MOTOR_4].substep_accumulator *= st_pre.mot[MOTOR_4].accumulator_correction;
			}
			if (st_pre.mot[MOTOR_4].direction != st_pre.mot[MOTOR_4].prev_direction) {
				st_pre.mot[MOTOR_4].prev_direction = st_pre.mot[MOTOR_4].direction;
				st_run.mot[MOTOR_4].substep_accumulator = -(st_run.dda_ticks_X_substeps + st_run.mot[MOTOR_4].substep_accumulator);
				//--if (st_pre.mot[MOTOR_4].direction == DIRECTION_CW)
				//--PORT_MOTOR_4_VPORT.OUT &= ~DIRECTION_BIT_bm; else
				//--PORT_MOTOR_4_VPORT.OUT |= DIRECTION_BIT_bm;
			}
			SET_ENCODER_STEP_SIGN(MOTOR_4, st_pre.mot[MOTOR_4].step_sign);
			if (st_cfg.mot[MOTOR_4].power_mode != MOTOR_DISABLED) {
				//--PORT_MOTOR_4_VPORT.OUT &= ~MOTOR_ENABLE_BIT_bm;
				st_run.mot[MOTOR_4].power_state = MOTOR_POWER_TIMEOUT_START;
			}
		} else {
			if (st_cfg.mot[MOTOR_4].power_mode == MOTOR_POWERED_IN_CYCLE) {
				//--PORT_MOTOR_4_VPORT.OUT &= ~MOTOR_ENABLE_BIT_bm;
				st_run.mot[MOTOR_4].power_state = MOTOR_POWER_TIMEOUT_START;
			}
		}
		ACCUMULATE_ENCODER(MOTOR_4);
#endif
#if (MOTORS >= 5)	//**** MOTOR_5 LOAD ****
		if ((st_run.mot[MOTOR_5].substep_increment = st_pre.mot[MOTOR_5].substep_increment) != 0) {
			if (st_pre.mot[MOTOR_5].accumulator_correction_flag == true) {
				st_pre.mot[MOTOR_5].accumulator_correction_flag = false;
				st_run.mot[MOTOR_5].substep_accumulator *= st_pre.mot[MOTOR_5].accumulator_correction;
			}
			if (st_pre.mot[MOTOR_5].direction != st_pre.mot[MOTOR_5].prev_direction) {
				st_pre.mot[MOTOR_5].prev_direction = st_pre.mot[MOTOR_5].direction;
				st_run.mot[MOTOR_5].substep_accumulator = -(st_run.dda_ticks_X_substeps + st_run.mot[MOTOR_5].substep_accumulator);
				if (st_pre.mot[MOTOR_5].direction == DIRECTION_CW)
				PORT_MOTOR_5_VPORT.OUT &= ~DIRECTION_BIT_bm; else
				PORT_MOTOR_5_VPORT.OUT |= DIRECTION_BIT_bm;
			}
			PORT_MOTOR_5_VPORT.OUT &= ~MOTOR_ENABLE_BIT_bm;
			st_run.mot[MOTOR_5].power_state = MOTOR_POWER_TIMEOUT_START;
			SET_ENCODER_STEP_SIGN(MOTOR_5, st_pre.mot[MOTOR_5].step_sign);
		} else {
			if (st_cfg.mot[MOTOR_5].power_mode == MOTOR_POWERED_IN_CYCLE) {
				PORT_MOTOR_5_VPORT.OUT &= ~MOTOR_ENABLE_BIT_bm;
				st_run.mot[MOTOR_5].power_state = MOTOR_POWER_TIMEOUT_START;
			}
		}
		ACCUMULATE_ENCODER(MOTOR_5);
#endif
#if (MOTORS >= 6)	//**** MOTOR_6 LOAD ****
		if ((st_run.mot[MOTOR_6].substep_increment = st_pre.mot[MOTOR_6].substep_increment) != 0) {
			if (st_pre.mot[MOTOR_6].accumulator_correction_flag == true) {
				st_pre.mot[MOTOR_6].accumulator_correction_flag = false;
				st_run.mot[MOTOR_6].substep_accumulator *= st_pre.mot[MOTOR_6].accumulator_correction;
			}
			if (st_pre.mot[MOTOR_6].direction != st_pre.mot[MOTOR_6].prev_direction) {
				st_pre.mot[MOTOR_6].prev_direction = st_pre.mot[MOTOR_6].direction;
				st_run.mot[MOTOR_6].substep_accumulator = -(st_run.dda_ticks_X_substeps + st_run.mot[MOTOR_6].substep_accumulator);
				if (st_pre.mot[MOTOR_6].direction == DIRECTION_CW)
				PORT_MOTOR_6_VPORT.OUT &= ~DIRECTION_BIT_bm; else
				PORT_MOTOR_6_VPORT.OUT |= DIRECTION_BIT_bm;
			}
			PORT_MOTOR_6_VPORT.OUT &= ~MOTOR_ENABLE_BIT_bm;
			st_run.mot[MOTOR_6].power_state = MOTOR_POWER_TIMEOUT_START;
			SET_ENCODER_STEP_SIGN(MOTOR_6, st_pre.mot[MOTOR_6].step_sign);
		} else {
			if (st_cfg.mot[MOTOR_6].power_mode == MOTOR_POWERED_IN_CYCLE) {
				PORT_MOTOR_6_VPORT.OUT &= ~MOTOR_ENABLE_BIT_bm;
				st_run.mot[MOTOR_6].power_state = MOTOR_POWER_TIMEOUT_START;
			}
		}
		ACCUMULATE_ENCODER(MOTOR_6);
#endif
		//**** do this last ****
		//printf("d %d\r\n",(uint16_t)st_pre.dda_period);
		//--TIMER_DDA.PER = st_pre.dda_period;
		//--TIMER_DDA.CTRLA = STEP_TIMER_ENABLE;			// enable the DDA timer
	    TIM_DDA->CR1 &= ~TIM_CR1_CEN;// Stop the timer
	    TIM_DDA->ARR = st_pre.dda_period-1;
	    TIM_DDA->CNT = 0;            // Reset the timer
	    TIM_DDA->CR1 |= TIM_CR1_CEN | TIM_CR1_ARPE; // Enable the timer


	// handle dwells
	} else if (st_pre.move_type == MOVE_TYPE_DWELL) {
		st_run.dda_ticks_downcount = st_pre.dda_ticks;

	    TIM_DWELL->CR1 &= ~TIM_CR1_CEN;// Stop the timer
	    TIM_DWELL->ARR = st_pre.dda_period-1;
	    TIM_DWELL->CNT = 0;            // Reset the timer
	    TIM_DWELL->CR1 |= TIM_CR1_CEN | TIM_CR1_ARPE; // Enable the timer

		//--TIMER_DWELL.PER = st_pre.dda_period;			// load dwell timer period
		//--TIMER_DWELL.CTRLA = STEP_TIMER_ENABLE;			// enable the dwell timer
	// handle synchronous commands
	} else if (st_pre.move_type == MOVE_TYPE_COMMAND) {
		mp_runtime_command(st_pre.bf);
	}

	// all other cases drop to here (e.g. Null moves after Mcodes skip to here)
	st_pre.move_type = MOVE_TYPE_NULL;
	st_pre.buffer_state = PREP_BUFFER_OWNED_BY_EXEC;	// we are done with the prep buffer - flip the flag back
	st_request_exec_move();								// exec and prep next move
}

/***********************************************************************************
 * st_prep_line() - Prepare the next move for the loader
 *
 *	This function does the math on the next pulse segment and gets it ready for
 *	the loader. It deals with all the DDA optimizations and timer setups so that
 *	loading can be performed as rapidly as possible. It works in joint space
 *	(motors) and it works in steps, not length units. All args are provided as
 *	floats and converted to their appropriate integer types for the loader.
 *
 * Args:
 *	  - travel_steps[] are signed relative motion in steps for each motor. Steps are
 *		floats that typically have fractional values (fractional steps). The sign
 *		indicates direction. Motors that are not in the move should be 0 steps on input.
 *
 *	  - following_error[] is a vector of measured errors to the step count. Used for correction.
 *
 *	  - segment_time - how many minutes the segment should run. If timing is not
 *		100% accurate this will affect the move velocity, but not the distance traveled.
 *
 * NOTE:  Many of the expressions are sensitive to casting and execution order to avoid long-term
 *		  accuracy errors due to floating point round off. One earlier failed attempt was:
 *		    dda_ticks_X_substeps = (int32_t)((microseconds/1000000) * f_dda * dda_substeps);
 */

stat_t st_prep_line(float travel_steps[], float following_error[], float segment_time){
	//printf("st_prep_line\n");
	// trap conditions that would prevent queueing the line
	if (st_pre.buffer_state != PREP_BUFFER_OWNED_BY_EXEC) { return (cm_hard_alarm(STAT_INTERNAL_ERROR));
	} else if (isinf(segment_time)) { 	return (cm_hard_alarm(STAT_PREP_LINE_MOVE_TIME_IS_INFINITE));	// never supposed to happen
	} else if (isnan(segment_time)) { 	return (cm_hard_alarm(STAT_PREP_LINE_MOVE_TIME_IS_NAN));		// never supposed to happen
	} else if (segment_time < EPSILON){ return (STAT_MINIMUM_TIME_MOVE); }
	// setup segment parameters
	// - dda_ticks is the integer number of DDA clock ticks needed to play out the segment
	// - ticks_X_substeps is the maximum depth of the DDA accumulator (as a negative number)

	st_pre.dda_period = _f_to_period(FREQUENCY_DDA);
	st_pre.dda_ticks = (int32_t)(segment_time * 60 * FREQUENCY_DDA);// NB: converts minutes to seconds
	st_pre.dda_ticks_X_substeps = st_pre.dda_ticks * DDA_SUBSTEPS;

	// setup motor parameters

	float correction_steps;
	for (uint8_t motor=0; motor<MOTORS; motor++) {	// I want to remind myself that this is motors, not axes

		// Skip this motor if there are no new steps. Leave all other values intact.
		if (fp_ZERO(travel_steps[motor])) { st_pre.mot[motor].substep_increment = 0; continue;}

		// Setup the direction, compensating for polarity.
		// Set the step_sign which is used by the stepper ISR to accumulate step position

		if (travel_steps[motor] >= 0) {					// positive direction
			st_pre.mot[motor].direction = DIRECTION_CW ^ st_cfg.mot[motor].polarity;
			st_pre.mot[motor].step_sign = 1;
		} else {
			st_pre.mot[motor].direction = DIRECTION_CCW ^ st_cfg.mot[motor].polarity;
			st_pre.mot[motor].step_sign = -1;
		}

		// Detect segment time changes and setup the accumulator correction factor and flag.
		// Putting this here computes the correct factor even if the motor was dormant for some
		// number of previous moves. Correction is computed based on the last segment time actually used.

		if (fabs(segment_time - st_pre.mot[motor].prev_segment_time) > 0.0000001) { // highly tuned FP != compare
			if (fp_NOT_ZERO(st_pre.mot[motor].prev_segment_time)) {					// special case to skip first move
				st_pre.mot[motor].accumulator_correction_flag = true;
				st_pre.mot[motor].accumulator_correction = segment_time / st_pre.mot[motor].prev_segment_time;
			}
			st_pre.mot[motor].prev_segment_time = segment_time;
		}

#ifdef __STEP_CORRECTION
		// 'Nudge' correction strategy. Inject a single, scaled correction value then hold off

		if ((--st_pre.mot[motor].correction_holdoff < 0) && (fabs(following_error[motor]) > STEP_CORRECTION_THRESHOLD)) {

			st_pre.mot[motor].correction_holdoff = STEP_CORRECTION_HOLDOFF;
			correction_steps = following_error[motor] * STEP_CORRECTION_FACTOR;

			if (correction_steps > 0) {
				correction_steps = min3(correction_steps, fabs(travel_steps[motor]), STEP_CORRECTION_MAX);
			} else {
				correction_steps = max3(correction_steps, -fabs(travel_steps[motor]), -STEP_CORRECTION_MAX);
			}
			st_pre.mot[motor].corrected_steps += correction_steps;
			travel_steps[motor] -= correction_steps;
		}
#endif
		// Compute substeb increment. The accumulator must be *exactly* the incoming
		// fractional steps times the substep multiplier or positional drift will occur.
		// Rounding is performed to eliminate a negative bias in the uint32 conversion
		// that results in long-term negative drift. (fabs/round order doesn't matter)

		st_pre.mot[motor].substep_increment = round(fabs(travel_steps[motor] * DDA_SUBSTEPS));
	}
	st_pre.move_type = MOVE_TYPE_ALINE;
	st_pre.buffer_state = PREP_BUFFER_OWNED_BY_LOADER;	// signal that prep buffer is ready
	return (STAT_OK);
}

/*
 * st_prep_null() - Keeps the loader happy. Otherwise performs no action
 */

void st_prep_null(){
	st_pre.move_type 	= MOVE_TYPE_NULL;
	st_pre.buffer_state = PREP_BUFFER_OWNED_BY_EXEC;	// signal that prep buffer is empty
}

/*
 * st_prep_command() - Stage command to execution
 */

void st_prep_command(void *bf){
	st_pre.move_type = MOVE_TYPE_COMMAND;
	st_pre.bf = (mpBuf_t *)bf;
	st_pre.buffer_state = PREP_BUFFER_OWNED_BY_LOADER;	// signal that prep buffer is ready
}

/*
 * st_prep_dwell() 	 - Add a dwell to the move buffer
 */

void st_prep_dwell(float microseconds){
	//printf("st_prep_dwell %2.5f\n",microseconds);
	st_pre.move_type 	= MOVE_TYPE_DWELL;
	st_pre.dda_period 	= _f_to_period(FREQUENCY_DWELL);
	st_pre.dda_ticks 	= (uint32_t)(max(1, (microseconds/1000000) * FREQUENCY_DWELL)); // Make sure it is positive.
	st_pre.buffer_state = PREP_BUFFER_OWNED_BY_LOADER;	// signal that prep buffer is ready
}

/*
 * _set_hw_microsteps() - set microsteps in hardware
 *
 *	For now the microsteps is the same as the microsteps (1,2,4,8)
 *	This may change if microstep morphing is implemented.
 */

static void _set_hw_microsteps(const uint8_t motor, const uint8_t microsteps){

}


/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 ***********************************************************************************/

/* HELPERS
 * _get_motor() - helper to return motor number as an index
 */

static int8_t _get_motor(const nvObj_t *nv){
    return ((nv->group[0] ? nv->group[0] : nv->token[0]) - 0x31);
}

/*
 * _set_motor_steps_per_unit() - what it says
 * This function will need to be rethought if microstep morphing is implemented
 */

static void _set_motor_steps_per_unit(nvObj_t *nv){
	uint8_t m = _get_motor(nv);
//	st_cfg.mot[m].units_per_step = (st_cfg.mot[m].travel_rev * st_cfg.mot[m].step_angle) / (360 * st_cfg.mot[m].microsteps); // unused
    st_cfg.mot[m].steps_per_unit = (360 * st_cfg.mot[m].microsteps) / (st_cfg.mot[m].travel_rev * st_cfg.mot[m].step_angle);
	st_reset();
}

/* PER-MOTOR FUNCTIONS
 * st_set_sa() - set motor step angle
 * st_set_tr() - set travel per motor revolution
 * st_set_mi() - set motor microsteps
 * st_set_pm() - set motor power mode
 * st_set_pl() - set motor power level
 */
// motor step angle
stat_t st_set_sa(nvObj_t *nv)			{
	set_flt(nv);
	_set_motor_steps_per_unit(nv);
	return(STAT_OK);
}
// motor travel per revolution
stat_t st_set_tr(nvObj_t *nv)			{
	set_flu(nv);
	_set_motor_steps_per_unit(nv);
	return(STAT_OK);
}
// motor microsteps
stat_t st_set_mi(nvObj_t *nv)			{
    uint32_t mi = (uint32_t)nv->value;
	if ((mi != 1) && (mi != 2) && (mi != 4) && (mi != 8)) {
		nv_add_conditional_message((const char_t *)"*** WARNING *** Setting non-standard microstep value");
	}
	set_int(nv);						// set it anyway, even if it's unsupported. It could also be > 255
	_set_motor_steps_per_unit(nv);
	_set_hw_microsteps(_get_motor(nv), (uint8_t)nv->value);
	return (STAT_OK);
}
// motor power mode
stat_t st_set_pm(nvObj_t *nv)			{
	if ((uint8_t)nv->value >= MOTOR_POWER_MODE_MAX_VALUE)
        return (STAT_INPUT_VALUE_RANGE_ERROR);
	set_ui8(nv);
	return (STAT_OK);
	// NOTE: The motor power callback makes these settings take effect immediately
}

/*
 * st_set_pl() - set motor power level
 *
 *	Input value may vary from 0.000 to 1.000 The setting is scaled to allowable PWM range.
 *	This function sets both the scaled and dynamic power levels, and applies the
 *	scaled value to the vref.
 */
// motor power level
stat_t st_set_pl(nvObj_t *nv)	{
#ifdef __ARM
	if (nv->value < (float)0.0) nv->value = 0.0;
	if (nv->value > (float)1.0) {
		if (nv->value > (float)100) nv->value = 1;
 		nv->value /= 100;		// accommodate old 0-100 inputs
	}
	set_flt(nv);	// set power_setting value in the motor config struct (st)

	uint8_t m = _get_motor(nv);
	st_cfg.mot[m].power_level_scaled = (nv->value * POWER_LEVEL_SCALE_FACTOR);
	st_run.mot[m].power_level_dynamic = (st_cfg.mot[m].power_level_scaled);
	_set_motor_power_level(m, st_cfg.mot[m].power_level_scaled);
#endif
	return(STAT_OK);
}

/*
 * st_get_pwr()	- get motor enable power state
 */
stat_t st_get_pwr(nvObj_t *nv){
	nv->value = _motor_is_enabled(_get_motor(nv));
	nv->valuetype = TYPE_INTEGER;
	return (STAT_OK);
}

/* GLOBAL FUNCTIONS (SYSTEM LEVEL)
 *
 * st_set_mt() - set motor timeout in seconds
 * st_set_md() - disable motor power
 * st_set_me() - enable motor power
 *
 * Calling me or md with NULL will enable or disable all motors
 * Setting a value of 0 will enable or disable all motors
 * Setting a value from 1 to MOTORS will enable or disable that motor only
 */

stat_t st_set_mt(nvObj_t *nv){
	st_cfg.motor_power_timeout = min(MOTOR_TIMEOUT_SECONDS_MAX, max(nv->value, MOTOR_TIMEOUT_SECONDS_MIN));
	return (STAT_OK);
}
// Make sure this function is not part of initialization --> f00
stat_t st_set_md(nvObj_t *nv)	{
	if (((uint8_t)nv->value == 0) || (nv->valuetype == TYPE_NULL)) {
		st_deenergize_motors();
	} else {
        uint8_t motor = (uint8_t)nv->value;
        if (motor > MOTORS) {
            return (STAT_INPUT_VALUE_RANGE_ERROR);
        }
        _deenergize_motor(motor-1);     // adjust so that motor 1 is actually 0 (etc)
	}
	return (STAT_OK);
}

stat_t st_set_me(nvObj_t *nv)	// Make sure this function is not part of initialization --> f00
{
	if (((uint8_t)nv->value == 0) || (nv->valuetype == TYPE_NULL)) {
		st_energize_motors();
	} else {
        uint8_t motor = (uint8_t)nv->value;
        if (motor > MOTORS) {
            return (STAT_INPUT_VALUE_RANGE_ERROR);
        }
		_energize_motor(motor-1);     // adjust so that motor 1 is actually 0 (etc)
	}
	return (STAT_OK);
}

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

static const char msg_units0[] = " in";	// used by generic print functions
static const char msg_units1[] = " mm";
static const char msg_units2[] = " deg";
static const char *const msg_units[] = { msg_units0, msg_units1, msg_units2 };
#define DEGREE_INDEX 2

static const char fmt_me[]  = "motors energized\n";
static const char fmt_md[]  = "motors de-energized\n";
static const char fmt_mt[]  = "[mt]  motor idle timeout%14.2f Sec\n";
static const char fmt_0ma[] = "[%s%s] m%s map to axis%15d [0=X,1=Y,2=Z...]\n";
static const char fmt_0sa[] = "[%s%s] m%s step angle%20.3f%s\n";
static const char fmt_0tr[] = "[%s%s] m%s travel per revolution%10.4f%s\n";
static const char fmt_0mi[] = "[%s%s] m%s microsteps%16d [1,2,4,8]\n";
static const char fmt_0po[] = "[%s%s] m%s polarity%18d [0=normal,1=reverse]\n";
static const char fmt_0pm[] = "[%s%s] m%s power management%10d [0=disabled,1=always on,2=in cycle,3=when moving]\n";
static const char fmt_0pl[] = "[%s%s] m%s motor power level%13.3f [0.000=min, 1.000=max]\n";
static const char fmt_pwr[] = "Motor %c power enabled state:%2.0f\n";

void st_print_mt(nvObj_t *nv) { text_print_flt(nv, fmt_mt);}
void st_print_me(nvObj_t *nv) { text_print_nul(nv, fmt_me);}
void st_print_md(nvObj_t *nv) { text_print_nul(nv, fmt_md);}

static void _print_motor_ui8(nvObj_t *nv, const char *format){
	fprintf(stderr, format, nv->group, nv->token, nv->group, (uint8_t)nv->value);
}

static void _print_motor_flt_units(nvObj_t *nv, const char *format, uint8_t units){
	fprintf(stderr, format, nv->group, nv->token, nv->group, nv->value, GET_TEXT_ITEM(msg_units, units));
}

static void _print_motor_flt(nvObj_t *nv, const char *format){
	fprintf(stderr, format, nv->group, nv->token, nv->group, nv->value);
}

static void _print_motor_pwr(nvObj_t *nv, const char *format){
	fprintf(stderr, format, nv->token[0], nv->value);
}

void st_print_ma(nvObj_t *nv) { _print_motor_ui8(nv, fmt_0ma);}
void st_print_sa(nvObj_t *nv) { _print_motor_flt_units(nv, fmt_0sa, DEGREE_INDEX);}
void st_print_tr(nvObj_t *nv) { _print_motor_flt_units(nv, fmt_0tr, cm_get_units_mode(MODEL));}
void st_print_mi(nvObj_t *nv) { _print_motor_ui8(nv, fmt_0mi);}
void st_print_po(nvObj_t *nv) { _print_motor_ui8(nv, fmt_0po);}
void st_print_pm(nvObj_t *nv) { _print_motor_ui8(nv, fmt_0pm);}
void st_print_pl(nvObj_t *nv) { _print_motor_flt(nv, fmt_0pl);}
void st_print_pwr(nvObj_t *nv){ _print_motor_pwr(nv, fmt_pwr);}

#endif // __TEXT_MODE
