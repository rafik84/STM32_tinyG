#include "tinyg.h"		// #1
#include "config.h"		// #2
#include "hardware.h"
#include "text_parser.h"
#include "gpio.h"
#include "pwm.h"


/***** PWM defines, structures and memory allocation *****/

pwmSingleton_t pwm;

// defines common to all PWM channels
//#define PWM_TIMER_TYPE	TC1_struct	// PWM uses TC1's
#define PWM_TIMER_t	TC1_t				// PWM uses TC1's
#define PWM_TIMER_DISABLE 0				// turn timer off (clock = 0 Hz)
#define PWM_MAX_FREQ (F_CPU/256)		// max frequency with 8-bits duty cycle precision
#define PWM_MIN_FREQ (F_CPU/64/65536)	// min frequency with supported prescaling

// channel specific defines
/* CLKSEL is used to configure default PWM clock operating ranges
 * These can be changed by pwm_freq() depending on the PWM frequency selected
 *
 * The useful ranges (assuming a 32 Mhz system clock) are:
 *	 TC_CLKSEL_DIV1_gc  - good for about 500 Hz to 125 Khz practical upper limit
 *   TC_CLKSEL_DIV2_gc  - good for about 250 Hz to  62 KHz
 *	 TC_CLKSEL_DIV4_gc  - good for about 125 Hz to  31 KHz
 *	 TC_CLKSEL_DIV8_gc  - good for about  62 Hz to  16 KHz
 *	 TC_CLKSEL_DIV64_gc - good for about   8 Hz to   2 Khz
 */
				// timer interrupt level (0=off, 1=lo, 2=med, 3=hi)

/***** PWM code *****/
/*
 * pwm_init() - initialize pwm channels
 *
 *	Notes:
 *	  - Whatever level interrupts you use must be enabled in main()
 *	  - init assumes PWM1 output bit (D5) has been set to output previously (stepper.c)
 *	  - See system.h for timer and port assignments
 *    - Don't do this: memset(&TIMER_PWM1, 0, sizeof(PWM_TIMER_t)); // zero out the timer registers
 */
void pwm_init()
{

}




stat_t pwm_set_freq(uint8_t chan, float freq)
{
	if (chan > PWMS) { return (STAT_NO_SUCH_DEVICE);}
	if (freq > PWM_MAX_FREQ) { return (STAT_INPUT_EXCEEDS_MAX_VALUE);}
	if (freq < PWM_MIN_FREQ) { return (STAT_INPUT_LESS_THAN_MIN_VALUE);}
	return (STAT_OK);
}

/*
 * pwm_set_duty() - set PWM channel duty cycle
 *
 *	channel	- PWM channel
 *	duty	- PWM duty cycle from 0% to 100%
 *
 *	Setting duty cycle to 0 disables the PWM channel with output low
 *	Setting duty cycle to 100 disables the PWM channel with output high
 *	Setting duty cycle between 0 and 100 enables PWM channel
 *
 *	The frequency must have been set previously
 */

stat_t pwm_set_duty(uint8_t chan, float duty)
{
	if (duty < 0.0) { return (STAT_INPUT_LESS_THAN_MIN_VALUE);}
	if (duty > 1.0) { return (STAT_INPUT_EXCEEDS_MAX_VALUE);}

	return (STAT_OK);
}

#ifdef __TEXT_MODE

static const char fmt_p1frq[] PROGMEM = "[p1frq] pwm frequency   %15.0f Hz\n";
static const char fmt_p1csl[] PROGMEM = "[p1csl] pwm cw speed lo %15.0f RPM\n";
static const char fmt_p1csh[] PROGMEM = "[p1csh] pwm cw speed hi %15.0f RPM\n";
static const char fmt_p1cpl[] PROGMEM = "[p1cpl] pwm cw phase lo %15.3f [0..1]\n";
static const char fmt_p1cph[] PROGMEM = "[p1cph] pwm cw phase hi %15.3f [0..1]\n";
static const char fmt_p1wsl[] PROGMEM = "[p1wsl] pwm ccw speed lo%15.0f RPM\n";
static const char fmt_p1wsh[] PROGMEM = "[p1wsh] pwm ccw speed hi%15.0f RPM\n";
static const char fmt_p1wpl[] PROGMEM = "[p1wpl] pwm ccw phase lo%15.3f [0..1]\n";
static const char fmt_p1wph[] PROGMEM = "[p1wph] pwm ccw phase hi%15.3f [0..1]\n";
static const char fmt_p1pof[] PROGMEM = "[p1pof] pwm phase off   %15.3f [0..1]\n";

void pwm_print_p1frq(nvObj_t *nv) { text_print_flt(nv, fmt_p1frq);}
void pwm_print_p1csl(nvObj_t *nv) { text_print_flt(nv, fmt_p1csl);}
void pwm_print_p1csh(nvObj_t *nv) { text_print_flt(nv, fmt_p1csh);}
void pwm_print_p1cpl(nvObj_t *nv) { text_print_flt(nv, fmt_p1cpl);}
void pwm_print_p1cph(nvObj_t *nv) { text_print_flt(nv, fmt_p1cph);}
void pwm_print_p1wsl(nvObj_t *nv) { text_print_flt(nv, fmt_p1wsl);}
void pwm_print_p1wsh(nvObj_t *nv) { text_print_flt(nv, fmt_p1wsh);}
void pwm_print_p1wpl(nvObj_t *nv) { text_print_flt(nv, fmt_p1wpl);}
void pwm_print_p1wph(nvObj_t *nv) { text_print_flt(nv, fmt_p1wph);}
void pwm_print_p1pof(nvObj_t *nv) { text_print_flt(nv, fmt_p1pof);}

#endif //__TEXT_MODE

#ifdef __cplusplus
}
#endif
