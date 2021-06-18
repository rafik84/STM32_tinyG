#ifndef gpio_h
#define gpio_h

void IndicatorLed_set(void);
void IndicatorLed_clear(void);
void IndicatorLed_toggle(void);

void gpio_led_on(uint8_t led);
void gpio_led_off(uint8_t led);
void gpio_led_toggle(uint8_t led);

uint8_t gpio_read_bit(uint8_t b);
void gpio_set_bit_on(uint8_t b);
void gpio_set_bit_off(uint8_t b);

#endif
