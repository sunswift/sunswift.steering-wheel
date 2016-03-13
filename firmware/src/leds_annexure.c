#include <scandal/types.h>

#include <arch/gpio.h>

#include <project/target_config.h>
#include <project/leds_annexure.h>

void precharge_led(u08 on) {
	if (on)
		GPIO_SetValue(PRCH_LED_PORT, PRCH_LED_BIT, 0);
	else
		GPIO_SetValue(PRCH_LED_PORT, PRCH_LED_BIT, 1);
}

void toggle_precharge_led(void) {
	GPIO_ToggleValue(PRCH_LED_PORT, PRCH_LED_BIT);
}

void reverse_led(u08 on) {
	if (on)
		GPIO_SetValue(REV_LED_PORT, REV_LED_BIT, 0);
	else
		GPIO_SetValue(REV_LED_PORT, REV_LED_BIT, 1);
}

void toggle_reverse_led(void) {
	GPIO_ToggleValue(REV_LED_PORT, REV_LED_BIT);
}

void cruise_led(u08 on) {
	if (on)
		GPIO_SetValue(CRUISE_LED_PORT, CRUISE_LED_BIT, 0);
	else
		GPIO_SetValue(CRUISE_LED_PORT, CRUISE_LED_BIT, 1);
}

void toggle_cruise_led(void) {
	GPIO_ToggleValue(CRUISE_LED_PORT, CRUISE_LED_BIT);
}

