#include <scandal/types.h>

#define left_led(x)  yellow_led(x)
#define right_led(x) red_led(x)
#define toggle_left_led()  toggle_yellow_led()
#define toggle_right_led() toggle_red_led()

void precharge_led(u08 on);
void toggle_precharge_led(void);

void reverse_led(u08 on);
void toggle_reverse_led(void);

void cruise_led(u08 on);
void toggle_cruise_led(void);

