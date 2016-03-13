#include <arch/gpio.h>

#include <scandal/devices.h>

/* LED defines */
#define PRCH_LED_PORT   PORT2
#define PRCH_LED_BIT    9

#define CRUISE_LED_PORT PORT1
#define CRUISE_LED_BIT  1

#define REV_LED_PORT    PORT3
#define REV_LED_BIT     1

#define RIGHT_LED_PORT  PORT2
#define RIGHT_LED_BIT   10

#define LEFT_LED_PORT   PORT3
#define LEFT_LED_BIT    0

#define BRAKE_PORT      PORT2
#define BRAKE_BIT       8

/* lets just use the red/yellow led stuff for the left and right leds, see leds_annexure.h */
#define RED_LED_PORT     RIGHT_LED_PORT
#define RED_LED_BIT      RIGHT_LED_BIT
#define YELLOW_LED_PORT  LEFT_LED_PORT
#define YELLOW_LED_BIT   LEFT_LED_BIT

/* Switch defines */
#define HORN_SWITCH_PORT PORT2
#define HORN_SWITCH_BIT  3

#define HAZARDS_SWITCH_PORT PORT1
#define HAZARDS_SWITCH_BIT  4

#define INDICATORS_OFF_SWITCH_PORT PORT3
#define INDICATORS_OFF_SWITCH_BIT  3

#define RIGHT_INDICATOR_SWITCH_PORT PORT3
#define RIGHT_INDICATOR_SWITCH_BIT  2

#define LEFT_INDICATOR_SWITCH_PORT PORT1
#define LEFT_INDICATOR_SWITCH_BIT  5

#define SPEED_UP_SWITCH_PORT PORT0
#define SPEED_UP_SWITCH_BIT 11

#define SPEED_HOLD_SWITCH_PORT PORT2
#define SPEED_HOLD_SWITCH_BIT 11

#define FUNCY2_SWITCH_PORT PORT1
#define FUNCY2_SWITCH_BIT 10

#define FUNCY3_SWITCH_PORT PORT0
#define FUNCY3_SWITCH_BIT 8

#define SPEED_DOWN_SWITCH_PORT PORT2
#define SPEED_DOWN_SWITCH_BIT 2

#define FWD_SWITCH_PORT PORT0
#define FWD_SWITCH_BIT 7

#define PRECHARGE_SWITCH_PORT PORT0
#define PRECHARGE_SWITCH_BIT 6

#define REAR_VISION_SWITCH_PORT PORT2
#define REAR_VISION_SWITCH_BIT 5

#define FUNCY1_SWITCH_PORT PORT2
#define FUNCY1_SWITCH_BIT 4

#define REV_SWITCH_PORT PORT1
#define REV_SWITCH_BIT 9

/* Other ports */

#define LEFT_PADDLE_PORT  1
#define LEFT_PADDLE_BIT   2

#define RIGHT_PADDLE_PORT 1
#define RIGHT_PADDLE_BIT  0

/* Other defines */

#define REGEN_ADC_CHANNEL       3
#define ACCELERATOR_ADC_CHANNEL 1

/* this is twice as fast as the recommended 250ms interval for wavesculptor drive commands */
#define WS_TIME_BETWEEN_DRIVE_COMMANDS_MS 125

#define PRECHARGE_SWITCH_HOLD_TIME_MS     3000
#define PRECHARGE_DISCHARGE_TIMEOUT_MS    30000
#define REVERSE_SWITCH_HOLD_TIME_MS       3000
#define FWD_SWITCH_HOLD_TIME_MS           3000

#undef  INTELLIGENT_PRECHARGE
#undef  WAVESCULPTOR_AUTODETECT
#define DEFAULT_WAVESCULPTOR_MODEL        WS20
