#include <arch/gpio.h>

#include <scandal/utils.h>
#include <scandal/message.h>
#include <scandal/leds.h>

#include <project/target_config.h>
#include <project/leds_annexure.h>
#include <project/conversion.h>

/* general externs. declared in steering-wheel.c */
extern int cruise;
extern int cruise_led_flash;
extern float cruise_velocity;
extern float current_velocity;
extern int hazards;
extern int horn;
extern float set_velocity;
extern uint32_t brake;
extern int left_indicator;
extern int right_indicator;
extern int rear_vision;
extern int reverse;
extern int reverse_led_fast_flash;
extern sc_time_t reverse_switch_on_time;
extern int reverse_switch;
extern int forward;
extern sc_time_t forward_switch_on_time;
extern int forward_switch;

/* precharge system externs. declared in steering-wheel.c */
extern sc_time_t precharge_switch_on_time;
extern sc_time_t precharge_discharge_request_time;
extern int precharge_switch;
extern int precharged;
#ifdef INTELLIGENT_PRECHARGE
extern int precharge_timeout;
extern int precharging;
extern int discharging;
#else
extern int precharge_caught;
#endif

/* And now into the interrupt handlers */
static int last_speed_hold_time = 0;
void speed_hold_handler() {
	/* debouncing */
	if (sc_get_timer() >= last_speed_hold_time + 150) {
		if (cruise) {
			cruise = 0;
			cruise_led(0);
			cruise_velocity = 0.0;
		} else {
			/* we don't want cruise in reverse */
			if (reverse) {
				cruise_led_flash = 20;
			} else {
				cruise = 1;
				cruise_led(1);
				cruise_velocity = current_velocity;
			}
		}
	}

	last_speed_hold_time = sc_get_timer();

	/* ack the interrupt */
	GPIO_IntClear(SPEED_HOLD_SWITCH_PORT, SPEED_HOLD_SWITCH_BIT);
}

static int last_speed_up_time = 0;
void speed_up_handler() {
	/* debouncing */
	if (sc_get_timer() - last_speed_up_time > 150) {
		if (cruise) {
			cruise_velocity += kph2mps(0.5);
			cruise_led_flash = 20;
		}
	}

	last_speed_up_time = sc_get_timer();

	/* ack the interrupt */
	GPIO_IntClear(SPEED_UP_SWITCH_PORT, SPEED_UP_SWITCH_BIT);
}

static int last_speed_down_time = 0;
void speed_down_handler() {
	/* debouncing */
	if (sc_get_timer() - last_speed_down_time > 150) {
		if (cruise) {
			cruise_velocity -= kph2mps(0.5);
			cruise_led_flash = 20;
		}
	}

	last_speed_down_time = sc_get_timer();

	/* ack the interrupt */
	GPIO_IntClear(SPEED_DOWN_SWITCH_PORT, SPEED_DOWN_SWITCH_BIT);
}

static int last_horn_time = 0;
void horn_handler() {
	/* debouncing */
	if (sc_get_timer() - last_horn_time > 50) {
		if (horn)
			horn = 0;
		else
			horn = 1;
	}

	last_horn_time = sc_get_timer();

	/* ack the interrupt */
	GPIO_IntClear(HORN_SWITCH_PORT, HORN_SWITCH_BIT);
}

static int last_hazard_time = 0;
void hazards_handler() {
	/* debouncing */
	if (sc_get_timer() - last_hazard_time > 50) {

		/* if indicators are on turn them off */
		if (left_indicator || right_indicator) {
			left_indicator = 0;
			right_indicator = 0;
			left_led(0);
			right_led(0);
		}

		if (hazards) {
			hazards = 0;
			left_led(0);
			right_led(0);
		} else {
			hazards = 1;
		}
	}

	last_hazard_time = sc_get_timer();

	/* ack the interrupt */
	GPIO_IntClear(HAZARDS_SWITCH_PORT, HAZARDS_SWITCH_BIT);
}

static int last_left_indicator_time = 0;
void left_indicator_handler() {
	/* debouncing */
	if (sc_get_timer() - last_left_indicator_time > 50) {
		/* if hazards are on turn them off */
		if (hazards)
			hazards = 0;

		if (left_indicator) {
			left_indicator = 0;
			left_led(0);
		} else {
			left_indicator = 1;
		}
	}

	last_left_indicator_time = sc_get_timer();

	/* ack the interrupt */
	GPIO_IntClear(LEFT_INDICATOR_SWITCH_PORT, LEFT_INDICATOR_SWITCH_BIT);
}

static int last_right_indicator_time = 0;
void right_indicator_handler() {
	/* debouncing */
	if (sc_get_timer() - last_right_indicator_time > 50) {
		/* if hazards are on turn them off */
		if (hazards)
			hazards = 0;

		if (right_indicator) {
			right_indicator = 0;
			right_led(0);
		} else {
			right_indicator = 1;
		}
	}

	last_right_indicator_time = sc_get_timer();

	/* ack the interrupt */
	GPIO_IntClear(RIGHT_INDICATOR_SWITCH_PORT, RIGHT_INDICATOR_SWITCH_BIT);
}

void brake_handler() {
	/* ack the interrupt early, else we miss an edge */
	GPIO_IntClear(BRAKE_PORT, BRAKE_BIT);
	//if (sc_get_timer() - last_brake_time > 50) {
	brake = GPIO_GetValue(BRAKE_PORT, BRAKE_BIT);
	if(brake){
		cruise = 0;
		cruise_led(0);
	}
	//}

}

static int last_rear_vision_time = 0;
void rear_vision_handler() {
	/* debouncing */
	if (sc_get_timer() - last_rear_vision_time > 50) {
		if (rear_vision)
			rear_vision = 0;
		else
			rear_vision = 1;
	}

	last_rear_vision_time = sc_get_timer();

	/* ack the interrupt */
	GPIO_IntClear(REAR_VISION_SWITCH_PORT, REAR_VISION_SWITCH_BIT);
}

/* If we are doing intelligent precharge, then all that needs to happen in here is the
 * switch state transition */
static int last_precharge_time = 0;
void precharge_handler() {
	/* debouncing */
	if (sc_get_timer() >= last_precharge_time + 200) {
		/* precharge switch was on, has been released. Check time */
		if (precharge_switch) {
			precharge_switch = 0;

#ifndef INTELLIGENT_PRECHARGE
			if ((sc_get_timer() >= precharge_switch_on_time + PRECHARGE_SWITCH_HOLD_TIME_MS)
				&& !precharge_caught) {

				precharge_caught = 1;

				/* going into discharged state */
				if (precharged) {
					precharge_led(0);
					precharged = 0;

					scandal_send_channel(TELEM_LOW, STEERINGWHEEL_START, 0);
					scandal_delay(10);
					scandal_send_channel(TELEM_LOW, STEERINGWHEEL_START, 0);
					scandal_delay(10);
					scandal_send_channel(TELEM_LOW, STEERINGWHEEL_START, 0);
					scandal_delay(10);
					scandal_send_channel(TELEM_LOW, STEERINGWHEEL_START, 0);

				/* going into precharged state */
				} else {
					precharge_led(1);
					precharged = 1;

					scandal_send_channel(TELEM_LOW, STEERINGWHEEL_START, 1);
					scandal_delay(10);
					scandal_send_channel(TELEM_LOW, STEERINGWHEEL_START, 1);
					scandal_delay(10);
					scandal_send_channel(TELEM_LOW, STEERINGWHEEL_START, 1);
					scandal_delay(10);
					scandal_send_channel(TELEM_LOW, STEERINGWHEEL_START, 1);
				}

			/* we released before the hold time */
			} else if (!precharge_caught) {
				precharge_led(0);
			}
#endif

		/* precharge switch has been pressed */
		} else {
			precharge_switch = 1;

#ifndef INTELLIGENT_PRECHARGE
			precharge_caught = 0;
#endif

			precharge_switch_on_time = sc_get_timer();
		}
	}

	last_precharge_time = sc_get_timer();

	/* ack the interrupt */
	GPIO_IntClear(PRECHARGE_SWITCH_PORT, PRECHARGE_SWITCH_BIT);
}

/* the timeouts are handled in the main loop */
static int last_reverse_time = 0;
void reverse_handler() {
	/* debouncing */
	if (sc_get_timer() >= last_reverse_time + 50) {

		/* reverse switch was on, has been released. Check time */
		if (reverse_switch) {
			reverse_switch = 0;

			/* if we didn't wait long enough, just turn the LED off and leave it */
			if (!(sc_get_timer() >= reverse_switch_on_time + REVERSE_SWITCH_HOLD_TIME_MS)) {
				reverse_led(0);

			/* if we did wait long enough, but we're already in reverse, just leave it */
			} else {
				if (reverse)
					goto out;
			}

		/* reverse switch has been pressed */
		} else {
			/* we're aready in reverse! */
			if (reverse)
				goto out;

			/* only allow reverse if we're not currently moving */
			if (current_velocity == 0) {
				reverse_switch = 1;
				reverse_switch_on_time = sc_get_timer();
			} else {
				reverse_led_fast_flash = 20;
			}
		}
	}

out:
	last_reverse_time = sc_get_timer();

	/* ack the interrupt */
	GPIO_IntClear(REV_SWITCH_PORT, REV_SWITCH_BIT);
}

/* the timeouts are handled in the main loop */
static int last_forward_time = 0;
void forward_handler() {
	/* debouncing */
	if (sc_get_timer() >= last_forward_time + 50) {

		/* forward switch was on, has been released. Check time */
		if (forward_switch) {
			forward_switch = 0;

			/* if we didn't wait long enough with the hold, turn the LED off */
			if (!(sc_get_timer() >= forward_switch_on_time + FWD_SWITCH_HOLD_TIME_MS)) {
				reverse_led(0);

			/* if we did wait long enough, but we're already going foward, just leave it */
			} else {
				if (!reverse)
					goto out;
			}

		/* forward switch has been pressed */
		} else {

			/* we're already going forward */
			if (!reverse)
				goto out;

			forward_switch = 1;
			forward_switch_on_time = sc_get_timer();
		}
	}

out:
	last_forward_time = sc_get_timer();

	/* ack the interrupt */
	GPIO_IntClear(FWD_SWITCH_PORT, FWD_SWITCH_BIT);
}

