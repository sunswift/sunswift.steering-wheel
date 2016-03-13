#include <scandal/wavesculptor.h>
#include <scandal/tritium.h>
#include <scandal/message.h>
#include <scandal/leds.h>

#include <project/target_config.h>

#include <string.h>

sc_time_t       ws_last_drive_command_time;
extern float    current_velocity;
extern float    current_bus_voltage;
extern uint32_t wavesculptor_model;

void ws_bus_handler(float bus_current, float bus_voltage, uint32_t time) {
	current_bus_voltage = bus_voltage;
}

void ws_status_handler(uint8_t rcv_err_count, uint8_t tx_err_count, uint16_t active_motor,
						uint16_t err_flags, uint16_t limit_flags, uint32_t time) {

}

#ifdef WAVESCULPTOR_AUTODETECT
void ws_base_handler(char *tritium_id, uint32_t serial_number, uint32_t time) {
	/* TO88 is the id string that the WS22 sends out */
	if (strncmp("TO88", tritium_id, 4) == 0)
		wavesculptor_model = WS22;
	/* TRIa is the id string that the WS20 sends out */
	else if (strncmp("TRIa", tritium_id, 4) == 0)
		wavesculptor_model = WS20;
}
#endif

void ws_velocity_handler(float vehicle_velocity, float motor_rpm, uint32_t time) {
	current_velocity = vehicle_velocity;
}

void init_ws_in_channels(void) {
	scandal_register_ws_bus_callback(&ws_bus_handler);
	scandal_register_ws_status_callback(&ws_status_handler);
#ifdef WAVESCULPTOR_AUTODETECT
	scandal_register_ws_base_callback(&ws_base_handler);
#endif
	scandal_register_ws_velocity_callback(&ws_velocity_handler);
}

void handle_ws_drive_commands(float velocity, float bus_current, float motor_current) {

	if (sc_get_timer() - ws_last_drive_command_time >= WS_TIME_BETWEEN_DRIVE_COMMANDS_MS) {
		scandal_send_ws_drive_command(DC_DRIVE, velocity, motor_current);
		scandal_send_ws_drive_command(DC_POWER, 0.0, bus_current);
//		scandal_send_ws_id(DC_BASE, "TRIb", 4);
		ws_last_drive_command_time = sc_get_timer();
	}

}
