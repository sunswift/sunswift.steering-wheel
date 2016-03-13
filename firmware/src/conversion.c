#include <project/conversion.h>

#define PI 3.14156

float mps2kph(float mps) {
	return (mps * 3.6);
}

float mps2rpm(float mps, float wheel_diameter) {
	return 60.0 * (mps / (PI * wheel_diameter));
}

float rpm2mps(float rpm, float wheel_diameter) {
	return (rpm * (PI * wheel_diameter)) / 60;
}

float kph2mps(float kph) {
	return (kph / 3.6);
}
