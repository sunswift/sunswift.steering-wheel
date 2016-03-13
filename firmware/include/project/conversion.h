#ifndef _CONVERSION_H
#define _CONVERSION_H

float mps2kph(float mps);
float mps2rpm(float mps, float wheel_diameter);
float rpm2mps(float rpm, float wheel_diameter);
float kph2mps(float kph);

#endif
