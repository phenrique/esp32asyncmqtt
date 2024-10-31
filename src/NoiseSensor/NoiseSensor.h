#ifndef NOISE_SENSOR_H
#define NOISE_SENSOR_H

#include <Arduino.h>

#define GPIO34 ADC1_CHANNEL_6

class NoiseSensor
{
public:
    NoiseSensor();
    void begin();
    u_int32_t read();
};

#endif