#ifndef LOW_POWER_ESP32_H
#define LOW_POWER_ESP32_H

#include <Arduino.h>

void init_blink(int sleep_mode, int buton_or_timer);
void simple_blink();
void sleep_blink();
void setup_adc_deep_sleep();

#endif