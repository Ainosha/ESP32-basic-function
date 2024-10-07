#ifndef MQTT_ESP32.H
#define MQTT_ESP32.H

#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>

#define WLAN_SSID       ""
#define WLAN_PASS       ""
#define AIO_SERVER      ""
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define IO_USERNAME     ""
//#define IO_KEY          ""

void process_mqtt();
void init_mqtt();

#endif
