#ifndef MQTT_ESP32_H
#define MQTT_ESP32_H

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>

#define WLAN_SSID       ""
#define WLAN_PASS       ""
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define IO_USERNAME     "Popy"
#define IO_KEY          ""
#define AIO_USERNAME    ""
#define AIO_KEY         ""
#define api_key         "YOUR_API_KEY"   // Replace with your Weatherstack API key
#define city            "Paris"


void init_WIFI();
void weatherstack_API();

void init_mqtt();
void process_mqtt();

#endif
