#include <low_power_esp32.h>

#define uS_TO_mS_FACTOR 1000ULL 
#define TIME_TO_SLEEP  2000 
#define LED_PIN 2

/* 
param sleep mode :
    0, no sleep mode init 
    1, sleep mode init 
    2, deep sleep mode init
*/
void int_blink(int sleep_mode) 
{ 
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  // ESP32 wakes after TIME_TO_SLEEP milliseconds  
  if (sleep_mode == 1) 
  {
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_mS_FACTOR); 
  }
  else if (sleep_mode == 2)
  {
    digitalWrite(LED_PIN,HIGH);  
    delay(TIME_TO_SLEEP); 
    digitalWrite(LED_PIN,LOW);
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_mS_FACTOR);
    esp_deep_sleep_start();
  }
  
  
}

void simple_blink() 
{
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(1000);       
  // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  delay(1000);                      // wait for a second
}

void sleep_blink()
{
    digitalWrite(LED_PIN,HIGH);  
    delay(TIME_TO_SLEEP);  
    digitalWrite(LED_PIN,LOW);  
    esp_light_sleep_start();
}