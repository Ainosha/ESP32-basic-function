#include <low_power_esp32.h>
#include "esp32/ulp.h"
#include "driver/rtc_io.h"
#include "driver/adc.h"

#define uS_TO_mS_FACTOR 1000ULL 
#define TIME_TO_SLEEP  2000 
#define LED_PIN 2

/*
RTC_DATA_ATTR unsigned int ulp_adc_value = 0;
RTC_DATA_ATTR unsigned int ulp_adc_counter = 0;
unsigned int ulp_adc_low_threshold = 1 * (4095 / 3.9);  // 1 volt
unsigned int ulp_adc_high_threshold = 2 * (4095 / 3.9); // 2 volt
*/
static void init_ulp_program();

/* 
param sleep_mode :
    0, no sleep mode init 
    1, sleep mode init 
    2, deep sleep mode init
param buton_or_timer : 
    0, timer
    1, button
*/
void init_blink(int sleep_mode, int buton_or_timer) 
{ 
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  // ESP32 wakes after TIME_TO_SLEEP milliseconds  
  if (sleep_mode == 1) 
  {
    if( buton_or_timer == 1)
    {
      esp_sleep_enable_ext0_wakeup(GPIO_NUM_33,HIGH);
    }
    else
    {
      esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_mS_FACTOR); 
    }
    
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

const ulp_insn_t ulp_adc_program[] = {
    I_DELAY(32000),                              // Wait until ESP32 goes to deep sleep
    M_LABEL(1),                                  // LABEL 1
    I_MOVI(R3, 0),                               // Set reg. R3 to initial 0
    I_MOVI(R0, 0),                               // Set reg. R0 to initial 0
    I_MOVI(R2, 0),                               // Set reg. R2 to initial 0
    M_LABEL(2),                                  // LABEL 2
    I_ADDI(R0, R0, 1),                           // Increment cycle counter (reg. R0)
    I_ADC(R1, 0, 0),                             // Read ADC value to reg. R1
    I_ADDR(R2, R2, R1),                          // Add ADC value from reg R1 to reg. R2
    I_ADDI(R3, R3, 1),                           // R3++, count the number of ADC conversions
    M_BL(2, 4),                                  // If cycle counter is less than 4, go to LABEL 2
    I_RSHI(R0, R2, 2),                           // Divide accumulated ADC value in reg. R2 by 4 and save it to reg. R0
    M_BGE(3, ulp_adc_high_threshold),            // If average ADC value from reg. R0 is higher or equal than high_threshold, go to LABEL 3
    M_BL(3, ulp_adc_low_threshold),              // If average ADC value from reg. R0 is lower than low_threshold, go to LABEL 3
    M_BX(1),                                     // Go to LABEL 1
    M_LABEL(3),                                  // LABEL 3
    // Copy ADC value to ulp_adc_value
    I_MOVI(R1, ((unsigned int)&ulp_adc_value - (unsigned int)RTC_SLOW_MEM) / 4), // Set reg. R1 to address of ulp_adc_value
    I_ST(R0, R1, 0),                             // Copy result of ADC to R1 address, so ulp_adc_value
    // Copy number of ADC value to ulp_adc_counter
    I_MOVI(R1, ((unsigned int)&ulp_adc_counter - (unsigned int)RTC_SLOW_MEM) / 4), // Set reg. R1 to address of ulp_adc_counter
    I_ST(R3, R1, 0),                             // Copy result of ADC to R1 address, so ulp_adc_value
    I_WAKE(),                                    // Wake up ESP32
    I_END(),                                     // Stop ULP program timer
    I_HALT()                                     // Halt the coprocessor
};

void setup_adc_deep_sleep()
{
    Serial.begin(115200);
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause != ESP_SLEEP_WAKEUP_ULP)
    {
        printf("Not ULP wakeup, first run\n");
        init_ulp_program();
    }
    else
    {
        // ***** HERE YOUR SKETCH *****
        printf("Deep sleep wakeup\n");
        ulp_adc_value &= UINT16_MAX; // Check https://docs.espressif.com/projects/esp-idf/en/v4.2.3/esp32/api-guides/ulp_macros.html?highlight=i_st#c.I_ST
        ulp_adc_counter &= UINT16_MAX;
        printf("ULP did %d measurements since last reset\n", ulp_adc_counter);
        printf("ULP Value=%d was %s threshold (low=%d high=%d)\n", ulp_adc_value, ulp_adc_value < ulp_adc_low_threshold ? "below" : "above", ulp_adc_low_threshold, ulp_adc_high_threshold);

        // *** Check current value
        int value = adc1_get_raw(ADC1_CHANNEL_0);
        printf("Current value=%d\n", value);

        // *** Do not forget to reactivate adc1_ulp_enable which is disabled by adc1_get_raw
        adc1_ulp_enable();

        delay(5000);
    }
    delay(100);
    ESP_ERROR_CHECK(ulp_run(0));
    ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());
    esp_deep_sleep_start();
}

static void init_ulp_program()
{
    /* Configure ADC channel */
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_ulp_enable();

    // Set ULP wake up period to 100ms
    ulp_set_wakeup_period(0, 100 * 1000);

    size_t size = sizeof(ulp_adc_program) / sizeof(ulp_insn_t);
    ulp_process_macros_and_load(0, ulp_adc_program, &size);
}
