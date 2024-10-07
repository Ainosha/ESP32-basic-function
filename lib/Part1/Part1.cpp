#include "Part1.h"

// Define and initialize global variables
unsigned long previousMillis = 0;
unsigned long sensorValue = 0;
volatile int buttonCounter = 0;
int lastButtonState = LOW;
int sensorMax = 1023;  // Maximum sensor value
int sensorMin = 50;    // Minimum sensor value

void sendhello(){
  unsigned long currentMillis = millis();
  // Check if the interval has passed
  if (currentMillis - previousMillis >= interval) {
    int time = currentMillis - previousMillis;
    Serial.printf("Time %d ",time);
    previousMillis = currentMillis;
    Serial.println("Hello World");
  }
}

//Setup in pullup schematics
void handleButtonPress(){
    buttonCounter++; // Increment the counter when the button is pressed
}
void interruptshow(){
  Serial.print("Button Press Count: ");
  Serial.println(buttonCounter); // Display the count on the serial monitor
  delay(1000);
}
void readbutton(int pin){ 
int currentButtonState = digitalRead(pin); // Read the current state of the button

  // Check for a button press (transition from HIGH to LOW)
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    buttonCounter++; // Increment the counter
    Serial.print("Button Press Count: ");
    Serial.println(buttonCounter); // Display the count on the serial monitor
    delay(50); // Simple debounce delay
  }
  // Update lastButtonState
  lastButtonState = currentButtonState;
      
}

void morse_s(int Pin){
  unsigned long currentMillis = millis();
  for(int i=0; i<3;i++){
    // Turn the LED on
    digitalWrite(Pin, HIGH);
    delay(dottime);  
    
    // Turn the LED off
    digitalWrite(Pin, LOW);
    delay(dottime); 
    Serial.print(".");
  }
  unsigned long dif = millis() - currentMillis;
  Serial.printf("\nTime send S : %d ms \n",dif);
}

void morse_o(int Pin){
  unsigned long currentMillis = millis();
  for(int i=0; i<3;i++){
    // Turn the LED on
    digitalWrite(Pin, HIGH);
    delay(3*dottime);  
    
    // Turn the LED off
    digitalWrite(Pin, LOW);
    delay(dottime); 
    Serial.print("-");
  }
  unsigned long dif = millis() - currentMillis;
  Serial.printf("\nTime send O : %d ms \n",dif);
}

void morse_espace_lettre(int ledPin){
  unsigned long currentMillis = millis();
  for(int i=0; i<3;i++){
    // Turn the LED off
    digitalWrite(ledPin, LOW);
    delay(dottime);  
    Serial.print(".");
  }
  unsigned long dif = millis() - currentMillis;
  Serial.printf("\nTime send espace lettre : %d ms \n",dif);
}

void morse_espace_mots(int ledPin){
  unsigned long currentMillis = millis();
  for(int i=0; i<7;i++){    
    // Turn the LED off
    digitalWrite(ledPin, LOW);
    delay(dottime);  
    Serial.print(".");
  }
  unsigned long dif = millis() - currentMillis;
  Serial.printf("\nTime send espace word : %d ms \n",dif);
}

void MessageSOS(int ledPin){
  morse_s(ledPin);
  morse_espace_lettre(ledPin);
  morse_o(ledPin);
  morse_espace_lettre(ledPin);
  morse_s(ledPin);
  morse_espace_mots(ledPin);
}

void morse_pattern(int Pin, const char* pattern) {
    unsigned long currentMillis = millis();
    for (int i = 0; pattern[i] != '\0'; i++) {
        if (pattern[i] == '.') {
            digitalWrite(Pin, HIGH);
            delay(dottime);
            digitalWrite(Pin, LOW);
            delay(dottime);
        } else if (pattern[i] == '-') {
            digitalWrite(Pin, HIGH);
            delay(3 * dottime);
            digitalWrite(Pin, LOW);
            delay(dottime);
        }
        Serial.print(pattern[i]);
    }
    unsigned long dif = millis() - currentMillis;
    Serial.printf("\nTime to send pattern '%s': %d ms\n", pattern, dif);
}

void Calibrationsensor(int sensorPin){
  while (millis() < 5000) {
    sensorValue = analogRead(sensorPin);
    // record the maximum sensor value
    if (sensorValue > sensorMax) {
      sensorMax = sensorValue;
    }
    // record the minimum sensor value
    if (sensorValue < sensorMin) {
      sensorMin = sensorValue;
    }
  }
  sensorValue = map(sensorValue, sensorMin, sensorMax, 0, 255);
}

void readsensor(int sensorPin){
  int sensorValue = analogRead(sensorPin);

  // in case the sensor value is outside the range seen during calibration
  sensorValue = constrain(sensorValue, sensorMin, sensorMax);

  // apply the calibration to the sensor reading
  sensorValue = map(sensorValue, sensorMin, sensorMax, 0, 255);
  Serial.printf("Analog Value from (GPIO%d): %d\n", sensorPin,sensorValue);
  delay(1000);
}

void LM35(){
  //quantum NodeMCU ESP32 adc   :   q = 1.1V/4095 = 0.27 mV/bit
  //équation LM32               :   Vout = 10mV/°C * T
  //1°C =>                          output = Vout/q = (10 mV/°C)/[(0.27 mV/bit)] * T = (37 bit/°C)* T => T = 0.027 *output

  adc1_config_width(ADC_WIDTH_BIT_12);// sur 12 bit
  adc1_config_channel_atten(ADC1_CHANNEL_5,ADC_ATTEN_DB_0); // sur un tension 1.1V sur channel 5 pin VP
  int value = adc1_get_raw(ADC1_CHANNEL_5); //prendre valeur
  float Temp = (value *0.027) + calibrationOffset; //calcul + offset (0)
  Serial.printf("Température LM35 : %f °C\n", Temp); //montrer résultat console
  delay(1000);

}

void LDR(){
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_DB_12);
  int value = adc1_get_raw(ADC1_CHANNEL_0);
    // Print the ADC raw value
  Serial.printf("Raw ADC Value: %d\n", value);

  float voltage = (float)value * (3.9 / 4095);
  float resistor_ldr = (float)Resistance * ((3.3 - voltage)/voltage);
  Serial.printf("Voltage: %.2f V\nResitor: %.2f\n", voltage,resistor_ldr);

  float lux =  constant * pow(resistor_ldr,droite_directeur); // Rough estimation??
  Serial.printf("Estimated Lux: %.2f lumens\n", lux);
  Serial.println("==============================================");
  delay(1000);
}