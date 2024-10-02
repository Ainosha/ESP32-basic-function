#include <Arduino.h>
#include <driver/adc.h>

#define dottime             1000
#define interval            2000
#define calibrationOffset   10

int sensorMax=1023;  // minimum sensor value
int sensorMin=50;  // maximum sensor value

const int ledPin = 2;  // Use GPIO 2 for built-in LED (on many ESP32 boards)
const int sensorPin = 36;  // GPIO36 is the VP pin
const int sensorPinVN = 39;  // GPIO39 is the VN pin
const int buttonPin = 32;


unsigned long previousMillis = 0;
unsigned long  sensorValue;
volatile int buttonCounter = 0;
int lastButtonState =LOW;

// Define function
void sendhello();
void readbutton(int pin);
void handleButtonPress();
void interruptshow();

void morse_s(int ledPin);
void morse_o(int ledPin);
void morse_espace_mots(int ledPin);
void morse_espace_lettre(int ledPin);
void MessageSOS(int ledPin);

void Calibrationsensor(int sensorPin);
void readsensor(int sensorPin);
void LM35(int sensorPin);


void setup() {
  // Initialize the LED pin as an output
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  pinMode(sensorPin, INPUT);  // Set the VP pin as an input
  pinMode(sensorPinVN, INPUT);  // Set the VN pin as an input

  //pinMode(buttonPin, INPUT_PULLUP); // Set the button pin as input with pull-up resistor
  Serial.println("ESP32 with millis() for non-blocking delay");
  //attachInterrupt(digitalPinToInterrupt(buttonPin), handleButtonPress, FALLING);
  //Calibrationsensor(sensorPin);
  //Calibrationsensor(sensorPinVN);

}

void loop() {
  //sendhello();
  //MessageSOS(ledPin);
  //readsensor(sensorPin);
  //readsensor(sensorPinVN);
  //readbutton(buttonPin);
  //interruptshow();
  LM35(sensorPinVN);
}

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
    delay(dottime);  // Wait for 1 second
    
    // Turn the LED off
    digitalWrite(Pin, LOW);
    delay(dottime);  // Wait for 1 second
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
    delay(3*dottime);  // Wait for 1 second
    
    // Turn the LED off
    digitalWrite(Pin, LOW);
    delay(dottime);  // Wait for 1 second
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
    delay(dottime);  // Wait for 1 second
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
    delay(dottime);  // Wait for 1 second
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

void LM35(int sensorPin){
  //quantum NodeMCU ESP32 adc   :   q = 1.1V/4095 = 0.27 mV/bit
  //équation LM32               :   Vout = 10mV/°C * T
  //1°C =>                          output = Vout/q = (10 mV/°C)/[(0.27 mV/bit)] * T = (2.7 bit/°C)* T
  
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_5,ADC_ATTEN_DB_0);
  int value = adc1_get_raw(ADC1_CHANNEL_5);
  float Temp = (value * 0.027) + calibrationOffset;
  Serial.printf("Température LM35 : %f °C\n", Temp);
  delay(1000);

}
