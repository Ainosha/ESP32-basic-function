#include <Arduino.h>

#define uptime      1000
#define downtime    1000
#define interval    2000  // Interval of 1 second
int sensorMax=1023;  // minimum sensor value
int sensorMin=50;  // maximum sensor value

const int ledPin = 2;  // Use GPIO 2 for built-in LED (on many ESP32 boards)
const int sensorPin = 36;  // GPIO36 is the VP pin
const int sensorPinVN = 39;  // GPIO39 is the VN pin

unsigned long previousMillis = 0;
unsigned long  sensorValue;

// Define function
void sendhello();

void morse_s(int ledPin);
void morse_o(int ledPin);
void morse_espace_mots(int ledPin);
void morse_espace_lettre(int ledPin);
void MessageSOS(int ledPin);

void Calibrationsensor(int sensorPin);
void readsensor(int sensorPin);


void setup() {
  // Initialize the LED pin as an output
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  pinMode(sensorPin, INPUT);  // Set the VP pin as an input
  Serial.println("ESP32 with millis() for non-blocking delay");
  Calibrationsensor(sensorPin);
  Calibrationsensor(sensorPinVN);

}

void loop() {
  //sendhello();
  //MessageSOS(ledPin);
  readsensor(sensorPin);
  readsensor(sensorPinVN);


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

void morse_s(int letPin){
  unsigned long currentMillis = millis();
  for(int i=0; i<3;i++){
    // Turn the LED on
    digitalWrite(ledPin, HIGH);
    delay(uptime);  // Wait for 1 second
    
    // Turn the LED off
    digitalWrite(ledPin, LOW);
    delay(downtime);  // Wait for 1 second
    Serial.print(".");
  }
  unsigned long dif = millis() - currentMillis;
  Serial.printf("\nTime send S : %d ms \n",dif);
}

void morse_o(int letPin){
  unsigned long currentMillis = millis();
  for(int i=0; i<3;i++){
    // Turn the LED on
    digitalWrite(ledPin, HIGH);
    delay(3*uptime);  // Wait for 1 second
    
    // Turn the LED off
    digitalWrite(ledPin, LOW);
    delay(downtime);  // Wait for 1 second
    Serial.print("-");
  }
  unsigned long dif = millis() - currentMillis;
  Serial.printf("\nTime send O : %d ms \n",dif);
}

void morse_espace_lettre(int letPin){
  unsigned long currentMillis = millis();
  for(int i=0; i<3;i++){
    // Turn the LED on
    digitalWrite(ledPin, HIGH);
    delay(uptime);  // Wait for 1 second
    
    // Turn the LED off
    digitalWrite(ledPin, LOW);
    delay(downtime);  // Wait for 1 second
    Serial.print(".");
  }
  unsigned long dif = millis() - currentMillis;
  Serial.printf("\nTime send espace lettre : %d ms \n",dif);
}

void morse_espace_mots(int letPin){
  unsigned long currentMillis = millis();
  for(int i=0; i<7;i++){
    // Turn the LED on
    digitalWrite(ledPin, HIGH);
    delay(uptime);  // Wait for 1 second
    
    // Turn the LED off
    digitalWrite(ledPin, LOW);
    delay(downtime);  // Wait for 1 second
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