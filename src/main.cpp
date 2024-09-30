#include <Arduino.h>

#define uptime      1000
#define downtime    1000
#define interval    2000  // Interval of 1 second
int ledPin = 2;  // Use GPIO 2 for built-in LED (on many ESP32 boards)
unsigned long previousMillis = 0;

// Define function
  void morse_s(int ledPin);
  void morse_o(int ledPin);
  void morse_espace_mots(int ledPin);
  void morse_espace_lettre(int ledPin);
  void sendhello();



void setup() {
  // Initialize the LED pin as an output
  pinMode(ledPin, OUTPUT);
  Serial.begin(115200);
  Serial.println("ESP32 with millis() for non-blocking delay");
}

void loop() {
  morse_s(ledPin);
  morse_espace_lettre(ledPin);
  morse_o(ledPin);
  morse_espace_lettre(ledPin);
  morse_s(ledPin);
  morse_espace_mots(ledPin);
  //sendhello();
  
}
//Pas compatible avec morse! 
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