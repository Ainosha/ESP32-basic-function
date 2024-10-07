#ifndef PART1_H
#define PART1_H

#include <Arduino.h>
#include <driver/adc.h>
#include <Wire.h>


// Definitions
#define dottime              1000
#define calibrationOffset    10

// LDR config constants
#define constant             46441588.83
#define droite_directeur     -1.333
#define Resistance           10000 // ohms

// Variable declarations (use extern, no initialization)
extern int interval;               // Interval for delay
extern unsigned long previousMillis;
extern unsigned long sensorValue;
extern volatile int buttonCounter;
extern int lastButtonState;

extern int sensorMax;  // Maximum sensor value
extern int sensorMin;  // Minimum sensor value

// Function declarations
void sendhello();
void readbutton(int pin);
void handleButtonPress();
void interruptshow();
void morse_s(int ledPin);
void morse_o(int ledPin);
void morse_espace_mots(int ledPin);
void morse_espace_lettre(int ledPin);
void MessageSOS(int ledPin);
void morse_pattern(int Pin, const char* pattern);
void Calibrationsensor(int sensorPin);
void readsensor(int sensorPin);
void LM35();
void LDR();

#endif // PART1_H
