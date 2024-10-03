#include <Arduino.h>
#include <driver/adc.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <LiquidCrystal_I2C.h>

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// Wifi tel Thomas
const char* ssid     = "wifi_thomas_s"; 
const char* password = "totolerigolo";

LiquidCrystal_I2C lcd(0x27, 16, 2);

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME680 bme;

#define dottime      1000
#define interval    2000
#define calibrationOffset   10

//LDR config constante
#define constant            46441588.83
#define droite_directeur    -1.333
#define Resistance          10000//ohm

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

int lcd_count=0;

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
void LM35();
void LDR();

void init_BME();
void read_BME();

void init_LCD();
void LCD_display_counter_s();
void LCDxBME_display_temp();

void init_WIFI();
void weatherstack_API();


void setup() {
  // Initialize the LED pin as an output
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  pinMode(sensorPin, INPUT);  // Set the VP pin as an input
  pinMode(sensorPinVN, INPUT);  // Set the VN pin as an input

  //pinMode(buttonPin, INPUT_PULLUP); // Set the button pin as input with pull-up resistor
  Serial.println("ESP32 with millis() for non-blocking delay");
  attachInterrupt(digitalPinToInterrupt(buttonPin), handleButtonPress, FALLING);
  //Calibrationsensor(sensorPin);
  //Calibrationsensor(sensorPinVN);

  //init_BME();
  //init_LCD();
  //init_WIFI();

}

void loop() {
  //sendhello();
  //MessageSOS(ledPin);
  //readsensor(sensorPin);
  //readsensor(sensorPinVN);

  //read_BME();
  //LCD_display_counter_s();
  //weatherstack_API();
  //readbutton(buttonPin);
  //interruptshow();
  //LM35();
  LDR();

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


// 2.1

/*
TEST CAPETEUR BME
I2C
D21 => SDA
D22 => SCL

 */
void init_BME()
{
   //Serial.begin(115200); 
   while (!Serial); 
   Serial.println(F("BME680 test"));
   
   if (!bme.begin()) 
   {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
   }
   bme.setTemperatureOversampling(BME680_OS_8X);
   bme.setHumidityOversampling(BME680_OS_2X);
   bme.setPressureOversampling(BME680_OS_4X);
   bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
   bme.setGasHeater(320, 150);
}

void read_BME()
{
  if (! bme.performReading()) 
  {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Serial.print("Temperature = ");
  Serial.print(bme.temperature);
  Serial.println(" *C");

  Serial.print("Temperature = ");
  Serial.print((bme.temperature * (9/5)) + 32);
  Serial.println(" *F");
  
  Serial.print("Humidity = ");
  Serial.print(bme.humidity);
  Serial.println(" %");
  Serial.println();
  delay(2000);
}

/*
I2C
D21 => SDA
D22 => SCL
+5V
GND
*/
void init_LCD()
{
  //initialize lcd screen
  lcd.init();
}

/* amélioration utilier un timer qui fait une interruption toutes les secondes pour update la valeur de lcd_counter*/
void LCD_display_counter_s()
{
  lcd.backlight();
  lcd.setCursor(0, 0); 
  lcd.print("Great IoT Tutorial!");
  //lcd.scrollDisplayRight();
  lcd.setCursor(0, 1);
  lcd.print("Launched for ");
  lcd.print(lcd_count++);
  lcd.print("s");
  delay(1000);
}

void LCDxBME_display_temp() //a tester
{
  lcd.backlight();
  lcd.setCursor(0, 0); 
  lcd.print("Temp :");
  lcd.print(bme.temperature);
  lcd.println("*C");

  lcd.setCursor(0, 1);
  lcd.print("Hum :");
  lcd.print(bme.humidity);
  lcd.println("%");
  //lcd.scrollDisplayRight();
}

// 2.2
void init_WIFI()
{
  Serial.println("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void weatherstack_API()
{
  if((WiFi.status() == WL_CONNECTED)) 
  {
    HTTPClient http;
    Serial.print("[HTTP] begin...\n");
    http.begin("http://api.weatherstack.com/current?access_key=XXXXXXXXXXXXXXXX&query=Paris"); //replace XXXX... by API key
    Serial.print("[HTTP] GET...\n");
    // start connection and send HTTP header         
    int httpCode = http.GET();
    // httpCode will be negative on error
    if(httpCode == HTTP_CODE_OK) 
    {
      String payload = http.getString();
      Serial.println(payload);
      // Convert to JSON
      DynamicJsonDocument doc(1024);
      deserializeJson(doc, payload);
      // Read and display values
      String temp = doc["current"]["temperature"];
      String desc = doc["current"]["weather_descriptions"][0];         
      Serial.println("Temperature: "+temp+"*C, description: "+desc);
      } 
      else 
      {
        Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
      }
      http.end();
    }
    delay(5000);
 }
