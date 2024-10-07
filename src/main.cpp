#include <Arduino.h>
#include <Part1.h>
#include <mqtt_esp32.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <LiquidCrystal_I2C.h>


// Wifi tel Thomas
const char* ssid     = "OnePlus 8T"; 
const char* password = "Jaibesoins4g";

LiquidCrystal_I2C lcd(0x27, 16, 2);

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME680 bme;

const int ledPin = 2;  // Use GPIO 2 for built-in LED (on many ESP32 boards)
const int sensorPin = 36;  // GPIO36 is the VP pin
const int sensorPinVN = 39;  // GPIO39 is the VN pin
const int buttonPin = 32;

int lcd_count=0;

// Define function
void init_BME();
void read_BME();

void init_LCD();
void LCD_display_counter_s();
void LCDxBME_display_temp();


void setup() {
  // Initialize the LED pin as an output
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  pinMode(sensorPin, INPUT);  // Set the VP pin as an input
  pinMode(sensorPinVN, INPUT);  // Set the VN pin as an input

  //pinMode(buttonPin, INPUT_PULLUP); // Set the button pin as input with pull-up resistor
  attachInterrupt(digitalPinToInterrupt(buttonPin), handleButtonPress, FALLING);
  Calibrationsensor(sensorPin);
  Calibrationsensor(sensorPinVN);

  //init_BME();
  //init_LCD();
  init_WIFI();

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
  //LDR();

  //weatherstack_API();
  process_mqtt();
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


