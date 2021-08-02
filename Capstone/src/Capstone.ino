/*
 * Project Capstone
 * Description:Smart Restroom
 * Author:Kalif Purce
 * Date:July 30th, 2021
 */

#include "credentials.h"

//Library setup for particle and GPS
#include "Particle.h"
#include "TinyGPS++.h"


//rotation setting for oled, and its defines,library's, etc
  int rot = 0;
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
 #define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
 Adafruit_SSD1306 display(OLED_RESET);
 #define NUMFLAKES     10 // Number of snowflakes in the animation example
 #define LOGO_HEIGHT   1
 #define LOGO_WIDTH    1
 #define XPOS   0 // Indexes into the 'icons' array in function below
 #define YPOS   1
 #define DELTAY 2

//AirQuality Sensor and setup and library
#include "Air_Quality_Sensor.h"
AirQualitySensor sensor(A2);

Servo myServo;

//Water Sensor Setup
int Sensor = A0;
int val = 0;



// setup() runs once, when the device is first turned on.
void setup()
{

  Serial.begin(9600);


//PinMode setup for water sensor
pinMode(Sensor,INPUT);


  //air quality sensor serial monitor test settings
  Serial.println("Waiting sensor to init...");
  delay(1000);

  if (sensor.init())
  {
    Serial.println("Sensor ready.");
  }
  else
  {
    Serial.println("Sensor ERROR!");
  }
}
// loop() runs over and over again, as quickly as it can execute.
void loop()
{


//Reading WaterSensor
   val=analogRead(Sensor);
  Serial.printf("Value is %d \n",Sensor);
  //tests the current air quality
  airQualitySensor();

//sets evrything for oled to work
displaySetup();

//sets the style of oled text
OledText();



}


//function to write text to oled
 void displaySettings(){
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextColor(WHITE,BLACK); // Draw 'inverse' text
  display.setTextSize(1);
  display.setRotation(rot);
  display.printf("Airquality is %i\n", sensor.getValue());
  display.printf("humidity is %f\n" ,bme.readHumidity());
  display.display();
  }


//function for text style setup for oled 
  void OledText(void) {
  display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(WHITE);        // Draw white text
  display.setTextSize(1.5);
  display.setCursor(0, 0);            // Start at top-left corner
  display.display();
  }





void airQualitySensor()
{
  int quality = sensor.slope();

  Serial.print("Sensor value: ");
  Serial.println(sensor.getValue());

  if (quality == AirQualitySensor::FORCE_SIGNAL)
  {
    Serial.println("High pollution! Force signal active.");
  }
  else if (quality == AirQualitySensor::HIGH_POLLUTION)
  {
    Serial.println("High pollution!");
  }
  else if (quality == AirQualitySensor::LOW_POLLUTION)
  {
    Serial.println("Low pollution!");
  }
  else if (quality == AirQualitySensor::FRESH_AIR)
  {
    Serial.println("Fresh air.");
  }

  delay(1000);
}