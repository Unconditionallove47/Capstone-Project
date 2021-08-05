/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "c:/Users/kalif/Documents/IoT/Capstone-Project/Capstone/src/Capstone.ino"
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
void setup();
void loop();
void displayInfo();
void airQualitySensor();
void helloWorld();
#line 13 "c:/Users/kalif/Documents/IoT/Capstone-Project/Capstone/src/Capstone.ino"
const unsigned long PUBLISH_PERIOD = 120000;
const unsigned long SERIAL_PERIOD = 5000;
const unsigned long MAX_GPS_AGE_MS = 10000;

TinyGPSPlus gps;
//Setting offset to PST
const int UTC_offset = -6;
unsigned long lastSerial = 0;
unsigned long lastPublish = 0;
unsigned long startFix = 0;
bool gettingFix = false;
//Longitude,Latitude,Altitude
float lat, lon, alt;

//rotation setting for oled, and its defines,library's, etc
int rot = 0;
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#define OLED_RESET 4        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(OLED_RESET);

//AirQuality Sensor and setup and library
#include "Air_Quality_Sensor.h"
AirQualitySensor sensor(A2);

Servo myServo;

//Water Sensor Setup
int waterSensor = A0;
int waterSensorValue = 0;

// myServo.attach(A3);

// setup() runs once, when the device is first turned on.
void setup()
{

  Serial.begin(9600);
  

  // The GPS module initialization
  Serial1.begin(9600);
  startFix = millis();
  gettingFix = true;

  // //PinMode setup for water sensor
  pinMode(waterSensor, INPUT);

  // air quality sensor serial monitor test settings
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
//Oled display turned on
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  // OledText();
  helloWorld();
  Serial.printf("Hello World\n");
}
// loop() runs over and over again, as quickly as it can execute.
void loop()
{
  //turns on gps printouts
  while (Serial1.available() > 0) {
    if (gps.encode(Serial1.read())) {
      displayInfo();
    }
  }
  delay(1000);

  //Reading WaterSensor
  waterSensorValue = analogRead(waterSensor);
  Serial.printf("Value is %d \n", waterSensor);

  airQualitySensor();


  
}

//Does just about everything for GPS prints/time and display outputs
void displayInfo()
{
  float lat, lon, alt;
  uint8_t hr, mn, se, sat;
  if (millis() - lastSerial >= SERIAL_PERIOD)
  {
    lastSerial = millis();

    char buf[128];
    if (gps.location.isValid() && gps.location.age() < MAX_GPS_AGE_MS)
    {
      lat = gps.location.lat();
      lon = gps.location.lng();
      alt = gps.altitude.meters();
      hr = gps.time.hour();
      mn = gps.time.minute();
      se = gps.time.second();
      sat = gps.satellites.value();

      if (hr > 7)
      {
        hr = hr + UTC_offset;
      }
      else
      {
        hr = hr + 24 + UTC_offset;
      }
      Serial.printf("Time: %02i:%02i:%02i --- ", hr, mn, se);
      Serial.printf("lat: %f, long: %f, alt: %f \n", lat, lon, alt);
      if (gettingFix)
      {
        gettingFix = false;
        unsigned long elapsed = millis() - startFix;
        Serial.printlnf("%lu milliseconds to get GPS fix", elapsed);
      }
      display.clearDisplay();
      display.setCursor(0, 0);
      display.printf("Time: %02i:%02i:%02i \n", hr, mn, se);
      display.printf("lat  %f \nlong %f \nalt %f\n", lat, lon, alt);
      display.printf("satelites %i", sat);
      display.display();
    }
    else
    {
      strcpy(buf, "no location");
      if (!gettingFix)
      {
        gettingFix = true;
        startFix = millis();
      }
    }
  }
}


//function for text style setup for oled
// void OledText(void)
// {
//   display.clearDisplay();
//   display.setTextSize(1);      // Normal 1:1 pixel scale
//   display.setTextColor(WHITE); // Draw white text
//   display.setTextSize(1);
//   display.setCursor(20, 5); // Start at top-left corner
//   display.println("GPS Initializing");
//   display.display();
// }

//Function for Air Quality Sensor
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

void helloWorld() {
	display.clearDisplay();
	display.setTextSize(1);
  	display.setTextColor(WHITE);
  	display.setCursor(20,5);
  	display.println("GPS Initializing");
	display.display();
}