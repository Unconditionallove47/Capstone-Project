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

//Dashboard library setup
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT.h" 
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h" 


//Library setup for particle and GPS
#include "Particle.h"
#include "TinyGPS++.h"
void setup();
void loop();
void displayInfo();
void helloWorld();
void MQTT_connect();
#line 19 "c:/Users/kalif/Documents/IoT/Capstone-Project/Capstone/src/Capstone.ino"
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

Servo myServo;

//Water Sensor Setup
const int waterSensorToilet = A0;
int waterSensorTValue;

const int waterSensorSink = A1;
int waterSensorSValue;

//Occupancy Sensor Setup
const int occupantSensor = A2;
int occupantSensorValue;

//Air Quality Sensor Setup
const int airQualitySensor = A3;
int airQualitySensorValue;

//Setting Servo Position
int servoPosition = 0;


//adafruit.io settings for publish and sub
TCPClient TheClient; 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 
Adafruit_MQTT_Publish GPSObject = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/GPS");
unsigned long last, lastTime;
unsigned long timeStamp;    //Timestamp for current time
unsigned long lastStamp;
//oled and adafruit live time
String DateTime , TimeOnly ;
float gpsValue;




void setup()
{

  Serial.begin(9600);

  myServo.attach(D5);

  // The GPS module initialization
  Serial1.begin(9600);
  startFix = millis();
  gettingFix = true;

  // //PinMode setup for water sensor
  pinMode(waterSensorToilet, INPUT);
  pinMode(waterSensorSink, INPUT);

  //Oled display turned on
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  helloWorld();
  Serial.printf("Hello World\n");
}
// loop() runs over and over again, as quickly as it can execute.
void loop()
{

MQTT_connect();
  timeStamp = millis();     // TIMESTAMP TO MILLISECONDS
  DateTime = Time . timeStr () ; // Current Date and Time from Particle Time class
  TimeOnly = DateTime . substring (11 ,19) ;


  //turns on gps printouts
  while (Serial1.available() > 0)
  {
    if (gps.encode(Serial1.read()))
    {
      displayInfo();
    }
  }
  delay(1000);

  //Reading WaterSensor Value
  waterSensorTValue = analogRead(waterSensorToilet);
  Serial.printf("Behind Toilet Water Value is %d \n", waterSensorTValue);
  //Reading WaterSensor Value
  waterSensorSValue = analogRead(waterSensorSink);
  Serial.printf("Sink Water Value is %d \n", waterSensorSValue);
  //Reading AirQuality
  airQualitySensorValue = analogRead(airQualitySensor);
  Serial.printf("air Quality is %d \n", airQualitySensorValue);

  //Reading Occupancy Value
  occupantSensorValue = analogRead(occupantSensor);
  Serial.printf("Occupancy value is %d \n", occupantSensorValue);

  // for(servoPosition = 0; servoPosition < 180; servoPosition += 1)  // goes from 0 degrees to 180 degrees
  //   {                                  // in steps of 1 degree
  //     myServo.write(servoPosition);              // tell servo to go to position in variable 'pos'
  //     delay(5);
  //   }
  //   for(servoPosition = 180; servoPosition>=1; servoPosition-=1)     // goes from 180 degrees to 0 degrees
  //   {
  //     myServo.write(servoPosition);              // tell servo to go to position in variable 'pos'
  //     delay(5);
  //   }

//MQTT ping to make sure it still works
  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
    if(! mqtt.ping()) {
      Serial.printf("Disconnecting \n");
      mqtt.disconnect();
    }
  last = millis();
  }

 // publish to cloud every 30 seconds
 gpsValue =(lat,lon,alt);
  
  if((millis()-lastTime > 15000)) {
    if(mqtt.Update()) {
      GPSObject.publish(gpsValue);
      
    } 
    lastTime = millis();
  }




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

void helloWorld()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(20, 5);
  display.println("GPS Initializing");
  display.display();
}


//connects MQTT automatically using function
void MQTT_connect() {
  int8_t ret;
  // Stop if already connected.
  if (mqtt.connected()) {
    return;
   }
  Serial.print("Connecting to MQTT... ");
     while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
      Serial.printf("%s\n",(char *)mqtt.connectErrorString(ret));
      Serial.printf("Retrying MQTT connection in 5 seconds..\n");
      mqtt.disconnect();
      delay(5000);  // wait 5 seconds
     }
  Serial.printf("MQTT Connected!\n");
} 