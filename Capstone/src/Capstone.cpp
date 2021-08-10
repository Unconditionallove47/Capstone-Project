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
#include <JsonParserGeneratorRK.h>

//Dashboard library setup
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"

//Library setup for particle and GPS
#include "Particle.h"
#include "TinyGPS++.h"
void setup();
void loop();
void displayGPSInfo();
void helloWorld();
void MQTT_connect();
void createEventPayLoad();
void analogReads();
void servoMotor();
void MQTTPing();
void MQTTPublish();
void FanWithOccupancy();
void AirQualityfan();
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
const int WATERSENSORTOILET = A0;
int waterSensorTValue;

const int WATERSENSORSINK = A1;
int waterSensorSValue;

//Occupancy Sensor Setup
const int OCCUPANTSENSOR = A2;
int occupantSensorValue;

//Air Quality Sensor Setup
const int AIRQUALITYSENSOR = A3;
int airQualitySensorValue;

//Setting Servo Position
int servoPosition = 0;

const int SERVOPIN = D5;

const int RELAYPIN = D6;

int Hour;
//adafruit.io settings for publish and sub
TCPClient TheClient;
Adafruit_MQTT_SPARK mqtt(&TheClient, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish GPSObject = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/GPS");
Adafruit_MQTT_Publish ToiletSensorObject = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/toiletsensor");
Adafruit_MQTT_Publish SinkSensorObject = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/sinksensor");
Adafruit_MQTT_Publish AirQualityObject = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/air-quality");
Adafruit_MQTT_Publish OccupancyObject = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/occupancy");
unsigned long last, lastTime;
unsigned long timeStamp; //Timestamp for current time
unsigned long lastStamp;

//oled and adafruit live time
String DateTime, TimeOnly;
float gpsValue, sinkSensorValue, toiletSensorValue, airQualityValue, OccupancyValue;

void setup()

{

  Serial.begin(9600);

  //Servo motor set to D5 pin
  myServo.attach(SERVOPIN);

  //Fan via relay set to D6 pin
  pinMode(RELAYPIN, OUTPUT);

  // The GPS module initialization
  Serial1.begin(9600);
  startFix = millis();
  gettingFix = true;

  // //PinMode setup for water sensor
  pinMode(WATERSENSORTOILET, INPUT);
  pinMode(WATERSENSORSINK, INPUT);

  //Oled display turned on
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  helloWorld();
  Serial.printf("Hello World\n");

  Time.zone(-7);       // MST = -7, MDT = -6
  Particle.syncTime(); // Sync time with Particle Cloud
}

void loop()

{
  //connects argon to MQTT Service
  MQTT_connect();
  timeStamp = millis();      // TIMESTAMP TO MILLISECONDS
  DateTime = Time.timeStr(); // Current Date and Time from Particle Time class
  TimeOnly = DateTime.substring(11, 13);
  Hour = TimeOnly.toInt();
  //turns on gps printouts
  while (Serial1.available() > 0)
  {
    if (gps.encode(Serial1.read()))
    {
      displayGPSInfo();
    }
  }
  delay(1000);

  analogReads();

  servoMotor();

  FanWithOccupancy();

  AirQualityfan();

  MQTTPing();

  MQTTPublish();

  // the . c_str () method converts a String to an array of char
  Serial.printf(" Date and time is %s\n", DateTime.c_str());
  Serial.printf(" Time is %i\n", TimeOnly.toInt());
  delay(10000); // only loop every 10 seconds
}

//Does just about everything for GPS prints/time and display outputs
void displayGPSInfo()
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

//GPS initialization for GPS
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
void MQTT_connect()
{
  int8_t ret;
  // Stop if already connected.
  if (mqtt.connected())
  {
    return;
  }
  Serial.print("Connecting to MQTT... ");
  while ((ret = mqtt.connect()) != 0)
  { // connect will return 0 for connected
    Serial.printf("%s\n", (char *)mqtt.connectErrorString(ret));
    Serial.printf("Retrying MQTT connection in 5 seconds..\n");
    mqtt.disconnect();
    delay(5000); // wait 5 seconds
  }
  Serial.printf("MQTT Connected!\n");
}

//JSON for gps on dashboard
void createEventPayLoad()
{
  JsonWriterStatic<256> jw;
  {
    JsonWriterAutoObject obj(&jw);

    jw.insertKeyValue("lat", gps.location.lat());
    jw.insertKeyValue("lon", gps.location.lng());
  }
  GPSObject.publish(jw.getBuffer());
}

//Analog Readouts for water sensors,occupancy sensor, and Air Quality Sensor
void analogReads()
{
  //Reading WaterSensor Value
  waterSensorTValue = analogRead(WATERSENSORTOILET);
  Serial.printf("Behind Toilet Water Value is %d \n", waterSensorTValue);
  //Reading WaterSensor Value
  waterSensorSValue = analogRead(WATERSENSORSINK);
  Serial.printf("Sink Water Value is %d \n", waterSensorSValue);
  //Reading AirQuality
  airQualitySensorValue = analogRead(AIRQUALITYSENSOR);
  Serial.printf("air Quality is %d \n", airQualitySensorValue);
  //Reading Occupancy Value
  occupantSensorValue = analogRead(OCCUPANTSENSOR);
  Serial.printf("Occupancy value is %d \n", occupantSensorValue);
}

//Setting servo motor to turn on and off via time signature
void servoMotor()
{
  if (Hour >= 13)
  {
    (servoPosition = 0, servoPosition <= 180, servoPosition += 1); // goes from 0 degrees to 180 degrees
    myServo.write(servoPosition);                                 // tell servo to go to position in variable 'pos'
  }
  else // goes from 180 degrees to 0 degrees
   {
    (servoPosition = 180, servoPosition >= 1,servoPosition -= 1);
        myServo.write(servoPosition); // tell servo to go to position in variable 'pos'
  }
}

//pings MQTT to see if still active
void MQTTPing()
{
  //MQTT ping to make sure it still works
  if ((millis() - last) > 120000)
  {
    Serial.printf("Pinging MQTT \n");
    if (!mqtt.ping())
    {
      Serial.printf("Disconnecting \n");
      mqtt.disconnect();
    }
    last = millis();
  }
}

//publishes mqtt to cloud every 30 seconds
void MQTTPublish()
{
  toiletSensorValue = (analogRead(A0));
  sinkSensorValue = (analogRead(A1));
  occupantSensorValue = (analogRead(A2));
  airQualityValue = (analogRead(A3));
  if ((millis() - lastTime > 15000))
  {
    if (mqtt.Update())
    {
      createEventPayLoad();
      ToiletSensorObject.publish(toiletSensorValue);
      SinkSensorObject.publish(sinkSensorValue);
      AirQualityObject.publish(airQualityValue);
      OccupancyObject.publish(occupantSensorValue);
    }
    lastTime = millis();
  }
}

//turns on fan when occupancy sensor detects movement
void FanWithOccupancy()
{
  if (analogRead(A2) >= 2200)
  {
    digitalWrite(D6, HIGH);
  }
  else
  {
    digitalWrite(D6, LOW);
  }
}
//turns fan on if air quality is detected as harmful
void AirQualityfan()
{
  if (analogRead(A3) >= 1000)
  {
    digitalWrite(D6, HIGH);
  }
  else
  {
    digitalWrite(D6, LOW);
  }
}