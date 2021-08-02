/*
 * Project Capstone
 * Description:
 * Author:
 * Date:
 */
#include "credentials.h"

#include "Air_Quality_Sensor.h"
 AirQualitySensor sensor(A2);

// setup() runs once, when the device is first turned on.
void setup() {
  // Put initialization like pinMode and begin functions here.
//air quality sensor serial monitor test settings
 Serial.println("Waiting sensor to init...");
  delay(1000);
  
  if (sensor.init()) {
    Serial.println("Sensor ready.");
  }
  else {
    Serial.println("Sensor ERROR!");
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  // The core of your code will likely live here.
//tests the current air quality
  airQualitySensor();
}




void airQualitySensor(){
int quality = sensor.slope();

  Serial.print("Sensor value: ");
  Serial.println(sensor.getValue());
  
  if (quality == AirQualitySensor::FORCE_SIGNAL) {
    Serial.println("High pollution! Force signal active.");
  }
  else if (quality == AirQualitySensor::HIGH_POLLUTION) {
    Serial.println("High pollution!");
  }
  else if (quality == AirQualitySensor::LOW_POLLUTION) {
    Serial.println("Low pollution!");
  }
  else if (quality == AirQualitySensor::FRESH_AIR) {
    Serial.println("Fresh air.");
  }
  
  delay(1000);


}