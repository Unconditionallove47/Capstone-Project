/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "c:/Users/IoT_Instructor/Documents/IoT/Cohort5/kalif/Capstone-Project/GPSTest/src/GPSTest.ino"
/*
 * Project GPSTest
 * Description:
 * Author:
 * Date:
 */

void setup();
void loop();
#line 8 "c:/Users/IoT_Instructor/Documents/IoT/Cohort5/kalif/Capstone-Project/GPSTest/src/GPSTest.ino"
#define GPSSerial Serial1
void setup() {
  Serial.begin(9600);
  while (!Serial);
  // 9600 baud is the default rate for the Ultimate GPS
  GPSSerial.begin(9600);
}
void loop() {
  if (GPSSerial.available()) {
    char c = GPSSerial.read();
    Serial.write(c);
  }
}