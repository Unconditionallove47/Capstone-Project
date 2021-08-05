/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "c:/Users/kalif/Documents/IoT/Capstone-Project/PlanC/src/PlanC.ino"
/*
 * Project PlanC
 * Description: Kalif's capstone Plan C
 * Author: Brian Rashap
 * Date: 05-AUG-2021
 */

void setup();
void loop();
#line 8 "c:/Users/kalif/Documents/IoT/Capstone-Project/PlanC/src/PlanC.ino"
const int leakPin = A0;
int leakValue;


void setup() {
  Serial.begin(9600);
  waitFor(Serial.isConnected,15000);

  pinMode(leakPin,INPUT);

  Serial.print("Getting Leak Data\n");

}

void loop() {
  leakValue = analogRead(leakPin);
  Serial.printf("The leak is %i\n",leakValue);
  delay(5000);
} 