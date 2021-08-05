/*
 * Project PlanC
 * Description: Kalif's capstone Plan C
 * Author: Brian Rashap
 * Date: 05-AUG-2021
 */

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