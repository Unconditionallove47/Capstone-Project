/*
 * Project WaterSensor
 * Description:
 * Author:
 * Date:
 */
int Sensor = A0;
int val = 0;
// setup() runs once, when the device is first turned on.
void setup() {
  // Put initialization like pinMode and begin functions here.
Serial.begin(9600);
pinMode(Sensor,INPUT);

}
// loop() runs over and over again, as quickly as it can execute.
void loop() {
  val=analogRead(Sensor);
  Serial.printf("Value is %d \n",Sensor);
  // The core of your code will likely live here.
}