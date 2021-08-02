/*
 * Project GPSTest
 * Description:
 * Author:
 * Date:
 */

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