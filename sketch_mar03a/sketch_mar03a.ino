#include <SoftwareSerial.h>

SoftwareSerial BTSerial(3, 10); //RX, TX //SoftwareSerial BTSerial(3, 10); //RX, TX

void setup(){
  BTSerial.begin(9600);
  Serial.begin(9600);
  while(!Serial);
}

void loop(){
  if (BTSerial.available()) {
    Serial.write(BTSerial.read());
  }
  if (Serial.available()) {
    BTSerial.write(Serial.read());
  }
} /*  MASTER
18:00:35.085 -> OK
18:00:40.643 -> +UART:9600,0,0
18:00:40.643 -> OK
18:00:45.385 -> +ADDR:13:EF:11D7
18:00:45.385 -> OK
18:01:09.519 -> +CMODE:0
18:01:09.519 -> OK
18:01:22.244 -> +BIND:18:91:D7FA7F
18:01:22.244 -> OK
*/
