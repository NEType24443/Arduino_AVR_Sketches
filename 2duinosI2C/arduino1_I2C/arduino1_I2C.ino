#include <Wire.h>
//  MASTER DEVICE CODE
#define SLAVEADDRESS 0x32
#define ANSIZE 7

void setup() {
  pinMode(7, INPUT_PULLUP); //button on pin 7
  attachInterrupt(digitalPinToInterrupt(7), interrupt(), LOW);
  Serial.begin(9600);
  Wire.begin();
}
int interrupt(){
  int i ;
  i = digitalRead(7);
  if(i){
    Wire.beginTransmission(SLAVEADDRESS);
    Wire.write("ON");
    Wire.endTransmission();
  }
  else {
    Wire.beginTransmission(SLAVEADDRESS);
    Wire.write("OFF");
    Wire.endTransmission();
  }
  Wire.requestFrom(SLAVEADDRESS,ANSIZE);
  byte b;
  String response = "";
  while(Wire.available()){
    b = Wire.read();
    response+=b;
  }
  Serial.print (response);
}
void loop() {

}
