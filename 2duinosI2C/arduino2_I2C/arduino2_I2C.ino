#include <Wire.h>
//  SLAVE DEVICE CODE
#define SLAVEADDRESS 0x32
String  response1 = "LED ON";
String  response2 = "LED OFF";
String  instruction = "";

void setup() {
  pinMode(13,OUTPUT);
  Serial.begin(9600);
  Wire.begin(SLAVEADDRESS); // My slave address
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void receiveEvent(int16_t b){  //Recieve data from master
  //byte b; 
  while(Wire.available()){
    b += Wire.read();
    instruction += b;
  }
  if(instruction == "ON")
    digitalWrite(13,HIGH);
  else if(instruction == "OFF")
    digitalWrite(13,LOW);
}

void requestEvent(int16_t b){    // What to reply to Master Device
  char buffer[32];
  if(instruction == "ON"){
    response1.toCharArray(buffer, 10);
    Wire.write(buffer, 6);
  } 
  else if (instruction == "OFF"){
    response2.toCharArray(buffer, 10);
    Wire.write(buffer, 7);
  } 
}

void loop(){
  //BLANK
}
