#include <MPU6050_tockn.h>
#include <Wire.h>

#include <SoftwareSerial.h>

SoftwareSerial BTSerial(10, 7); //RX, TX //SoftwareSerial BTSerial(3, 10); //RX, TX

#define MOT1  11 //IN1 //LF
#define MOT2  9  //IN2 //LB
#define MOT3  5  //In3 //RF
#define MOT4  6  //IN4 //RB //swapped 9 and 6 do in hardware  

#define CW  1
#define ACW 0
unsigned long last1 = 0 , last2 = 0 ;
uint32_t now;
int val = 0;

MPU6050 mpu(Wire);

float angle;

//void stepIn(bool);

void setup() {
  Serial.begin(9600);
  BTSerial.begin(9600);
  while(!Serial);
  for(int i =0; i<50;i++ ){
    Serial.print('.');
    delay(300);
  }
  Wire.begin();
  mpu.begin();
  mpu.calcGyroOffsets(true, 10, 10);
  BTSerial.println(mpu.getGyroZoffset());
}

void loop(){
  bool dir = CW;
  while(true){
    //Serial.println(".");
    now = millis();
    if(now - last1 > 5000){
      dir ^= 1;
      stepIn(dir);
      last1 = now;
    }
    if(now - last2 > 10){
      mpu.update();
   /*   Serial.println(millis());
      Serial.print(",");
      Serial.print(mpu.getAngleZ());
      Serial.print(",");
      Serial.print(int((dir)?6:-6));
    */  BTSerial.print(millis());
      BTSerial.print(",");
      BTSerial.print(mpu.getAngleZ());
      BTSerial.print(",");
      BTSerial.println(int((dir)?6:-6));
      last2 = now;
    }
    //delay(5);
  }
}

void stepIn(bool direct){
  if(direct == CW){
    Serial.print("CW");
    digitalWrite(MOT1, 1);
    digitalWrite(MOT2, 0);
    digitalWrite(MOT3, 0);
    digitalWrite(MOT4, 1);
  }
  else{
    Serial.print("ACW");
    digitalWrite(MOT1, 0);
    digitalWrite(MOT2, 1);
    digitalWrite(MOT3, 1);
    digitalWrite(MOT4, 0);
  }
}
