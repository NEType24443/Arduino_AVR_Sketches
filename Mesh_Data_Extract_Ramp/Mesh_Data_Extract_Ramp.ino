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

unsigned long last1 = 0, last2 = 0;
uint32_t now = millis();

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
  bool increasing = true;
  int val = 0;
  last1 = millis();
  last2 = last1;
  while(true){
    now = millis();
    if(now - last1 > 100){
      rampIn( dir, val);
      (increasing)? val++ : val--;
      //Serial.println(val);
      if(val>=255){
        increasing ^= true;
      }
      if(val == 0 && !increasing){
        //Serial.println("ClockWise");
        dir = ACW;
      }
      if(val == 0 && increasing){
        //Serial.println("AntiClockWise");
        dir = CW;
      }
      if(val <= 0){
        increasing ^= true;
        //Serial.println("________Increasing :");
        //Serial.print((increasing)?"true_________":"false_________");
      }
      last1 = now;
    }
    if(now - last2 > 10){
      mpu.update();
    /*  Serial.println(millis());
      Serial.print(",");
      Serial.print(mpu.getAngleZ());
      Serial.print(",");
      Serial.println(float((val*((dir)?6:-6)))/255);
    */  BTSerial.print(millis());
      BTSerial.print(",");
      BTSerial.print(mpu.getAngleZ());
      BTSerial.print(",");
      BTSerial.println(float((val*((dir)?6:-6)))/255);
      last2 = now;
    }
    //delay(5);
  }
}
void rampIn(bool direct, int val){
  if(direct == CW){
    analogWrite(MOT1, val);
    digitalWrite(MOT2, 0);
    digitalWrite(MOT3, 0);
    analogWrite(MOT4, val);
  }
  else{
    digitalWrite(MOT1, 0);
    analogWrite(MOT2, val);
    analogWrite(MOT3, val);
    digitalWrite(MOT4, 0);
  }
}
