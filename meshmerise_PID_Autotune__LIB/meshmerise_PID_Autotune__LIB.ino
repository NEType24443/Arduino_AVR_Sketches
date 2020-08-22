#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

//motor driver-pins must be pwm to control speed!!!

// On the Arduino board, digital pins are also used
// for the analog output (software PWM).  Analog input
// pins are a separate set.

// ATMEL ATMEGA8 & 168 / ARDUINO
//
//                  +-\/-+
//            PC6  1|    |28  PC5 (AI 5)
//      (D 0) PD0  2|    |27  PC4 (AI 4)
//      (D 1) PD1  3|    |26  PC3 (AI 3)
//      (D 2) PD2  4|    |25  PC2 (AI 2)
// PWM+ (D 3) PD3  5|    |24  PC1 (AI 1)
//      (D 4) PD4  6|    |23  PC0 (AI 0)
//            VCC  7|    |22  GND
//            GND  8|    |21  AREF
//            PB6  9|    |20  AVCC
//            PB7 10|    |19  PB5 (D 13)
// PWM+ (D 5) PD5 11|    |18  PB4 (D 12)
// PWM+ (D 6) PD6 12|    |17  PB3 (D 11) PWM
//      (D 7) PD7 13|    |16  PB2 (D 10) PWM
//      (D 8) PB0 14|    |15  PB1 (D 9) PWM
//                  +----+
//
// (PWM+ indicates the additional PWM pins on the ATmega168.)

#define i1  11 //IN1 //LF
#define i2  9 //IN2 //LB
#define i3  5 //In3 //RF
#define i4  6 //IN4 //RB //swapped 9 and 6 do in hardware  

//IR ARRAY
#define  sensor1 A0     // RIGHT most sensor
#define  sensor2 A1     
#define  sensor3 A2 
#define  sensor4 A3    
#define  sensor5 A4     // LEFT most sensor
//#define  sensorL A5
//#define  sensorR 10
#define LED 12

#define THRESH 1.00

MPU6050 mpu6050(Wire);

float ir[5];  //ir array values

byte ATuneModeRemember=2;
double input=0, output=0, setpoint=0;
double kp=1.9, ki=0.20,kd=0.05;

double outputStart=5;
double aTuneStep=50, aTuneNoise=1, aTuneStartValue=100;
unsigned int aTuneLookBack=20;
unsigned long serialTime, lastTime;

float zAngle, initZAngle, P, I, D;

int previousError;

boolean tuning = false;

//PID myPID(&input, &output, &setpoint,kp,ki,kd ,P_ON_M, DIRECT);
PID_ATune aTune(&input, &output);

void setup(){
  //Setup the pid 
  //myPID.SetMode(AUTOMATIC);
  //myPID.SetOutputLimits(0,255);
  //myPID.SetSampleTime(100);
  serialTime = 5000;
  setpoint = 0.00;
//motor driver
  pinMode(i1,OUTPUT);
  pinMode(i2,OUTPUT);
  pinMode(i3,OUTPUT);
  pinMode(i4,OUTPUT);

// ir array
  pinMode(sensor1,INPUT);
  pinMode(sensor2,INPUT);
  pinMode(sensor3,INPUT);
  pinMode(sensor4,INPUT);
  pinMode(sensor5,INPUT);
  //pinMode(sensorL, INPUT);
  //pinMode(sensorR, INPUT);
  //pinMode(BUTTON, INPUT);
  
  Serial.begin(115200);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  initZAngle = mpu6050.getAngleZ();
  Serial.print(initZAngle);
  delay(500);
  if(tuning){
    tuning = false;
    changeAutoTune();
    tuning = true;
  }/*
  digitalWrite(i1,0);
  digitalWrite(i2,1);
  digitalWrite(i3,1);
  digitalWrite(i4,0);
  delay(250);
  Serial.begin(115200);*/
  lastTime = millis();
}

void loop()
{
  unsigned long now = millis();
  mpu6050.update();
  zAngle = mpu6050.getAngleZ();
  input = zAngle + 0.00;//- initZAngle;
  
  if(tuning){
    byte val = (aTune.Runtime());
    if (val!=0){
      tuning = false;
    }
    if(!tuning){ //we're done, set the tuning parameters
      kp = aTune.GetKp();
      ki = aTune.GetKi();
      kd = aTune.GetKd();
      //myPID.SetTunings(kp,ki,kd);
    }
      AutoTuneHelper(false);
  }
  else Compute();
  if(now-lastTime>100){
    if(zAngle > THRESH){
      //if (output < 0) output *= -1;
      //Serial.println(s+output);
      analogWrite(i1,output);
      digitalWrite(i2,0);
      digitalWrite(i3,0);
      analogWrite(i4,output);
    }
    if(zAngle < -THRESH){
      //if (output < 0) output *= -1;
      //Serial.println(s+output);
      digitalWrite(i1,0);
      analogWrite(i2,output);  
      analogWrite(i3,output);
      digitalWrite(i4,0);
    }
    if((-THRESH < zAngle) && (zAngle < THRESH)){
      //if (output < 0) output *= -1;
      //Serial.println(s+output);
      digitalWrite(i2,0);
      digitalWrite(i3,0);
      digitalWrite(i1,0);
      digitalWrite(i4,0);
    }
  }
  //send-receive with processing if it's time
  if(millis()>serialTime){
    //SerialReceive();
    SerialSend();
    serialTime+=200;
  }
}

void changeAutoTune(){
 if(!tuning){
    //Set the output to the desired starting frequency.
    output=aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  }
  else{ //cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(boolean start){
  if(start){}
//    ATuneModeRemember = myPID.GetMode();
  else{}
//    myPID.SetMode(ATuneModeRemember);
}

void Compute(){
  //unsigned long now = mills();
  unsigned long now = millis();
  if(lastTime+10>=now){
    float error = setpoint - input;
    P = error;
    I += ki * error;
    D =  previousError - error;
    previousError = error;
    output = int(P*kp + I - D*kd);
    output = constrain(output,-255,255);
    lastTime = now;
  }
}

void SerialSend(){
  Serial.print("setpoint: ");Serial.print(setpoint); Serial.print(" ");
  Serial.print("input: ");  Serial.print(input); Serial.print(" ");
  Serial.print("output: ");Serial.print(output); Serial.println();
  if(tuning){
    Serial.println("tuning mode");
  } else {
//    Serial.print("kp: ");Serial.print(myPID.GetKp());Serial.print(" ");
//    Serial.print("ki: ");Serial.print(myPID.GetKi());Serial.print(" ");
//    Serial.print("kd: ");Serial.print(myPID.GetKd());Serial.println();
  }
}

void SerialReceive(){
  if(Serial.available()){
   char b = Serial.read(); 
   Serial.flush(); 
   if((b=='1' && !tuning) || (b!='1' && tuning))changeAutoTune();
  }
}
