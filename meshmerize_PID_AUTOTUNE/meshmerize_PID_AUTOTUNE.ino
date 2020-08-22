//PID LINE FOLLOWER 
//(imagine that you are driving the bot for right-left convention)

// use MEGA
/* on motor driver
 *  M1 left motor red
 *  M2
 *  M3 right motor red
 *  M4
 *  I1 11 M1
 *  I2 9
 *  I3 right motor
 *  I4
 */
#include <PID_AutoTune_v0.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

//motor driver-pins must be pwm to control speed!!!

#define i1 11 //right forward
#define i2 9  //right backward
#define i3 5  //left forward
#define i4 6  //left backward 
 
//IR ARRAY
#define  sensor1 A0     // RIGHT most sensor
#define  sensor2 A1     
#define  sensor3 A2 
#define  sensor4 A3    
#define  sensor5 A4     // LEFT most sensor
//#define  sensorL A5
//#define  sensorR 10
#define LED 12

MPU6050 mpu6050(Wire);

bool ir[5];  //ir array values

// PID Constants
float Kp=19.0; //30 // 19
float Ki=0.01;
float Kd=31.0; //50.0;  //31.05

//unsigned long last_time = millis();

bool runBot = true;

double error=0, previous_error = 0, PID_value = 0;

double setpoint=0;

double outputStart=0;
double AutoTuneStep=5, AutoTuneNoise=1, AutoTuneStartValue=0;
unsigned int AutoTuneLookBack=10, P = 0, I = 0, D = 0;

unsigned long SampleTime = 100, last_time; 

boolean tuning = 1;

float zAngle, initZAngle;

void calculate_PID();

uint8_t ir_val;

PID_ATune AutoTune(&error, &PID_value);


void setup() {
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
  
  Serial.begin(9600);
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
  }
  digitalWrite(i1,0);
  digitalWrite(i2,1);
  digitalWrite(i3,1);
  digitalWrite(i4,0);
  delay(250);
  last_time = millis();
  
}


void loop() {
  if(runBot){
    mpu6050.update();
    zAngle = mpu6050.getAngleZ();//- initZAngle;
    error = zAngle;
    Serial.println(zAngle);
    //find_position();
    //error_val();
    //calculate_PID();
    if(tuning){
      byte val = (AutoTune.Runtime());
      if (val != 0){
        tuning = false;
      }
      if(!tuning){ //we're done, set the tuning parameters
        Kp = AutoTune.GetKp();
        Ki = AutoTune.GetKi();
        Kd = AutoTune.GetKd();
        digitalWrite(LED, HIGH);
        runBot = false;
      }
    }
    else calculate_PID();
    if(error > 0){
      if (PID_value < 0) PID_value = PID_value * -1;
      analogWrite(i1,PID_value);
      analogWrite(i2,0);
      analogWrite(i3,0);
      analogWrite(i4,PID_value);
    }
    if(error < 0){
      if (PID_value < 0) PID_value = PID_value * -1;
      //Serial.println(s+PID_value);
      analogWrite(i1,0);
      analogWrite(i2,PID_value);  
      analogWrite(i3,PID_value);
      analogWrite(i4,0);
    }
    if(error == 0){
      if (PID_value < 0) PID_value = PID_value * -1;
      //Serial.println(s+PID_value);
      analogWrite(i1,0);
      analogWrite(i2,0);
      analogWrite(i3,0);
      analogWrite(i4,0);
    }
  }
  else {
    //SerialReceive();
    //SerialSend();
  }
}


void find_position(){ 
 
  ir[0] = digitalRead(sensor1); // Right Most
  ir[0]=(!ir[0]);
  //Serial.print("1 : ");
  //Serial.println(ir[1]);
  ir[1] = digitalRead(sensor2); 
  ir[1]=(!ir[1]);
  //Serial.print("2 : ");
  //Serial.println(ir[2]);
  ir[2] = digitalRead(sensor3);
  ir[2]=(!ir[2]);
  //Serial.print("3 : ");
  //Serial.println(ir[3]);
  ir[3] = digitalRead(sensor4);
  ir[3]=(!ir[3]);
  //Serial.print("4 : ");
  //Serial.println(ir[4]);
  ir[4] = digitalRead(sensor5); // Left Most
  ir[4]=(!ir[4]);
  ir_val = ir[4]<<4 | ir[3]<<3 | ir[2]<<2 | ir[1]<<1 | ir[0];
  Serial.print("ir_val: ");
  Serial.println(ir_val);
}


void error_val(void){
  if      (ir_val == 0b10000){
    error = 4;
  }
  else if (ir_val == 0b01000){
    error = 2;
  }
  else if (ir_val == 0b00100){
    error = 0;
  }
  else if (ir_val == 0b00010){
    error = -2;
  }
  else if (ir_val == 0b00001){
    error = -4;
  }
  else if (ir_val == 0b11000){
    error = 3;
  }
  else if (ir_val == 0b01100){
    error = 0.8;
  }
  else if (ir_val == 0b00110){
    error = -0.8;
  }
  else if (ir_val == 0b00011){
    error = -3;
  }
  /*if      (!ir[0] && !ir[1] && !ir[2] && !ir[3] && ir[4]){
    error = -4;
  }
  else if (!ir[1] && !ir[2] && !ir[3] && ir[4] && !ir[5]){      < -Slightly wrong sub 1 from array values
    error = -2;//-2
  }
  else if (!ir[1] && !ir[2] && ir[3] && !ir[4] && !ir[5]){
    error = 0;
  }
  else if (!ir[1] && ir[2] && !ir[3] && !ir[4] && !ir[5]){
    error = 2; //2
  }
  else if ((ir[1] == 1) && (!ir[2] == 0) && (!ir[3] == 0) && (!ir[4] == 0) && (!ir[5] == 0)){
    error = 4;
  }
  else if ((!ir[1] == 0) && (!ir[2] == 0) && (!ir[3] == 0) && (ir[4] == 1) && (ir[5] == 1)){
    error = -3;
  }
  else if ((!ir[1] == 0) && (!ir[2] == 0) && (ir[3] == 1) && (ir[4] == 1) && (!ir[5] == 0)){
    error = -0.8;
  }
  else if ((!ir[1] == 0) && (ir[2] == 1) && (ir[3] == 1) && (!ir[4] == 0) && (!ir[5] == 0)) { 
    error = 0.8;
  }
  else if ((ir[1] == 1) && (ir[2] == 1) && (!ir[3] == 0) && (!ir[4] == 0) && (!ir[5] == 0)) { 
    error = 3;
  }*/
  
}

void calculate_PID(){
  unsigned long now = millis();
  if(last_time+SampleTime<=now){
    P = setpoint + error;   // error
    if(I>240){
      I=240;
    }
    if(I<-240){
      I=-240;
    }
    I += Ki * error;
    D = previous_error - error;
    previous_error = error;
    PID_value = int(P*Kp + I - D*Kd);
    PID_value = constrain(PID_value, 0 ,255);
    last_time = now;
  }
}

void run_mode(){
  runBot = true;
}

void changeAutoTune()
{
  if(!tuning)
  {
    //Set the output to the desired starting frequency.
    PID_value=AutoTuneStartValue;
    AutoTune.SetNoiseBand(AutoTuneNoise);
    AutoTune.SetOutputStep(10);
    AutoTune.SetOutputStep(AutoTuneStep);
    AutoTune.SetLookbackSec((int)AutoTuneLookBack);
    tuning = true;
  }
  else
  { //cancel autotune
    AutoTune.Cancel();
    tuning = false;
  }
}

void SerialSend()
{
  Serial.print("setpoint: ");Serial.print(setpoint); Serial.print(" ");
  Serial.print("input: ");Serial.print(error); Serial.print(" ");
  //Serial.print("output: ");Serial.print(); Serial.print(" ");
  if(tuning){
    Serial.println("tuning mode");
  } 
  else {
    Serial.print("kp: ");Serial.print(Kp);Serial.print(" ");
    Serial.print("ki: ");Serial.print(Ki);Serial.print(" ");
    Serial.print("kd: ");Serial.print(Kd);Serial.println();
  }
}

void SerialReceive()
{
  while(!Serial.available());
  if(Serial.available())
  {
   char b = Serial.read(); 
   Serial.flush(); 
   if((b=='1' && !tuning) || (b!='1' && tuning))changeAutoTune();   // 1 typed in serial monitor means it will enter tuning mode
  }
}
