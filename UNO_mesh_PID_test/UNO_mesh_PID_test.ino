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
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
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
 
float ir[5];  //ir array values

// PID Constants
float Kp=19.0; //30 // 19
float Ki=0.00;
float Kd=31.0; //50.0;  //31.05

//unsigned long last_time = millis();

bool runBot = true;

byte ATuneModeRemember=2;
double input=0, previous_error = 0, output= 0;

double setpoint=0;

double outputStart=0;
double AutoTuneStep=50, AutoTuneNoise=1, AutoTuneStartValue=5;
unsigned int AutoTuneLookBack=20;
int  P = 0, I = 0, D = 0;

unsigned long SampleTime = 100, last_time; 

boolean tuning = true;


void calculate_PID();


PID controller(&input, &output, &setpoint,Kp,Ki,Kd, DIRECT);
PID_ATune AutoTune(&input, &output);


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
  
  //Setup the pid 
  controller.SetMode(AUTOMATIC);
  
  if(tuning){
    tuning=false;
    changeAutoTune();
    tuning=true;
  }
  Serial.begin(9600);
  delay(500);
  Serial.println("---DONE SETUP---");
      
}


void loop() {
  if(runBot){
    find_position();
    error_val();
    //calculate_PID();
   
    if(tuning){
      byte val = (AutoTune.Runtime());
      Serial.println("----------------Runtime------------");
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
    else controller.Compute();
    
    if(input > 0){
      //if (output< 0) output= output* -1;
      //Serial.println(s+PID_value);
      analogWrite(i1,output);
      analogWrite(i2,0);
      analogWrite(i3,0);
      analogWrite(i4,0);
    }
    if(input < 0){
      //if (output< 0) output= output* -1;
      //Serial.println(s+PID_value);
      analogWrite(i1,0);
      analogWrite(i2,0);  
      analogWrite(i3, output);
      analogWrite(i4,0);
    }
    if(input == 0){
      //if (output< 0) output= output* -1;
      //Serial.println(s+PID_value);
      analogWrite(i1,output);
      analogWrite(i2,0);
      analogWrite(i3,output);
      analogWrite(i4,0);
    }
    Serial.print("\t\t\t\t\tOutput: ");
    Serial.println(output);
  }
  else {
    SerialReceive();
    SerialSend();
  }
}


void find_position(){ 
 
  ir[1] = digitalRead(sensor1);
  //ir[1]=(!ir[1]);
  Serial.print("1 : ");
  Serial.println(ir[1]);
  ir[2] = digitalRead(sensor2);
  //ir[2]=(!ir[2]);
  Serial.print("2 : ");
  Serial.println(ir[2]);
  ir[3] = digitalRead(sensor3);
  //ir[3]=(!ir[3]);
  Serial.print("3 : ");
  Serial.println(ir[3]);
  ir[4] = digitalRead(sensor4);
  //ir[4]=(!ir[4]);
  Serial.print("4 : ");
  Serial.println(ir[4]);
  ir[5] = digitalRead(sensor5);
  //ir[5]=(!ir[5]);
  Serial.print("5 : ");
  Serial.println(ir[5]);
  Serial.println("\n\n");
}


void error_val(void){
  if ((ir[1] == 0) && (ir[2] == 0) && (ir[3] == 0) && (ir[4] == 0) && (ir[5] == 1)){
    input = -4;
  }
  else if ((ir[1] == 0) && (ir[2] == 0) && (ir[3] == 0) && (ir[4] == 1) && (ir[5] == 0)){
    input = -2;//-2
  }
  else if ((ir[1] == 0) && (ir[2] == 0) && (ir[3] == 1) && (ir[4] == 0) && (ir[5] == 0)){
    input = 0;
  }
  else if ((ir[1] == 0) && (ir[2] == 1) && (ir[3] == 0) && (ir[4] == 0) && (ir[5] == 0)){
    input = 2; //2
  }
  else if ((ir[1] == 1) && (ir[2] == 0) && (ir[3] == 0) && (ir[4] == 0) && (ir[5] == 0)){
    input = 4;
  }
  else if ((ir[1] == 0) && (ir[2] == 0) && (ir[3] == 0) && (ir[4] == 1) && (ir[5] == 1)){
    input = -3;
  }
  else if ((ir[1] == 0) && (ir[2] == 0) && (ir[3] == 1) && (ir[4] == 1) && (ir[5] == 0)){
    input = -0.8;
  }
  else if ((ir[1] == 0) && (ir[2] == 1) && (ir[3] == 1) && (ir[4] == 0) && (ir[5] == 0)) { 
    input = 0.8;
  }
  else if ((ir[1] == 1) && (ir[2] == 1) && (ir[3] == 0) && (ir[4] == 0) && (ir[5] == 0)) { 
    input = 3;
  }
}

void calculate_PID(){
  unsigned long now = millis();
  if(last_time+SampleTime>=now){
    P = setpoint + input;   // error
    if(I>240){
      I=240;
    }
    if(I<-240){
      I=-240;
    }
    I += Ki * input;
    D = previous_error - input;
    previous_error = input;
    output= int(P*Kp + I - D*Kd);
    output= constrain(output, 0 ,255);
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
    output=AutoTuneStartValue;
    AutoTune.SetNoiseBand(AutoTuneNoise);
    AutoTune.SetOutputStep(AutoTuneStep);
    AutoTune.SetLookbackSec((int)AutoTuneLookBack);
    tuning = true;
    AutoTuneHelper(true);
  }
  else
  { //cancel autotune
    AutoTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(boolean start)
{
  if(start)
    ATuneModeRemember = controller.GetMode();
  else
    controller.SetMode(ATuneModeRemember);
}

void SerialSend()
{
  Serial.print("setpoint: ");Serial.print(setpoint); Serial.print(" ");
  Serial.print("input: ");Serial.print(input); Serial.print(" ");
  Serial.print("output: ");Serial.print(output); Serial.print(" ");
  if(tuning){
    Serial.println("tuning mode");
  } 
  else {
    Serial.print("kp: ");Serial.print(Kp);Serial.print(" ");
    Serial.print("ki: ");Serial.print(Ki);Serial.print(" ");
    Serial.print("kd: ");Serial.print(Kd);Serial.println();
  }
  delay(5000);
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
