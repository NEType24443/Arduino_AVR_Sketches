//PID LINE FOLLOWER 
//(imagine th at you are driving the bot for right-left convention)

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

byte ATuneModeRemember=2;
double input=80, output=50, setpoint=180;
double kp=2,ki=0.5,kd=2;

double kpmodel=1.5, taup=100, theta[50];
double outputStart=5;
double aTuneStep=0.5, aTuneNoise=1, aTuneStartValue=0;
unsigned int aTuneLookBack=20;

boolean tuning = false;
unsigned long  modelTime, serialTime;

PID_ATune aTune(&input, &output);

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
#define  sensorL A5
#define  sensorR 10

//BUTTON,LED
#define BUTTON 3  //Interrupt 
#define LED 12

#define L 1
#define S 2
#define R 3
//#define U 4

//#define Sample_Time = 1;
     
int  inter[50]={0}, dir[50] = {0};
int inter_num = 0 ; // keeps track of directions
 
float ir[6];  //ir array values

int s=90;//fixed speed

// PID Constants
float Kp=0;//30 // 19
float Ki=0.00;
float Kd=0;//50.0;  //31.05


//unsigned long last_time = millis();
 
float sum;

float pos;    //instantaneous position
//float setpoint;
float PID_value = 0;

bool runMode = false;
bool pathErrorFlag = false;
bool runBot = true;

void calculate_PID();
void forward();
void stop_();
void left_turn(int);
void right_turn(int);
void run_mode();
void path_follow();

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
  attachInterrupt(digitalPinToInterrupt(BUTTON), run_mode, FALLING);
  Serial.begin(115200);

  setpoint=0 ;
  if(tuning)
  {
    tuning=false;
    changeAutoTune();
    tuning=true;
  }
  
  serialTime = 0;
  Serial.begin(9600);
  delay(500);
}


float error=0, previous_error = 0, P = 0, I = 0, D = 0;

void loop() {
  if(runBot){
    find_position();
    if(!runMode){
      direction_decoder();
    }
    else{
      path_follow();
    }
    calculate_PID();
    if(error>0){
      if (PID_value < 0) PID_value = PID_value * -1;
      //Serial.println(s+PID_value);
      analogWrite(i1,s+PID_value);
      analogWrite(i2,0);
      analogWrite(i3,0);//sminor
      analogWrite(i4,0);
    }
    if(error<0){
      if (PID_value < 0) PID_value = PID_value * -1;
      //Serial.println(s+PID_value);
      analogWrite(i1,0);  //sminor
      analogWrite(i2,0);  
      analogWrite(i3,s+PID_value);
      analogWrite(i4,0);
    }
    if(error==0){
      if (PID_value < 0) PID_value = PID_value * -1;
      //Serial.println(s+PID_value);
      analogWrite(i1,s);
      analogWrite(i2,0);
      analogWrite(i3,s);
      analogWrite(i4,0);
    }
    delay(100);
    forward();
    if(tuning)
  {
    byte val = (aTune.Runtime());
    if (val!=0)
    {
      tuning = false;
    }
    if(!tuning)
    { //we're done, set the tuning parameters
      kp = aTune.GetKp();
      ki = aTune.GetKi();
      kd = aTune.GetKd();
      myPID.SetTunings(kp,ki,kd);
      AutoTuneHelper(false);
    }
  }
  else calculate_PID();
  
  }
  else{
    stop_();
  }
}


void find_position(){ 
 
  ir[1] = digitalRead(sensor1);
  ir[1]=(!ir[1]);
  //Serial.print("1 : ");
  //Serial.println(ir[1]);
  ir[2] = digitalRead(sensor2);
  ir[2]=(!ir[2]);
  //Serial.print("2 : ");
  //Serial.println(ir[2]);
  ir[3] = digitalRead(sensor3);
  ir[3]=(!ir[3]);
  //Serial.print("3 : ");
  //Serial.println(ir[3]);
  ir[4] = digitalRead(sensor4);
  ir[4]=(!ir[4]);
  //Serial.print("4 : ");
  //Serial.println(ir[4]);
  ir[5] = digitalRead(sensor5);
  ir[5]=(!ir[5]);
  //Serial.print("5 : ");
  //Serial.println(ir[5]);
  ir[6] = digitalRead(sensorL);
  ir[6]=(!ir[6]);
  //Serial.print("L : ");
  //Serial.println(ir[6]);
  ir[7] = digitalRead(sensorR);
  ir[7]=(!ir[7]);
  //Serial.print("R : ");
  //Serial.println(ir[7]);
}


void direction_decoder(void){
  //int flag = false;
  /*if ((ir[7] == 0) && (ir[1] == 1) && (ir[2] == 1) && (ir[3] == 1)  && (ir[6] == 0)){//R - MAIN
    error = 0;
    stop_();
    delay(1000);
    Serial.println("R-Main");
    forward();
    delay(100);
    //while((!digitalRead(sensor2))||(!digitalRead(sensor3))||(!digitalRead(sensor4)));
    stop_();
    delay(100);
    find_position();
    if ((ir[1] == 1) && (ir[2] == 1) && (ir[3] == 1) && (ir[4] == 1) && (ir[5] == 1)){// T
      //stop_();
      //delay(50);
      forward();
      delay(100);
      find_position();
      //flag = true;
      if ((ir[7] == 1) && (ir[1] == 1) && (ir[2] == 1) && (ir[3] == 1) && (ir[4] == 1) && (ir[5] == 1) && (ir[6] == 1)){// END POINT
        stop_();
        digitalWrite(LED, HIGH);
        runBot = false;
        Serial.println("R-Main -END");
      }
      else if((ir[7] == 1) && (ir[3] == 0) && (ir[6] == 1)){//left_turn replaced
        forward();
        delay(200);
        left_turn(400);
        Serial.println("R-Main - T-LEFT");
        inter[inter_num] += L;
        inter_num++;
      }
    }
    else{
      forward();
      delay(100);
      stop_();
      delay(1000);
      find_position();
      if ((ir[7] == 1) && (ir[1] == 0) && (ir[2] == 0) && ((ir[5] == 1)||(ir[4] == 1))){//R-LA 
        delay(100);
        Serial.println("R-Main R- LA");
        left_turn(200);
        inter[inter_num] += L;
        inter_num++;
      }
      else if (((ir[6] == 1)||(ir[5] == 1)) && (ir[3] == 0) && (ir[1] == 1)){//L-RA
        forward();
        delay(200);
        left_turn(400);
        Serial.println("R-Main - L-RA");
        inter[inter_num] += L;
        inter_num++;
      }
      /*else if ((ir[1] == 1) && (ir[4] == 0) && (ir[5] == 0)){//R-RA
        
      }*/
/*      else if ((ir[6] == 0) &&( (ir[2] == 1)||(ir[3] == 1)||(ir[4] == 1)) && (ir[7] == 1)){//R-ST
        Serial.println("R-Main - ST");
        inter[inter_num] += S;
        inter_num++;
        delay(100);
      }
      else if(ir[7]==1){
        Serial.println("R-Main - R");
        delay(200);
        right_turn(400);
      }
       else if(ir[6] == 1){//L
        Serial.println("R-Main - L");
        delay(200);
        left_turn(400);
       }
    }
  }
  else if ((ir[7] == 0)  && (ir[3] == 1) && (ir[4] == 1) && (ir[5] == 1) && (ir[6] == 0)){//L - MAIN
    error = 0;
    stop_();
    delay(1000);
    Serial.println("L-Main");
    forward();
    delay(100);
   // while((!digitalRead(sensor2))||(!digitalRead(sensor3))||(!digitalRead(sensor4)));
    stop_();
    delay(100);
    find_position();
    if ((ir[2] == 1) && (ir[3] == 1) && (ir[4] == 1) && (ir[5] == 1) && (ir[1] == 1)){ // T
      //stop_();
      //delay(50);
      forward();
      delay(100);
      find_position();
      if ((ir[7] == 1) && (ir[1] == 1) && (ir[2] == 1) && (ir[3] == 1) && (ir[4] == 1) && (ir[5] == 1) && (ir[6] == 1)){// END POINT
        stop_();
        digitalWrite(LED, HIGH);
        runBot = false;
        Serial.println("L-Main - END");
      }
      else if((ir[7] == 1) && (ir[3] == 0) && (ir[6] == 1)){
        forward();
        delay(200);
        left_turn(400);
        Serial.println("L-Main- T-LEFT");
        inter[inter_num] += L;
        inter_num++;
      }
    }
    else{
      forward();
      delay(100);
      //stop_();
      //delay(1000);
      //flag = false;
      find_position();
      /*if ((ir[1] == 0) && (ir[2] == 0) && (ir[5] == 1)){//L-LA
        //delay(50);
        left_turn(400);
      }*/
/*      if (((ir[6] == 1)||(ir[5] == 1)) && ((ir[1] == 1) || (ir[7] == 0))){//L-RA
        //forward();
        delay(200);
        Serial.println("L-Main- L-RA");
        left_turn(400);
        inter[inter_num] += L;
        inter_num++;
      }
      else if ((ir[7] == 1) && (ir[1] == 0) && (ir[2] == 0) && ((ir[5] == 1)||(ir[6] == 1))){//R-LA 
        delay(100);
        Serial.println("L-Main - R-LA");
        left_turn(200);
        inter[inter_num] += L;
        inter_num++;
      }
      else if (  ( (ir[2] == 1)||(ir[3] == 1)||(ir[4] == 1)) && (ir[6]==1)&&(ir[7]==0)){//L-ST
        //forward();
        delay(200);
        Serial.println("L-Main - L-ST");
        left_turn(400);
        inter[inter_num] += L;
        inter_num++;
      }
      else if(ir[6] == 1){//L
        Serial.println("L-Main - L");
        delay(200);
        left_turn(400);
      }
      else if(ir[7]==1){
        Serial.println("L-Main - R");
        delay(200);
        right_turn(400);
      }
    }
  }
  /*else if ((ir[1] == 1) && (ir[2] == 1) && (ir[3] == 1) && (ir[4] == 1) && (ir[5] == 1)){
    error = 0;
    delay(0);
    stop_();
    delay(50);
    left_turn(400);
  }*/
/*  else if (((ir[1] == 1) || (ir[2] == 1)) &&(ir[3] == 0) && ((ir[4] == 1) || (ir[5] == 1))){ // Y- Intersection
    stop_();
    delay(400);
    forward();
    delay(200);
    stop_();
    delay(400);
    find_position();
    Serial.println("Y-Main");
    if(((ir[7] == 1)||(ir[1] == 1)) && (ir[3] == 0) && ((ir[5] == 1)||(ir[6] == 1))){  //Y
      forward();
      delay(200);
      left_turn(200);
      Serial.println("Y-MAIN - LA");
      inter[inter_num] += L;
      inter_num++;
    }
    else if((ir[3] == 1) && (ir[7] == 1)){  // R-ST
      forward();
      Serial.println("Y-MAIN - R - ST");
      inter[inter_num] += S;
      inter_num++;
      delay(50);
    }
    else if((ir[3] == 1) && (ir[6] == 1)){  // L-ST
      forward();
      delay(300);
      left_turn(400);
      Serial.println("Y-MAIN - L - ST");
      inter[inter_num] += L;
      inter_num++;
    }
  }*/
  else if ((ir[1] == 0) && (ir[2] == 0) && (ir[3] == 0) && (ir[4] == 0) && (ir[5] == 1)){
    error = -4;
  }
  else if ((ir[1] == 0) && (ir[2] == 0) && (ir[3] == 0) && (ir[4] == 1) && (ir[5] == 0)){
    error = -2;//-2
  }
  else if ((ir[1] == 0) && (ir[2] == 0) && (ir[3] == 1) && (ir[4] == 0) && (ir[5] == 0)){
    error = 0;
  }
  else if ((ir[1] == 0) && (ir[2] == 1) && (ir[3] == 0) && (ir[4] == 0) && (ir[5] == 0)){
    error = 2; //2
  }
  else if ((ir[1] == 1) && (ir[2] == 0) && (ir[3] == 0) && (ir[4] == 0) && (ir[5] == 0)){
    error = 4;
  }
  else if ((ir[1] == 0) && (ir[2] == 0) && (ir[3] == 0) && (ir[4] == 1) && (ir[5] == 1)){
    error = -3;
  }
  else if ((ir[1] == 0) && (ir[2] == 0) && (ir[3] == 1) && (ir[4] == 1) && (ir[5] == 0)){
    error = -0.8;
  }
  else if ((ir[1] == 0) && (ir[2] == 1) && (ir[3] == 1) && (ir[4] == 0) && (ir[5] == 0)) { 
    error = 0.8;
  }
  else if ((ir[1] == 1) && (ir[2] == 1) && (ir[3] == 0) && (ir[4] == 0) && (ir[5] == 0)) { 
    error = 3;
  }
/*  else if ((ir[7] == 0) && (ir[1] == 0) && (ir[2] == 0) && (ir[3] == 0) && (ir[4] == 0) && (ir[5] == 0)&& (ir[6] == 0)){
    error = 0;
    stop_();
    delay(100);
    u_turn();
    //dir[dir_num] = U;
    //inter[inter_num] = 0
    inter_num--;
    //dir_num++;
  }
  */
}


//-------------------------------------------------PATH----------------------------------------//


void path_follow(){
  if ((ir[1] == 1) && (ir[2] == 1) && (ir[3] == 1) && (ir[5] == 0)){//R - MAIN
    error = 0;
    stop_();
    delay(1000);
    forward();
    delay(100);
    find_position();
    if ((ir[1] == 1) && (ir[2] == 1) && (ir[3] == 1) && (ir[4] == 1) && (ir[5] == 1)){// T
      stop_();
      delay(50);
      if(inter[inter_num]==L){
        left_turn(400);
      }
      else if(inter[inter_num]==R){
        right_turn(400);
      }
      else{
        pathErrorFlag = true;
        path_error();
      }
      inter_num++;
    }
    else{
      //forward();
      delay(100);
      stop_();
      delay(1000);
      find_position();
      if ((ir[1] == 0) && (ir[2] == 0) && (ir[5] == 1)){//R-LA 
        delay(50);
        if(inter[inter_num]==L){
          left_turn(200);  
        }
        else if(inter[inter_num]==R){
          right_turn(400);
        }
        else{
        pathErrorFlag = true;
          path_error();
        }
        inter_num++;
      }
      /*else if ((ir[1] == 1) && (ir[4] == 0) && (ir[5] == 0)){//R-RA
        
      }*/
      else if ((ir[1] == 0) &&( (ir[2] == 1)||(ir[3] == 1)||(ir[4] == 1)) && (ir[5] == 0)){//R-ST
        if(inter[inter_num]==S){
          forward();
        }
        else if(inter[inter_num]==R){
          right_turn(400);
        }
        else{
          pathErrorFlag = true;
          path_error();
        }
        inter_num++;
        delay(50);
      }
      else {//R
        //delay(50);
        right_turn(400);
      }
    }
  }
  else if ((ir[1] == 0) && (ir[3] == 1) && (ir[4] == 1) && (ir[5] == 1)){//L - MAIN
    error = 0;
    stop_();
    delay(1000);
    forward();
    delay(100);
    find_position();
    if ((ir[2] == 1) && (ir[3] == 1) && (ir[4] == 1) && (ir[5] == 1) && (ir[1] == 1)
    ){ // T
      stop_();
      delay(50);
      if(inter[inter_num]==L){
        left_turn(400);  
      }
      else if(inter[inter_num]==R){
        right_turn(400);
      }
      else{
        path_error();
      }
      inter_num++;
      find_position();
    }
    else{
      //forward();
      delay(100);
      stop_();
      delay(1000);
      find_position();
      /*if ((ir[1] == 0) && (ir[2] == 0) && (ir[5] == 1)){//L-LA
        //delay(50);
        left_turn(400);
      }*/
      if ((ir[1] == 1) && (ir[4] == 0) && (ir[5] == 0)){//L-RA
        delay(50);
        if(inter[inter_num]==L){
          left_turn(400);  
        }
        else if(inter[inter_num]==R){
          right_turn(200);
        }
        else{
          path_error();
        }
        inter_num++;
      }
      else if ((ir[1] == 0) && ( (ir[2] == 1)||(ir[3] == 1)||(ir[4] == 1)) && (ir[5] == 0)){//L-ST
        delay(50);
        if(inter[inter_num]==L){
          left_turn(400);  
        }
        else if(inter[inter_num]==S){
          forward();
        }
        else{
          path_error();
        }
        inter_num++;
      }
      else {//L
        delay(50);
        left_turn(400);
      }
    }
  }
  /*else if ((ir[1] == 1) && (ir[2] == 1) && (ir[3] == 1) && (ir[4] == 1) && (ir[5] == 1)){
    error = 0;
    delay(0);
    stop_();
    delay(50);
    left_turn(400);
  }*/
  else if ((ir[7] == 0)&& (ir[1] == 0) && (ir[2] == 1) && (ir[3] == 1) && (ir[4] == 1) && (ir[5] == 0) && (ir[6] == 0)){ // Y- Intersection
    //stop_();
    delay(100);
    //forward();
    //delay(50);
    stop_();
    delay(1000);
    find_position();
    if((ir[7] == 1) && (ir[3] == 0) && (ir[6] == 1)){
      if(inter[inter_num]==L){
          left_turn(200);  
      }
      else if(inter[inter_num]==R){
        right_turn(200);
      }
      else{
        path_error();
      }
      inter_num++;
    }
  }
  else if ((ir[1] == 0) && (ir[2] == 0) && (ir[3] == 0) && (ir[4] == 0) && (ir[5] == 1)){
    error = -4;
  }
  else if ((ir[1] == 0) && (ir[2] == 0) && (ir[3] == 0) && (ir[4] == 1) && (ir[5] == 0)){
    error = -2;//2
  }
  else if ((ir[1] == 0) && (ir[2] == 0) && (ir[3] == 1) && (ir[4] == 0) && (ir[5] == 0)){
    error = 0;
  }
  else if ((ir[1] == 0) && (ir[2] == 1) && (ir[3] == 0) && (ir[4] == 0) && (ir[5] == 0)){
    error = 2; //2
  }
  else if ((ir[1] == 1) && (ir[2] == 0) && (ir[3] == 0) && (ir[4] == 0) && (ir[5] == 0)){
    error = 4;
  }
  else if ((ir[1] == 0) && (ir[2] == 0) && (ir[3] == 0) && (ir[4] == 1) && (ir[5] == 1)){
    error = -3;
  }
  else if ((ir[1] == 0) && (ir[2] == 0) && (ir[3] == 1) && (ir[4] == 1) && (ir[5] == 0)){
    error = -0.8;
  }
  else if ((ir[1] == 0) && (ir[2] == 1) && (ir[3] == 1) && (ir[4] == 0) && (ir[5] == 0)) { 
    error = 0.8;
  }
  else if ((ir[1] == 1) && (ir[2] == 1) && (ir[3] == 0) && (ir[4] == 0) && (ir[5] == 0)) { 
    error = 3;
  }
  else if ((ir[1] == 0) && (ir[2] == 0) && (ir[3] == 0) && (ir[4] == 0) && (ir[5] == 0)){
    error = 0;
    stop_();
    delay(100);
    u_turn();
    path_error();
    //dir[dir_num] = U;
    //inter[inter_num] = 0
    //inter_num--;
    //dir_num++;
  }
}


void calculate_PID(){
  //unsigned long now = mills();
  //if(last_time+SampleTime>=now){
  P = error;
  if(I>240)
  {
    I=240;
  }
  if(I<-240)
  {
    I=-240;
  }
  I += Ki * error;
  D =  previous_error - error;
  previous_error = error;
  PID_value = int(P*Kp + I - D*Kd);
  PID_value = constrain(PID_value,-255 + s,255 - s);
}


void forward(){
  analogWrite(i1,150);
  analogWrite(i2,0);
  analogWrite(i3,150);
  analogWrite(i4,0);
}

void right_turn(int delay_) {
  /*forward();
  //  while((!digitalRead(sensor1))||(!digitalRead(sensor2))||(!digitalRead(sensor3))||(!digitalRead(sensor4))||(!digitalRead(sensor5)));
  delay(delay_/2);*/
  stop_();
  delay(200);
  analogWrite(i1, 150);
  analogWrite(i2, 0);
  analogWrite(i3, 0);
  analogWrite(i4, 150);
  delay(delay_);
  /*while((!digitalRead(sensor4))){
    delay(10);
    Serial.print("Right");
  }*/
}
void left_turn(int delay_) {
  /*forward();
  //while((!digitalRead(sensor1))||(!digitalRead(sensor2))||(!digitalRead(sensor3))||(!digitalRead(sensor4))||(!digitalRead(sensor5)));
  delay(delay_/2);*/
  stop_();
  delay(100);
  analogWrite(i1, 0);
  analogWrite(i2, 150);
  analogWrite(i3, 150);
  analogWrite(i4, 0);
  delay(delay_);
  /*while((!digitalRead(sensor2))){
    delay(10);
    Serial.print("Left");
  }*/
}
  
void u_turn() {
  /*The pin numbers and high, low values might be different depending on your connections */
  forward();
  //while((!digitalRead(sensor1))||(!digitalRead(sensor2))||(!digitalRead(sensor3))||(!digitalRead(sensor4))||(!digitalRead(sensor5)));
  delay(400);
  stop_();
  delay(100);
  analogWrite(i1, 0);
  analogWrite(i2, 150);
  analogWrite(i3, 150);
  analogWrite(i4, 0);
  delay(1000);
  /*while((!digitalRead(sensor2))){
    delay(10);
    Serial.print("Left");
  }*/
}
  
void stop_()
{
  analogWrite(i1, 0);
  analogWrite(i2, 0);
  analogWrite(i3, 0);
  analogWrite(i4, 0); 
}

void path_error(){
  stop_();
  while(pathErrorFlag){// Wait for Manual Reset or ISR
    digitalWrite(LED, HIGH);
    delay(300);
    digitalWrite(LED, LOW);
    delay(300);
  }
}

void run_mode(){
  runMode = true;
  runBot = true;
  inter_num = 0;
  pathErrorFlag = false;
}


void changeAutoTune()
{
 if(!tuning)
  {
    //Set the output to the desired starting frequency.
    output=aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  { //cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(boolean start)
{
  if(start)
    ATuneModeRemember = myPID.GetMode();
  else
    myPID.SetMode(ATuneModeRemember);
}


void SerialSend()
{
  Serial.print("setpoint: ");Serial.print(setpoint); Serial.print(" ");
  Serial.print("input: ");Serial.print(input); Serial.print(" ");
  Serial.print("output: ");Serial.print(output); Serial.print(" ");
  if(tuning){
    Serial.println("tuning mode");
  } else {
    Serial.print("kp: ");Serial.print(myPID.GetKp());Serial.print(" ");
    Serial.print("ki: ");Serial.print(myPID.GetKi());Serial.print(" ");
    Serial.print("kd: ");Serial.print(myPID.GetKd());Serial.println();
  }
}

void SerialReceive()
{
  if(Serial.available())
  {
   char b = Serial.read(); 
   Serial.flush(); 
   if((b=='1' && !tuning) || (b!='1' && tuning))changeAutoTune();
  }
}
