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
#define  sensorR 10     // A6 is damaged

//BUTTON,LED
#define BUTTON 3  //Interrupt 
#define LED 12

#define L 1
#define S 2
#define R 3
//#define U 4

//#define Sample_Time = 1;
     
int  inter[50]={0};
int inter_num = 0 ; // keeps track of directions
 
float ir[6];  //ir array values

int s=90;//fixed speed

// PID Constants
float Kp=19;//30
float Ki=0.00;
float Kd=37;//50.0;


//unsigned long last_time = millis();
 
float sum;

float pos;    //instantaneous position
float setpoint;
float PID_value = 0;


void calculate_PID();
void forward();
void stop_();
void left_turn(int);
void right_turn(int);
void path_error();
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
  //attachInterrupt(digitalPinToInterrupt(BUTTON), run_mode, FALLING);
  Serial.begin(9600);

  setpoint=0 ;
  delay(1000);
}


float error=0, previous_error = 0, P = 0, I = 0, D = 0;

void loop() {
  find_position();
  //if(!runMode){
    direction_decoder();
  //}
  //else{
    //path_follow();
  //}
  calculate_PID();
  if(error>0){
    if (PID_value < 0) PID_value = PID_value * -1;
    Serial.println(s+PID_value);
    analogWrite(i1,s+PID_value-5);
    analogWrite(i2,0);
    analogWrite(i3,0);//sminor
    analogWrite(i4,0);
  }
  if(error<0){
    if (PID_value < 0) PID_value = PID_value * -1;
    Serial.println(s+PID_value);
    analogWrite(i1,0);  //sminor
    analogWrite(i2,0);  
    analogWrite(i3,s+PID_value);
    analogWrite(i4,0);
  }
  if(error==0){
    if (PID_value < 0) PID_value = PID_value * -1;
    Serial.println(s+PID_value);
    analogWrite(i1,s-5);
    analogWrite(i2,0);
    analogWrite(i3,s);
    analogWrite(i4,0);
  }
  delay(100);
  forward();
}


void find_position(){ 
 
  ir[1] = digitalRead(sensor1);
  ir[1]=(!ir[1]);
  Serial.println(ir[1]);
  ir[2] = digitalRead(sensor2);
  ir[2]=(!ir[2]);
  Serial.println(ir[2]);
  ir[3] = digitalRead(sensor3);
  ir[3]=(!ir[3]);
  Serial.println(ir[3]);
  ir[4] = digitalRead(sensor4);
  ir[4]=(!ir[4]);
  Serial.println(ir[4]);
  ir[5] = digitalRead(sensor5);
  ir[5]=(!ir[5]);
  Serial.println(ir[5]);
  /*ir[6] = digitalRead(sensorL);
  ir[6]=(!ir[6]);
  Serial.println(ir[6]);
  ir[7] = digitalRead(sensorR);
  ir[7]=(!ir[7]);
  Serial.println(ir[5]);*/
}


void direction_decoder(void){
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
      left_turn(400);
      inter[inter_num] += L;
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
        left_turn(200);
        inter[inter_num] += L;
        inter_num++;
      }
      /*else if ((ir[1] == 1) && (ir[4] == 0) && (ir[5] == 0)){//R-RA
        
      }*/
      else if ((ir[1] == 0) &&( (ir[2] == 1)||(ir[3] == 1)||(ir[4] == 1)) && (ir[5] == 0)){//R-ST
        forward();
        inter[inter_num] += S;
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
    if ((ir[2] == 1) && (ir[3] == 1) && (ir[4] == 1) && (ir[5] == 1) && (ir[1] == 1)){ // T
      stop_();
      delay(50);
      left_turn(400);
      inter[inter_num] += L;
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
        left_turn(400);
        inter[inter_num] += L;
        inter_num++;
      }
      else if ((ir[1] == 0) && ( (ir[2] == 1)||(ir[3] == 1)||(ir[4] == 1)) && (ir[5] == 0)){//L-ST
        delay(50);
        left_turn(400);
        inter[inter_num] += L;
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
  else if ((ir[1] == 0) && (ir[2] == 1) && (ir[3] == 1) && (ir[4] == 1) && (ir[5] == 0)){ // Y- Intersection
    //stop_();
    delay(100);
    //forward();
    //delay(50);
    stop_();
    delay(1000);
    find_position();
    if(((ir[7] == 1)||(ir[1] == 1)) && ((ir[5] == 1)||(ir[6] == 1))){
      left_turn(200);
      //dir[dir_num] = L;
      //dir++;
      inter[inter_num] += L;
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
    //dir[dir_num] = U;
    //inter[inter_num] = 0
    inter_num--;
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
  I = I + error;
  D =  previous_error - error;
  previous_error = error;
  PID_value = int(P*Kp + I*Ki - D*Kd);
 /* Serial.print("error value is   ");
  Serial.println(error);
  Serial.print("pid value is   ");
  Serial.println(PID_value);
    Serial.print("p value is   ");
  Serial.println(P);
    Serial.print("I value is   ");
  Serial.println(I);
  Serial.print("D value is   ");
  Serial.println(D);*/
}
//last_time = now;
  
//      //////////////////////////////RIGHT TURN
//        if(ir[1]==1)
//        {
//         //delay(500);
//         while(1){
//        analogWrite(i1,0);
//        analogWrite(i2,rs);
//        analogWrite(i3,ls);
//        analogWrite(i4,0);
//      
//      ir[3] = digitalRead(sensor3);
//      ir[3]=(!ir[3]);`
//      if(ir[3]==1){
//        break;
//        }
//          }
//          //forward();
//        }
//        /////////////////////////////////////LEFT TURN
//        if (ir[5]==1)
//        { 
//          //delay(5000);
//         while(1){
//        analogWrite(i1,rs);
//        analogWrite(i2,0);
//        analogWrite(i3,0);
//        analogWrite(i4,ls);
//      
//      ir[3] = digitalRead(sensor3);
//       ir[3]=(!ir[3]);
//      if(ir[3]==0){
//        break;
//        
//        }
//       }
//       //forward();
//      }
//}

//void turnspeed(){  
//  //Restricting the error value between +255.
//rs = s;
//ls = s;
// if(PID_value<0)
// {
//  ls = s+PID_value;
//  }
//  if(PID_value>0)
//  {
//    rs = s-PID_value;
//    }
//  
//  if (rs < 0){   
//    rs = 0;
//  }
//  if (rs> 255){
//    rs = 255;
//    }
//
//    if (ls < 0){
//    ls = 0;
//    }
//  if (ls> 255){
//    ls = 255;
//    }
//  
//}


/*void reverse()
{
  /*The pin numbers and high, low values might be different depending on your connections */
//  digitalWrite(i1, LOW);
//  digitalWrite(i2, HIGH);
//  digitalWrite(i3, LOW);
//  digitalWrite(i4, HIGH);
//}
//
// void right()
//{
//  /*The pin numbers and high, low values might be different depending on your connections */
//  analogWrite(i1, 0);
//  analogWrite(i2, 0);
//  analogWrite(i3, rs);
//  analogWrite(i4, 0);
//  //delay(500);
//}
//
//void left()
//{
//  /*The pin numbers and high, low values might be different depending on your connections */
//  analogWrite(i1, rs);
//  analogWrite(i2, 0);
//  analogWrite(i3, 0);
//  analogWrite(i4, 0);
//  delay(500);
//}
//void forwardmove()
//{
//  /*The pin numbers and high, low values might be different depending on your connections */
//  analogWrite(i1, rs);
//  analogWrite(i2, 0);
//  analogWrite(i3, ls);
//  analogWrite(i4, 0);
//}*/
//


void forward(){
  analogWrite(i1,150);
  analogWrite(i2,0);
  analogWrite(i3,155);
  analogWrite(i4,0);
}

 

void right_turn(int delay_) {
  /*The pin numbers and high, low values might be different depending on your connections */
  /*analogWrite(i1, 150);
  analogWrite(i2, 0);
  analogWrite(i3, 150);
  analogWrite(i4, 0);*/
  forward();
  //  while((!digitalRead(sensor1))||(!digitalRead(sensor2))||(!digitalRead(sensor3))||(!digitalRead(sensor4))||(!digitalRead(sensor5)));
  delay(delay_/2);
  stop_();
  delay(200);
  analogWrite(i1, 150);
  analogWrite(i2, 0);
  analogWrite(i3, 0);
  analogWrite(i4, 155);
  delay(delay_);
  /*while((!digitalRead(sensor4))){
    delay(10);
    Serial.print("Right");
  }*/
}
void left_turn(int delay_) {
  /*The pin numbers and high, low values might be different depending on your connections */
  forward();
  //while((!digitalRead(sensor1))||(!digitalRead(sensor2))||(!digitalRead(sensor3))||(!digitalRead(sensor4))||(!digitalRead(sensor5)));
  delay(delay_/2);
  stop_();
  delay(200);
  analogWrite(i1, 0);
  analogWrite(i2, 150);
  analogWrite(i3, 155);
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
  delay(300);
  stop_();
  delay(100);
  analogWrite(i1, 0);
  analogWrite(i2, 150);
  analogWrite(i3, 155);
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

void run_mode(){
//  runMode = true;
}
