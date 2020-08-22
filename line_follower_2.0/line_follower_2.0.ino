//PID LINE FOLLOWER 
//(imagine th at you are driving the bot for right-left convention)

// use MEGA

//motor driver-pins must be pwm to control speed!!!
#include <PID_v1.h>
#define i1 10 //right forward
#define i2 9  //right backward
#define i3 5  //left forward
#define i4 6  //left backward 
 
//IR ARRAY
#define  sensor1 A0     // RIGHT most sensor
#define  sensor2 A1     
#define  sensor3 A2 
#define  sensor4 A3    
#define  sensor5 A4     // LEFT most sensor

     
float ir[6];  //ir array values
int s=150;//fixed speed
int rs,ls;    //right speed , left speed

// PID Constants
float Kp=30;
float Ki=0;
float Kd=30;
 
float sum;

float pos;    //instantaneous position
float setpoint;
float PID_value = 0;


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
  
  Serial.begin(9600);

  setpoint=0 ;
}

float error=0, previous_error = 0, P = 0, I = 0, D = 0;

void loop() {
  find_position();
  calculate_PID();
  turnspeed() ;
  move_bot();
}

int find_position(){  
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
       if ((ir[1] == 0) && (ir[2] == 0) && (ir[3] == 0) && (ir[4] == 0) && (ir[5] == 1))
    error = -4;
  else if ((ir[1] == 0) && (ir[2] == 0) && (ir[3] == 0) && (ir[4] == 1) && (ir[5] == 0))
    error = -2;
  else if ((ir[1] == 0) && (ir[2] == 0) && (ir[3] == 1) && (ir[4] == 0) && (ir[5] == 0))
    error = 0;
  else if ((ir[1] == 0) && (ir[2] == 1) && (ir[3] == 0) && (ir[4] == 0) && (ir[5] == 0))
    error = 2;
  else if ((ir[1] == 1) && (ir[2] == 0) && (ir[3] == 0) && (ir[4] == 0) && (ir[5] == 0))
    error = 4;
  else if ((ir[1] == 0) && (ir[2] == 0) && (ir[3] == 0) && (ir[4] == 1) && (ir[5] == 1))
    error = -3;
  else if ((ir[1] == 0) && (ir[2] == 0) && (ir[3] == 1) && (ir[4] == 1) && (ir[5] == 0))
    error = -1;
  else if ((ir[1] == 0) && (ir[2] == 1) && (ir[3] == 1) && (ir[4] == 0) && (ir[5] == 0))
    error = 1;
  else if ((ir[1] == 1) && (ir[2] == 1) && (ir[3] == 0) && (ir[4] == 0) && (ir[5] == 0)) 
    error = 3;
// else if ((ir[1] == 0) && (ir[2] == 0) && (ir[3] == 0) && (ir[4] == 0) && (ir[5] == 0))
  //  error = 102;
   /*else if ((ir[1] == 1) && (ir[2] == 1) && (ir[3] == 1) && (ir[4] == 0) && (ir[5]==0)) // Turn robot sharp right side
    error = 10;
  else if ((ir[1] == 0) && (ir[2] == 0) && (ir[3] == 1) && (ir[4] == 1) && (ir[5] == 1)) // Turn sharp left side
   error = 15;
  // else if ((ir[1] == 1) && (ir[2] == 1) && (ir[3] == 1) && (ir[4] == 1) && (ir[5] == 1)) // Turn left side
   //error = 100;*/

}


void turnspeed(){  
  //Restricting the error value between +255.
  rs = s;
  ls = s;
  if(PID_value<0){
    ls = s+PID_value;
  }
  if(PID_value>0){
    rs = s-PID_value;
  }
  constrain(rs,0,255);
  constrain(rs,0,255);
  /*if (rs < 0){   
    rs = 0;
  }
  if (rs> 255){
    rs = 255;
    }

    if (ls < 0){
    ls = 0;
    }
  if (ls> 255){
    ls = 255;
    }
  */
}
void move_bot(){
  
  Serial.print("rs value is  ");
  Serial.println(rs);
  Serial.print("ls value is  ");
  Serial.println(ls);
  if(rs>ls){
    Serial.println("Left Turn");
  }
  else if(rs<ls){
    Serial.println("Right Turn");
  }
  else{
    Serial.println("Forward");
  }
  
  Serial.println("****");
  
  analogWrite(i1,rs);
  analogWrite(i2,0);
  analogWrite(i3,ls);
  analogWrite(i4,0);
}

/*void reverse(){
 void sharpRightTurn() {
  analogWrite(i1, rs);
  analogWrite(i2, ls);
  analogWrite(i3, rs);
  analogWrite(i4, rs);
}
void sharpLeftTurn() {
  analogWrite(i1, rs);
  analogWrite(i2, rs);
  analogWrite(i3, rs);
  analogWrite(i4,ls );
  }
}*/
