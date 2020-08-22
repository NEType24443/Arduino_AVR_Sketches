//PID LINE FOLLOWER 
//(imagine th at you are driving the bot for right-left convention)

// use MEGA

//motor driver-pins must be pwm to control speed!!!

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

//#define StR 0
#define ST 1
#define RT 2
#define LT 3
#define SR 4
#define SL 5
#define UT 6
#define EN 7

bool run_bot = true;
bool reachedIntersection = false;

int dir[50]= {0},intersection[10]={0}, current_pos = 0; // keeps track of directions

int ir[6];  //ir array values

int s=150;//fixed speed

int rs,ls;    //right speed , left speed

// PID Constants
float Kp=32;
float Ki=0;
float Kd=32;
 
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
  if (run_bot){
    find_position();
    calculate_PID();
    if(!digitalRead(sensor3)) reachedIntersection = false;
  }
}

void process_data(){
  for (int i=0; dir[i]!=0; i++){
    if((dir[i] == UT) && (i>=1)){
      switch(dir[i-1]){
        case ST:
          
          break;
        case SR:
          
          break;
        case SL:
          
          break;
        case LT:
          
          break;
        case RT:
          
          break;
        case UT:
          
          break;
          
      }
      switch(dir[i+1]){
        
      }
    }
  }
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
   /*else if ((ir[1] == 1) && (ir[2] == 1) && (ir[3] == 1) && (ir[4] == 0) && (ir[5]==0)) // Turn robot right side
    {error = 100; dir[current_pos] =}
   else if ((ir[1] == 0) && (ir[2] == 0) && (ir[3] == 1) && (ir[4] == 1) && (ir[5] == 1)) // Turn left side
    {error = 101; dir[current_pos] = EN}*/
  if(!reachedIntersection){
    /*if ((ir[1] == 0) && (ir[2] == 0) && (ir[3] == 0) && (ir[4] == 1) && (ir[5] == 1)) // SL+LT
      {error = -3; dir[current_pos] = SL;} // DEBATEABLE!!*/
    if ((ir[1] == 0) && (ir[2] == 0) && (ir[3] == 1) && (ir[4] == 1) && (ir[5] == 0)) // ST+LT
      {error = -1; dir[current_pos] = LT;}
    else if ((ir[1] == 0) && (ir[2] == 1) && (ir[3] == 1) && (ir[4] == 0) && (ir[5] == 0)) // ST+RT
      {error = 1; dir[current_pos] = ST;}
    /*else if ((ir[1] == 1) && (ir[2] == 1) && (ir[3] == 0) && (ir[4] == 0) && (ir[5] == 0)) // RT+SR
      {error = 3; dir[current_pos] = SR;}*/
    else if ((ir[1] == 1) && (ir[2] == 0) && (ir[3] == 0) && (ir[4] == 0) && (ir[5] == 1)) // SR and SL
      {error = -4; dir[current_pos] = SL;}
    else if ((ir[1] == 1) && (ir[2] == 0) && (ir[3] == 0) && (ir[4] == 1) && (ir[5] == 0))// SR and LT
      {error = -2; dir[current_pos] = LT;}
    else if ((ir[1] == 0) && (ir[2] == 1) && (ir[3] == 0) && (ir[4] == 1) && (ir[5] == 0))//LT and RT
      {error = -2; dir[current_pos] = LT;}
    else if ((ir[1] == 0) && (ir[2] == 1) && (ir[3] == 0) && (ir[4] == 0) && (ir[5] == 1))//SL and RT
      {error = -4; dir[current_pos] = SL;}
    else if ((ir[1] == 0) && (ir[2] == 0) && (ir[3] == 0) && (ir[4] == 0) && (ir[5] == 0)) // Make U turn
      {error = 102; dir[current_pos] = UT;}
    else if ((ir[1] == 1) && (ir[2] == 1) && (ir[3] == 1) && (ir[4] == 1) && (ir[5] == 1)) //end point
      {error = 0; stop_bot(); dir[current_pos] = EN;}
    reachedIntersection = true; 
    current_pos++;
  }
  else{
    if ((ir[1] == 0) && (ir[2] == 0) && (ir[3] == 0) && (ir[4] == 0) && (ir[5] == 1)) // SL
      {error = -4; }
    else if ((ir[1] == 0) && (ir[2] == 0) && (ir[3] == 0) && (ir[4] == 1) && (ir[5] == 0)) //LT
      {error = -2; }
    else if ((ir[1] == 0) && (ir[2] == 0) && (ir[3] == 1) && (ir[4] == 0) && (ir[5] == 0)) // ST
      {error = 0; }
    else if ((ir[1] == 0) && (ir[2] == 1) && (ir[3] == 0) && (ir[4] == 0) && (ir[5] == 0)) // RT
      {error = 2; }
    else if ((ir[1] == 1) && (ir[2] == 0) && (ir[3] == 0) && (ir[4] == 0) && (ir[5] == 0)) // SR
      {error = 4; }
  }
}

void stop_bot(){
  run_bot = false;
  digitalWrite(i1, LOW);
  digitalWrite(i2, LOW);
  digitalWrite(i3, LOW);
  digitalWrite(i4, LOW);
  //process_data(); // processes data from initial Exploration
  //explore_paths()
}

void calculate_PID(){ 
  P= error;
  constrain(I, -240, 240);
  I = I + error;
  D =  previous_error - error;
  previous_error = error;
  PID_value = int(P*Kp + I*Ki - D*Kd);
  Serial.print("error value is   ");
  Serial.println(error);
  Serial.print("pid value is   ");
  Serial.println(PID_value);
  Serial.print("I value is   ");
  Serial.println(I);
  /*
  ///                               RIGHT TURN
  if(dir[current_pos] == (SR||RT){
    analogWrite(i1,0);
    analogWrite(i2,rs);
    analogWrite(i3,ls);
    analogWrite(i4,0);
    while(digitalRead(sensor3) == 0);
    reachedIntersection = false;
  }
  ////                              LEFT TURN
  if (dir[current_pos] == (SL||RT)){ 
    analogWrite(i1,rs);
    analogWrite(i2,0);
    analogWrite(i3,0);
    analogWrite(i4,ls);
    while(digitalRead(sensor3) == 1);
    reachedIntersection = false;
  }
  //                                    STRAIGHT
  if (dir[current_pos] == (ST)){ 
    analogWrite(i1,rs);
    analogWrite(i2,0);
    analogWrite(i3,ls);
    analogWrite(i4,0);
    while(digitalRead(sensor3) == 1);
    reachedIntersection = false;
  }*/
  //alternative 
  analogWrite(i1,rs);
  analogWrite(i2,0);
  analogWrite(i3,ls);
  analogWrite(i4,0);
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
  constrain(ls,0,255); 
}
