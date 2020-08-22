#define IN1_L 2
#define IN2_L 3 //LEFT SIDE MOTOR
#define IN3_R 4
#define IN4_R 5 //RIGHT SIDE MOTOR

#define ENA_L 6
#define ENB_R 7

#define LEFT_IR 8 
#define RIGHT_IR 9  //IR SENSOR

uint8_t en = 1;

void setup() {
pinMode(RIGHT_IR, INPUT);
pinMode(LEFT_IR, INPUT); 
pinMode(IN1_L, OUTPUT);
pinMode(IN2_L, OUTPUT);
pinMode(IN3_R, OUTPUT);
pinMode(IN4_R, OUTPUT);
pinMode(ENA_L, OUTPUT);
pinMode(ENB_R, OUTPUT);
digitalWrite(ENA_L, HIGH);
digitalWrite(ENB_R, HIGH);
}

void loop() {
if(en && digitalRead(RIGHT_IR) && digitalRead(LEFT_IR)){
  //Move Foreward
  digitalWrite(IN1_L, 1);
  digitalWrite(IN2_L, 0);
  digitalWrite(IN3_R, 1);
  digitalWrite(IN4_R, 0);
}
else if(en && !digitalRead(RIGHT_IR) && digitalRead(LEFT_IR)){
  //Turn Right
  digitalWrite(IN1_L, 1);
  digitalWrite(IN2_L, 0);
  digitalWrite(IN3_R, 0);
  digitalWrite(IN4_R, 1);
}
else if(en && digitalRead(RIGHT_IR) && !digitalRead(LEFT_IR)){
  //Turn Left
  digitalWrite(IN1_L, 0);
  digitalWrite(IN2_L, 1);
  digitalWrite(IN3_R, 1);
  digitalWrite(IN4_R, 0);
}
else if(en && !digitalRead(RIGHT_IR) && !digitalRead(LEFT_IR )){  
  //Stop Motors
  digitalWrite(IN1_L, 0);
  digitalWrite(IN2_L, 0);
  digitalWrite(IN3_R, 0);
  digitalWrite(IN4_R, 0);
  digitalWrite(ENA_L, 0);
  digitalWrite(ENB_R, 0);
  en = 0;
 }
}
