int irl=11;
int irr=12;
int lmc=9;
int lma=6;
int rmc=10;
int rma=5;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(irl,INPUT);
  pinMode(irr,INPUT);
  pinMode(lma,OUTPUT);
  pinMode(rma,OUTPUT);
   pinMode(lmc,OUTPUT);
  pinMode(rmc,OUTPUT);
  

}

void loop() {
  // put your main code here, to run repeatedly:
  int val1=digitalRead(irl);
  int val2=digitalRead(irr);
  if(irl==0&&irr==0)//stop
  {
   analogWrite(rmc,0);
    analogWrite(lmc,0);
      analogWrite(rma,0);
        analogWrite(lma,0);
  }
  if(irl==1&&irr==0)//right
  {
   analogWrite(rmc,0);
    analogWrite(lmc,225);
      analogWrite(rma,25);
        analogWrite(lma,0);
  }
  if(irl==0&&irr==1)//left
  {
   analogWrite(rmc,225);
    analogWrite(lmc,0);
      analogWrite(rma,0);
        analogWrite(lma,25);
  }
  if(irl==1&&irr==1)//straight
  {
   analogWrite(rmc,25);
    analogWrite(lmc,225);
      analogWrite(rma,0);
        analogWrite(lma,0);
  }
  

}
