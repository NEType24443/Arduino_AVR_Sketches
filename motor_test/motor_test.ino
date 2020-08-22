#define i1 11 //right forward
#define i2 9  //right backward
#define i3 5  //left forward
#define i4 6  //left backward 
 

void setup() {

  // put your setup code here, to run once:
  pinMode(i1,OUTPUT);
  pinMode(i2,OUTPUT);
  pinMode(i3,OUTPUT);
  pinMode(i4,OUTPUT);
  
}

void loop() {
  // put your main code here, to run repeatedly:
//  analogWrite(i1,150);
//  analogWrite(i2,0);
//  analogWrite(i3,0);
//  analogWrite(i4,0);
//  delay(5000);
//  analogWrite(i1,150);
//  analogWrite(i2,0);
//  analogWrite(i3,150);
//  analogWrite(i4,0);
  delay(5000);
  digitalWrite(i1,HIGH);
  digitalWrite(i2,LOW);
  digitalWrite(i3,HIGH);
  digitalWrite(i4,LOW);
//  analogWrite(i1,0);
//  analogWrite(i2,0);
//  analogWrite(i3,150);
//  analogWrite(i4,0);
//  delay(5000);
//  analogWrite(i1,0);
//  analogWrite(i2,150);
//  analogWrite(i3,0);
//  analogWrite(i4,150);
//  delay(5000);
}
