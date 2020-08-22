const int green1Led = 7;
const int green2Led = 6;
const int green3Led = 5;
const int green4Led = 4;
const unsigned long Threshold1 =  70;
const unsigned long Threshold2 = 80;
const unsigned long Threshold3 = 90;
const unsigned long Threshold4 = 100;
unsigned long last1Time;
unsigned long last2Time;
unsigned long last3Time;
unsigned long last4Time;
void setup() {
  pinMode(green1Led,OUTPUT);
  pinMode(green2Led,OUTPUT);
  pinMode(green3Led,OUTPUT);
  pinMode(green4Led,OUTPUT);
  last1Time = millis();
  last2Time = millis();
  last3Time = millis();
  last4Time = millis();
  Serial.begin(9600);
}
void g1(){
  if(digitalRead(green1Led) == HIGH)
    digitalWrite(green1Led,LOW);
  else
    digitalWrite(green1Led,HIGH);
  last1Time = millis(); 
}
void g2(){
  if(digitalRead(green2Led) == HIGH)
    digitalWrite(green2Led,LOW);
  else
    digitalWrite(green2Led,HIGH);
  last2Time = millis(); 
}
void g3(){
  if(digitalRead(green3Led) == HIGH)
    digitalWrite(green3Led,LOW);
  else
    digitalWrite(green3Led,HIGH);
  last3Time = millis(); 
}
void g4(){
  if(digitalRead(green4Led) == HIGH)
    digitalWrite(green4Led,LOW);
  else
    digitalWrite(green4Led,HIGH);
  last4Time = millis(); 
}
void loop() {
  unsigned long currentTime = millis(); 
  if((currentTime - last1Time) > Threshold1)
    g1();
  if((currentTime - last2Time) > Threshold2)
    g2();
  if((currentTime - last3Time) > Threshold3)
    g3();
  if((currentTime - last4Time) > Threshold4)
    g4();  
  Serial.println(millis());
}
