const int trigPin=12;
const int echoPin=13;
int distance; 
int buzz = 6;
long duration=0;
void setup() {
  pinMode(8,INPUT);
  pinMode(4,OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
}
int ultra()
{
  digitalWrite(trigPin,LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin,LOW);

  duration = pulseIn(echoPin,HIGH);
  
  //Serial.print("Distance: ");
  
  distance = duration*0.034/2;
  
  //Serial.println(distance);
  
  delay(100);
  
  return(distance);
} 
void loop() {
  int ultracheck = ultra();
  ultracheck = constrain(ultracheck,5,200);
  ultracheck = map(ultracheck,5,200,10,900);
  tone(buzz,50,ultracheck);
  Serial.println(ultracheck);
}
