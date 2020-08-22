const int trigPin=9;
const int echoPin=10;
int distance; 
long duration;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin (9600);
}

int ultra()
{
  digitalWrite(trigPin,LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin,LOW);

  duration=pulseIn(echoPin,HIGH);
   
  distance = duration*0.034/2;

  return(distance);
} 
void loop()
{
  delayMicroseconds(5);
  int uc = ultra();
  int val = constrain(uc,0,200) ;
  analogWrite(6,val);
}
 
