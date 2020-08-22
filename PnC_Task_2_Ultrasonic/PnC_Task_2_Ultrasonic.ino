const int trigPin=D1;
const int echoPin=D0;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin (9600);
}

int ultra()
{
  int distance; 
  long duration;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration=pulseIn(echoPin, HIGH);
   
  distance = duration*0.034/2;

  return(distance);
} 
uint32_t last = millis();
void loop()
{
  if(millis() - last >100){
    int dis = ultra();
    int val = constrain(dis,0,50) ;
    int out = map(val, 0, 50, 256 , -1);
    constrain(out, 0, 255);
    analogWrite(D3, out);
    Serial.println(dis);
    last = millis();
  }
}
 
