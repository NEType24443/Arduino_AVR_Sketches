int pot1 = A0;
int pot2 = A1;
int led = 3;
void setup() {
  pinMode(led,OUTPUT);
}

void loop() {
  int bright = map(analogRead(pot2),0,1023,0,255);
  int del = analogRead(pot1);
  analogWrite(led,bright);
  delay(del);
  analogWrite(led,0);
  delay(del);
}
