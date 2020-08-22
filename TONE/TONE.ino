void setup(){
  pinMode(4,OUTPUT);
}
void loop(){
  int i;
  for(i=0;i<4;i++){
    tone(4,50);
    delay(300);
    noTone(4);
    delay(300);
  }
  for(i=0;i<4;i++){
    tone(4,400);
    delay(300);
    noTone(4);
    delay(300);
  }
}
