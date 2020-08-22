int led = 13;
void setup(){
  pinMode(led, OUTPUT);
  Serial.begin(9600);
}
void loop(){
  if( Serial.available()>0){
    int data = Serial.read();
    if (data == '1'){
      digitalWrite(led, HIGH);
      delay(100);
    }
    else if(data == '0'){
      digitalWrite(led, LOW);
      delay(100);
    }
    Serial.println(data);
  }
}
