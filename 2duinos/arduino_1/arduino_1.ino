int in = 2;
int state = 0;
void setup() {
  pinMode(2,INPUT_PULLUP);
  Serial.begin(9600);
  
}

void loop() {
  if(digitalRead(in)== 0)
    Serial.write('n');
    else
    Serial.write(0);
}
