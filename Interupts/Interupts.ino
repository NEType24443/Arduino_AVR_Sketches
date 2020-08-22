const uint8_t btnPin = 3;
const uint8_t ledPin = 5;
const uint8_t sensPin = 2;

uint8_t ledState = LOW;
uint8_t btnPrev = HIGH;

void setup(){
  pinMode(btnPin,INPUT_PULLUP);
  pinMode(ledPin,OUTPUT);
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(sensPin),toggle,FALLING);
}
void loop() {
    Serial.println(digitalRead(sensPin));
}

void toggle(){
  ledState = !ledState;  
  digitalWrite(ledPin,ledState);
}
