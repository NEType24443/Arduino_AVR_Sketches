#define DATA A0

void setup(){
  pinMode( DATA, INPUT);
  Serial.begin(115200);
}
int value;
void loop() {
  value = analogRead(DATA);
  Serial.println(value);
}
