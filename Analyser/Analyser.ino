const int scl = 6;
const int sda = 5;
void setup() {
  pinMode(scl,INPUT);
  pinMode(sda,INPUT);
  Serial.begin(9600);
}

void loop() {
  Serial.println(digitalRead(sda));

}
