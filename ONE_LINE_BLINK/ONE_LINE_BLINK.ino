void setup() {
  // put your setup code here, to run once:
  pinMode(13, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(13, millis()%1000 < 500); 
                    // millis()%Time_period < T_ON 
}
