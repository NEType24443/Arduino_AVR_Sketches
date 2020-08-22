void setup() 
{
  pinMode(A0,INPUT);
  Serial.begin(9600);
}

void loop() 
{
 Serial.print(analogRead(A0));
 Serial.println("\n");
 delay(500);
 }
 /*wont work for built in led bcuz of analog to dig conversion*/
