void setup() 
{
pinMode(13,OUTPUT);
}
static int count=0;
void loop() 
{
  while(count<5)
  {digitalWrite(13,HIGH);
  delay(1000);
  digitalWrite(13,LOW);
  delay(800);
  count++;
  }
  while(count<15)
  {digitalWrite(13,HIGH);
  delay(100);
  digitalWrite(13,LOW);
  delay(400);
  count++;
  } 
}
