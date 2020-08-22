#include "DHT.h"
DHT dht();
#define DHTPIN 13

void setup(){
  Serial.begin(9600);
}

void loop()
{
  digitalWrite(10,LOW);
  int chk = DHT.read11(DHT11_PIN);
  Serial.print("Temperature = ");
  Serial.println(DHT.temperature);
  Serial.print("Humidity = ");
  Serial.println(DHT.humidity);
  if (DHT.humidity>75)
    digitalWrite(10,HIGH);
  delay(2000);
}
