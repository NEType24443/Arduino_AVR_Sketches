#define USE_ARDUINO_INTERRUPTS true
#include<dht.h>
dht DHT; 
#include<PulseSensorPlayground.h>
#define ldr A1
#define pulse A0
#define temp 13
             
int Threshold = 550; 
PulseSensorPlayground pulseSensor;
void setup() 
{
  pinMode(pulse,INPUT);
  Serial.begin(9600);
  pulseSensor.analogInput(pulse);   
  pulseSensor.setThreshold(Threshold);
  if (pulseSensor.begin()) 
  {
    Serial.println("pulseSensor Object created"); 
  }  
}

void loop() {
  int BPM = pulseSensor.getBeatsPerMinute();
  int chk = DHT.read11(temp);
  int temperature=DHT.temperature;
  int t=0;
  int l =analogRead(ldr);
  if(l>600)
  {
    t=0;
  }
  else if((l<570)&&(l>400))
   {
    t=1;
   }
  else if (l<300)
  {
    t=2;
  }
  if (pulseSensor.sawStartOfBeat()||temp>26&&temp<26) 
  {
   if((BPM>100||BPM<50)||(temp))
   {
    if(BPM>100)
    {
      Serial.println("Your Heart Rate is High");
    }
    else if (BPM<50)
    {
      Serial.println("Your Heart Rate is LOW");
    }
    if (temperature>26)
    {
      Serial.println("Your Body Temperature is High");  
    }
    if (temperature<26)
    {
      Serial.println("Your Body Temperature is LOW");  
    }
      
      Serial.println("Current BP:");
      Serial.print(BPM); 
      Serial.println("\nCurrent Body Temperature:");
      Serial.print(temperature);
      Serial.println("\nIssue reported in the ");
      if(t=0)
      {
        Serial.println("Night hours");
      }
      else if(t=1)
      {
        Serial.println("Evening hours");
      }
      else if(t=2)
      {
        Serial.println("Morning hours");
      }
    }
  }
  delay(2000);
}
