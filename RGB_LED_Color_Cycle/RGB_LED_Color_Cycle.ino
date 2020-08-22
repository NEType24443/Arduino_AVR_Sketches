#define R  11
#define G 9
#define B 10

#include<math.h>

int pins[] = {R, G, B};

void setup()
{
  Serial.begin(115200);
  for(int i = 0; i < sizeof(pins)/sizeof(int); i++) pinMode(i, OUTPUT);
}

int val_1 = 0, val_2 = 0, val_3 = 0;
float intensity = 0.0;

void loop()
{
  //int val_1 = 0, val_2 = 0, val_3 = 0;
  //    val_1 = int(sin(intensity-(PI/2.0)+(2.0*PI/3.0)-0.01)*(255.0/2)+(255.0/2)),
  //    val_2 = int(sin(intensity-(PI/2.0)+(PI/3.0)-0.01)*(255.0/2)+(255.0/2)), 
  //    val_3 = int(sin(intensity-(PI/2.0)-0.01)*(255.0/2)+(255.0/2));
  val_1 = int(sin(intensity-(PI/2.0)+(PI)-0.01)*(255.0/2)+(255.0/2));
  analogWrite(pins[0], val_1);
  val_2 = int(sin(intensity-(PI/2.0)+(PI/2.0)-0.01)*(255.0/2)+(255.0/2));
  analogWrite(pins[1], val_2);
  val_3 = int(sin(intensity-(PI/2.0)-0.01)*(255.0/2)+(255.0/2));
  analogWrite(pins[2], val_3);
  Serial.print(val_1);
  Serial.print("\t");
  Serial.print(val_2);
  Serial.print("\t");
  Serial.println(val_3);
  intensity += 0.01;
  delay(10);
}
