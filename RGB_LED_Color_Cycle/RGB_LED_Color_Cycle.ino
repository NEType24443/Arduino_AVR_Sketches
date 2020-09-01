#include <Arduino.h>
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

int val_1 = 255, val_2 = 0, val_3 = 0, cycle[] = {1, 1, 0};
float intensity_1 = 0.0, intensity_2 = 0.0, intensity_3 = 0.0;
bool f1 = 0, f2 = 2, f3 = 0;
void loop()
{ 
  if(val_1 >= 254)   f1 = 1;
  else if(val_1 <=1) f1 = 0;
  if(val_2 >= 254)   f2 = 1;
  else if(val_2 <=1) f2 = 0;
  if(val_3 >= 254)   f3 = 1;
  else if(val_3 <=1) f3 = 0;

  
  //int val_1 = 0, val_2 = 0, val_3 = 0;
  //    val_1 = int(sin(intensity-(PI/2.0)+(2.0*PI/3.0)-0.01)*(255.0/2)+(255.0/2)),
  //    val_2 = int(sin(intensity-(PI/2.0)+(PI/3.0)-0.01)*(255.0/2)+(255.0/2)), 
  //    val_3 = int(sin(intensity-(PI/2.0)-0.01)*(255.0/2)+(255.0/2));
  val_1 = ((cycle[0]%2 == 1)) ? int(sin(intensity_1)*(255.0/2)+(255.0/2)) : 0; //+   (PI)
  //analogWrite(pins[0], val_1);
  val_2 = ((cycle[1]%2 == 1)) ? int(sin(intensity_2)*(255.0/2)+(255.0/2)) : 0; //+ (PI/2.0)
  //analogWrite(pins[1], val_2);
  val_3 = ((cycle[2]%2 == 1)) ? int(sin(intensity_3)*(255.0/2)+(255.0/2)) : 0;
  //analogWrite(pins[2], val_3);
  if( !(val_1 + val_2) || !(val_2 + val_3) || !(val_3 + val_1) ) {
    cycle[0]++;
    cycle[1]++;
    cycle[2]++;
    if(cycle[0]>2) cycle[0] = 0;
    if(cycle[1]>2) cycle[1] = 0;
    if(cycle[2]>2) cycle[2] = 0;
  }
  if(f1 || !f3){
    intensity_1 += 0.01;
  }
  if (f2 || !f1){
    intensity_2 += 0.01;
  }
  if (f3 || !f2){
    intensity_3 += 0.01;
  }
  
  
  //last_val_2 = val_2;
  Serial.print(val_1);
  Serial.print("\t");
  Serial.print(val_2);
  Serial.print("\t");
  Serial.println(val_3);
  // intensity += 0.01;
  delay(1);
}
