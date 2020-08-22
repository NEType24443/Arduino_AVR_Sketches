#include <Streaming.h>

#define BIT 0xFFFC0000

long lat = 74567839;
long lon = 135422318;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial<< "Lat: " << (lat/10000000) << "." << (lat%10000000)
        << endl
        << "Lon: " << (lon/10000000) << "." << (lon%10000000)
        << endl;

}

void loop() {
  // put your main code here, to run repeatedly:
  
}
