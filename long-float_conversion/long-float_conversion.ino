#include <Streaming.h>

const long l = 172573947;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial<<_FLOAT(l/10000000.F, 7)<<endl;
}

void loop() {
  // put your main code here, to run repeatedly:

}
