
#define dW digitalWrite

void setup() {
  for(int x = 2; x <= 8; x++){
    pinMode(x, OUTPUT);
  }
    

}

int x = random(200,800);
uint32_t now ;
void loop() {
  now = millis();
  dW(2, now%1000 < x);
  if (now%1000 < x) x = random(50, 950);
  dW(3, now%1000 < x);
  if (now%1000 < x) x = random(50, 950);
  dW(4, now%1000 < x);
  if (now%1000 < x) x = random(50, 950);
  dW(5, now%1000 < x);
  if (now%1000 < x) x = random(50, 950);
  dW(6, now%1000 < x);
  if (now%1000 < x) x = random(50, 950);
  dW(7, now%1000 < x);
  if (now%1000 < x) x = random(50, 950);
  dW(8, now%1000 < x);
  if (now%1000 < x) x = random(50, 950);
  /*if (millis()- last_ > 2000){
    x = random(50, 950);
    last_ = millis();
  }*/
}
