const int led = 13;
int state = 0;
int prestate = 0;
int count = 0;
void setup() {
  pinMode(led,OUTPUT);
  Serial.begin(9600);
  digitalWrite(led,LOW);
}

void loop() {
  if(Serial.available()>0)
  {
    char input = Serial.read();
    
    if( input == 'n')
      state = 1;
    else
      state = 0;
      
    if(state != prestate){
      if(state == 1){
        count++;
        }    
      if(count%2 == 0)
        digitalWrite(led,1);
      else      
        digitalWrite(led,0);
    }
    prestate = state;
  }
}
