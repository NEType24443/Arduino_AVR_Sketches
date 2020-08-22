#define CAN_H_Rx A4
#define CAN_L_Rx A5
#define CAN_H_Tx 4
#define CAN_L_Tx 5
#define SAMPLE_RATE_1KHz 1
#define SAMPLE_RATE_100Hz 10

void setup() {
  pinMode(CAN_H_Rx, INPUT);
  pinMode(CAN_L_Rx, INPUT);
  pinMode(CAN_H_Tx, OUTPUT);
  pinMode(CAN_L_Tx, OUTPUT);
  Serial.begin(115200);
}
unsigned long time_now , time_last = 0; 
int CAN_H, CAN_L, i=0, zero_count=0, one_count=0;
bool buff[40];

void loop() {
  time_now = millis();
  if(time_now - time_last >= SAMPLE_RATE_100Hz) {
    CAN_H = analogRead(CAN_H_Rx);
    CAN_L = analogRead(CAN_L_Rx);
    /*Serial.print("\nCAN_H: ");
    Serial.print(analogRead(CAN_H_Rx));
    Serial.print("\nCAN_L: ");
    Serial.print(analogRead(CAN_L_Rx));*/
    if(CAN_L>900 && CAN_H<10){
      buff[i] = true;
      one_count++;
    }
    else if(CAN_H>900 && CAN_L<10) {
      buff[i] = false;
      zero_count++;
    }
    if(zero_count>5 && buff[i])i--;
    else errorFrame();
    if(one_count>5 && buff[i])i--;
    else errorFrame();
    i++;
    if(i==40){
      i=0;
      for(int a=0; a<40; a++){
        Serial.print(buff[a]);
        buff[a]=0;
      }
      Serial.println("");
      one_count = 0;
      zero_count = 0;
    }
    time_last = millis();
  }
}
void errorFrame(void){
  time_now = millis();
  if(time_now - time_last >= SAMPLE_RATE_100Hz) {
    digitalWrite(CAN_H_Tx, HIGH);
    digitalWrite(CAN_L_Tx, LOW);
    if(CAN_L>900 && CAN_H<10){
    
  }
  time_last = millis();
}
