// Maps Potentiometer readings to a PWM signal of 1ms to 2ms duration
#include <Streaming.h>

void setup() {
  Serial.begin(115200);
  ADCSRA |= (0 << ADPS2) | (0<< ADPS1) | (0<< ADPS0);
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 5000;
  TCCR1B |= (1 << WGM12) | (1 << CS11) | (1 << CS10);
  TIMSK1 |= (1 << OCIE1A) | (1 << OCIE1B);
  DDRB = B00000001;
}

ISR(TIMER1_COMPA_vect){
  PORTB = B00000001;
  Serial<<"Timer 1 ISR set :"<<_WIDTH(micros(), 14)<<endl;
}

ISR(TIMER1_COMPB_vect){
  PORTB = B00000000;
  Serial<<"Timer 1 ISR reset :"<<_WIDTH(micros(), 12)<<endl;
}
uint16_t counterValue = 250;
void loop() {
  if(Serial.available()){
    String c = "";
    while(Serial.available() )c += Serial.read();
    counterValue = c.toInt();
    counterValue = counterValue>=250 ? (counterValue<=500 ? counterValue : 500 ) : 250 ;
    OCR1B = counterValue;
  }
}
