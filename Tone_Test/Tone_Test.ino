#include "pitches.h"

// notes in the melody:
int melody_1[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
};
uint16_t melody_2[] = {
  
};
// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations_1[] = {
  4, 8, 8, 4, 4, 4, 4, 4
};

void setup() {
  // iterate over the notes of the melody:
  float modifier = 1.0;
  do{
    for (int thisNote = 0; thisNote < 8; thisNote++) {
      uint16_t noteDuration = 1000 / noteDurations_1[thisNote];
      tone(8, melody_1[thisNote], noteDuration);
      
      delay(noteDuration * modifier );
      // stop the tone playing:
      noTone(8);
    }
    delay(100);
  }while(modifier+=0.1 && modifier <= 2.0);
}

void loop() {
  // no need to repeat the melody.
}
