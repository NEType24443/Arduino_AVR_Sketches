// Which pins are connected to which LED
const byte oneLED =2;
const byte twoLED =4;

int oneLEDdim ;
int twoLEDdim ;

const unsigned int oneLEDdelay = 10;
const unsigned int twoLEDdelay = 40;

unsigned long oneLEDprev;
unsigned long twoLEDprev;

int oneLEDbrightness = 0;
int twoLEDbrightness = 0;

void setup () 
  {
    pinMode (oneLED, OUTPUT);
    pinMode (twoLED, OUTPUT);
    oneLEDprev = 0;
    twoLEDprev = 0;
    oneLEDdim = 5;
    twoLEDdim = 5;
  }

void fadeoneLED ()
  {
    oneLEDbrightness += oneLEDdim;
    if(oneLEDbrightness > 250 || oneLEDbrightness <= 0)
      oneLEDdim = -oneLEDdim;
    
    analogWrite(oneLED,oneLEDbrightness);   
    oneLEDprev = millis();   
  }

void fadetwoLED ()
  {
    twoLEDbrightness += twoLEDdim;
    if(twoLEDbrightness > 250 || twoLEDbrightness <= 0)
      twoLEDdim = -twoLEDdim;
    
    analogWrite(twoLED,twoLEDbrightness);   
    twoLEDprev = millis();
  }

void loop ()
  {
   if(millis() - oneLEDprev == oneLEDdelay)
    fadeoneLED ();
   if(millis() - twoLEDprev == twoLEDdelay)
    fadetwoLED ();

/* Other code that needs to execute goes here.
   It will be called many thousand times per second because the above code
   does not wait for the LED fade interval to finish. */

}
