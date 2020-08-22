//YWROBOT
//Compatible with the Arduino IDE 1.0
//Library version:1.1
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display
String str = "Neil is here";
void setup()
{
  lcd.init();                      // initialize the lcd 
  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(2,0);
  lcd.print("Hello, world!");
  lcd.setCursor(0,1);
  lcd.print("Shine Baka(@_@)");
  delay(2000);
 lcd.setCursor(0,0);
  for(int i = 0; i<16; i++){
    lcd.setCursor(i,0);
    if(i<str.length()) lcd.print(str[i]);
    else lcd.print(" ");
    delay(150);
  }
}


void loop()
{
  delay(5000);
}
