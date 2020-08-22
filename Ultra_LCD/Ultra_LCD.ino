
/*
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

void setup(){
  lcd.init();                      // initialize the lcd 
  lcd.backlight();
  Serial.begin(9600);
}
void loop(){
  delay(100);
  lcd.clear();
  lcd.write(c[i]);
}
void lcd_print_line(char c){
  
}*/
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define CUSTOM_CHAR_COUNT 22

uint8_t bell[8]  = {0x4, 0xe, 0xe, 0xe, 0x1f, 0x0, 0x4};
uint8_t note[8]  = {0x2, 0x3, 0x2, 0xe, 0x1e, 0xc, 0x0};
uint8_t clock[8] = {0x0, 0xe, 0x15, 0x17, 0x11, 0xe, 0x0};
uint8_t heart[8] = {0x0, 0xa, 0x1f, 0x1f, 0xe, 0x4, 0x0};
uint8_t duck[8]  = {0x0, 0xc, 0x1d, 0xf, 0xf, 0x6, 0x0};
uint8_t check[8] = {0x0, 0x1 ,0x3, 0x16, 0x1c, 0x8, 0x0};
uint8_t cross[8] = {0x0, 0x1b, 0xe, 0x4, 0xe, 0x1b, 0x0};
uint8_t retarrow[8] = {0x1, 0x1, 0x5, 0x9, 0x1f, 0x8, 0x4};
uint8_t line_vert1[9] = {0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01};
uint8_t line_vert2[9] = {0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02};
uint8_t line_vert3[9] = {0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04};
uint8_t line_vert4[9] = {0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08};
uint8_t line_vert5[9] = {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10};
uint8_t line_horiz1[9] = {0xff, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
uint8_t line_horiz2[9] = {0x0, 0xff, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
uint8_t line_horiz3[9] = {0x0, 0x0, 0xff, 0x0, 0x0, 0x0, 0x0, 0x0};
uint8_t line_horiz4[9] = {0x0, 0x0, 0x0, 0xff, 0x0, 0x0, 0x0, 0x0};
uint8_t line_horiz5[9] = {0x0, 0x0, 0x0, 0x0, 0xff, 0x0, 0x0, 0x0};
uint8_t line_horiz6[9] = {0x0, 0x0, 0x0, 0x0, 0x0, 0xff, 0x0, 0x0};
uint8_t line_horiz7[9] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xff, 0x0};
uint8_t line_horiz8[9] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xff};
uint8_t fill[9] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

uint32_t custom_char[CUSTOM_CHAR_COUNT] = { bell
                                         , note
                                         , clock
                                         , heart
                                         , duck
                                         , check
                                         , cross
                                         , retarrow
                                         , line_vert1
                                         , line_vert2
                                         , line_vert3
                                         , line_vert4
                                         , line_vert5
                                         , line_horiz1
                                         , line_horiz2
                                         , line_horiz3
                                         , line_horiz4
                                         , line_horiz5
                                         , line_horiz6
                                         , line_horiz7
                                         , line_horiz8
                                         , fill};

void setup()
{
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  /*for(int j=0; j<=CUSTOM_CHAR_COUNT;j++ ){
    lcd.createChar( j, custom_char[j]);
    Serial.println(j); 
  }*/
  
  lcd.createChar(1, bell);
  lcd.createChar(2,note);
  lcd.createChar(3,clock);
  lcd.createChar(4,heart);
  lcd.createChar(5,duck);
  lcd.createChar(6,check);
  lcd.createChar(7,cross);
  lcd.createChar(8,retarrow);
  lcd.createChar(9,line_vert1);
  lcd.createChar(10,line_vert2);
  lcd.createChar(11,line_vert3);
  lcd.createChar(12,line_vert4);
  lcd.createChar(13,line_vert5);
  lcd.createChar(14,line_horiz1);
  lcd.createChar(15,line_horiz2);
  lcd.createChar(16,line_horiz3);
  /*lcd.createChar(17,line_horiz4);
  lcd.createChar(18,line_horiz5);
  lcd.createChar(19,line_horiz6);
  lcd.createChar(20,line_horiz7);
  lcd.createChar(21,line_horiz8);
  lcd.createChar(22,fill);
  */
  lcd.home();

  lcd.print("Hello world...");
  lcd.setCursor(0, 1);
  lcd.print(" i ");
  lcd.write(3);
  lcd.print(" arduinos!");
  delay(5000);
  //displayKeyCodes();
}

void displayChar(uint8_t num, uint8_t time){
  lcd.clear();
  lcd.home();
  lcd.write(num);
  delay(time *1000);
}

// display all keycodes
void displayKeyCodes(void) {
  uint8_t i = 0;

  while (1) {
    lcd.clear();
    lcd.print("Codes 0x");
    lcd.print(i, HEX);
    lcd.print("-0x");
    lcd.print(i + 16, HEX);
    lcd.setCursor(0, 1);

    for (int j = 0; j <= CUSTOM_CHAR_COUNT; j++) {
      lcd.write(i + j);
      delay(100);
    }
    i += 16;

    delay(4000);
  }
}

void loop()
{
  for(int i = 0; i <= CUSTOM_CHAR_COUNT; i++){
    Serial.println(i);
    displayChar(i,2);
  }
  // Do nothing here...
}
