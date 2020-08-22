//#include<SPI.h>
void setup() {
  pinMode(SS, INPUT);
  pinMode(MOSI,INPUT);
  pinMode(SCK,INPUT);
  Serial.begin(115200);
  SPCR = B11100011;
  SPSR = B00000000;
}
//                  _________SPCR__________
//| 7    | 6    | 5    | 4    | 3    | 2    | 1    | 0    |
//| SPIE | SPE  | DORD | MSTR | CPOL | CPHA | SPR1 | SPR0 |
//|   1  |  1   |   1   |  0  |   0  |  0   |   1  |   1  |

//                  _________SPSR__________
//| 7    | 6    | 5    | 4    | 3    | 2    | 1    | 0    |
//| SPIF | WCOL |      |      |      |      |      | SPI2X|
//|   0  |  0   |   0  |  0   |  0   |  0   |   0  |   0  |

volatile bool data_recieved;
char buff[100];
volatile uint8_t pos; 

ISR (SPI_STC_vect)
{
byte data = SPDR;
 // add to buffer if room
 if (pos < sizeof buff)
   {
   buff [pos] = data;
   pos++;
   // example: newline means time to process buffer
   if (data == '\n')
     data_recieved = true;
     Serial.println(data);
     
   }  // end of room available
}
byte SPI_Send(byte data){
  SPDR = data;
  while(!(SPSR &(1<<SPIF)));
  return SPDR;
}
void loop(){
  if(data_recieved){
    char data = buff;
    //if (data == "0x01"){
      SPI_Send(0x01);
      data_recieved = false;
      buff[pos] = 0;  
      Serial.println(buff);
      pos = 0;
    //}
  }
}
