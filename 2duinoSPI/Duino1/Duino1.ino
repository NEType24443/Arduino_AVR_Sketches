//#include<SPI.h>
#define SLAVE_ADDR 1
const uint8_t button = 3;

uint8_t text = 47;

void setup() {
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(SS, OUTPUT);
  pinMode(button, INPUT);
  digitalWrite(SS, HIGH);
  Serial.begin(115200);
  SPCR = B11110000;
  SPSR = B00000000;
}
//                  _________SPCR__________
//| 7    | 6    | 5    | 4    | 3    | 2    | 1    | 0    |
//| SPIE | SPE  | DORD | MSTR | CPOL | CPHA | SPR1 | SPR0 |
//|   1  |  1   |   1   |  0  |   0  |  0   |   1  |   1  |

//                  _________SPSR__________
//| 7    | 6    | 5    | 4    |  3  | 2    | 1    | 0    |
//| SPIF | WCOL |      |      |     |      |      | SPI2X|
//|   0  |  0   |   0  |  0   |  0  |  0   |   0  |   0  |
int SPI_Recieve(){
  while(!(SPSR & (1<<SPIF)));
  return SPDR;
}

byte SPI_Send(byte data){
  SPDR = data;
  Serial.print("Data Sent: ");
  Serial.println(data);
  while(!(SPSR & (1<<SPIF)));
}
bool lastState = HIGH;
void loop(){
  delay(10000);
  Serial.println(digitalRead(button));
  if(digitalRead(button)){
    //lastState = HIGH;
    Serial.println("SPI Begin");
    delay(10);
    digitalWrite(SS, LOW);
    //delay(10);
    byte ack = SPI_Send(SLAVE_ADDR); 
    Serial.print("ACK: ");
    Serial.println(ack);
    if(ack == 0x03){
      SPI_Send(text);
    }  
  }
  digitalWrite(SS, HIGH);
}
