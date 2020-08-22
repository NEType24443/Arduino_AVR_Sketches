#include<Wire.h>
#define BMP180_ADDRREAD 0xEE
#define BMP180_ADDRWRITE 0xEF
#define  BMP180_REG_CONTROL 0xF4
#define BMP180_REG_RESULT 0xF6

#define BMP180_COMMAND_TEMPERATURE 0x2E
#define BMP180_COMMAND_PRESSURE3 0xF4

float AC1 ,AC2 ,AC3 ,AC4 ,AC5 ,AC6 ,B1 ,B2 ,MB ,MC ,MD; 
int oss;
void setup() {
  __init();
  Wire.begin();
}

void loop() {
  Wire.beginTransmission(BMP180_ADDRWRITE);
  Wire.write(BMP180_REG_CONTROL);
  Wire.write(0x2E);
  delay(5);
  Wire.requestFrom(BMP180_REG_RESULT, 1);  // MSB
  byte msb = Wire.read();
  Wire.requestFrom(BMP180_REG_RESULT + 0x01, 1);  //LSB
  byte lsb = Wire.read();
  uint16_t ut = (msb << 8) | lsb;
  Wire.endTransmission();
  int Temperature = realTemp(ut);
  delay(100);
  
  Wire.beginTransmission(BMP180_ADDRWRITE);
  Wire.write(BMP180_REG_CONTROL);
  Wire.write(0x34);
  delay(5);
  Wire.requestFrom(BMP180_REG_RESULT, 1);  // MSB
  byte msb = Wire.read();
  Wire.requestFrom(BMP180_REG_RESULT + 0x01, 1);  //LSB
  byte lsb = Wire.read();
  Wire.requestFrom(BMP180_REG_RESULT + 0x02, 1);  //XLSB
  byte xlsb = Wire.read();
  uint16_t up = (((msb << 16) + (lsb<<8))>>oss) + xlsb;
  Wire.endTransmission();
  int Pressure = realPress(up);
  delay(10000);
}
int realTemp(uint16_t ut){
  float X1= (ut - AC6)*AC5/(2^15);
  float X2= MC * (2^11)/(X1 + MD);
  float B5 = X1 + X2
  return((B5 + 8)/(2^4));
}
int realPress(uint16_t up){
  float B6 = B5- 4000;
  float X1 = (B2*(B6*B6/(2^12))/(2^11);
  float X2 = AC2*B6/(2^11);
  float X3 = X1 + X2;
  B3 = (((AC1*4+X3)<<oss)+2)/4;
  X1 = AC4 * B6/2^13;
  X2 = (B1 * (B6*B6/(2^12))/2^16;
  X3 = ((X1+X2)+2)/2^2;
  B4 = AC4*(unsigned long)up - B3)*(50000>>oss);
  B7 = 
  if()
}
char __init()
// Initialize library for subsequent pressure measurements
{
  double c3,c4,b1;
  Wire.begin();
  // The BMP180 includes factory calibration data stored on the device.
  // Each device has different numbers, these must be retrieved and
  // used in the calculations when taking pressure measurements.
  // Retrieve calibration data from device:
  /*if (readInt(0xAA,AC1) &&
    readInt(0xAC,AC2) &&
    readInt(0xAE,AC3) &&
    readUInt(0xB0,AC4) &&
    readUInt(0xB2,AC5) &&
    readUInt(0xB4,AC6) &&
    readInt(0xB6,VB1) &&
    readInt(0xB8,VB2) &&
    readInt(0xBA,MB) &&
    readInt(0xBC,MC) &&
    readInt(0xBE,MD)){*/
  if(1){
    // If you need to check your math using known numbers,
    // you can uncomment one of these examples.
    // (The correct results are commented in the below functions.)

    // Example from Bosch datasheet
     AC1 = 408; 
     AC2 = -72;
     AC3 = -14383;
     AC4 = 32741;
     AC5 = 32757; 
     AC6 = 23153;
     B1 = 6190;
     B2 = 4;
     MB = -32768;
     MC = -8711;
     MD = 2868;
    // Example from http://wmrx00.sourceforge.net/Arduino/BMP180-Calcs.pdf
    // AC1 = 7911; AC2 = -934; AC3 = -14306; AC4 = 31567; AC5 = 25671; AC6 = 18974;
    // VB1 = 5498; VB2 = 46; MB = -32768; MC = -11075; MD = 2432;
    // Compute floating-point polynominals:

    /*c3 = 160.0 * pow(2,-15) * AC3;
    c4 = pow(10,-3) * pow(2,-15) * AC4;
    b1 = pow(160,2) * pow(2,-30) * VB1;
    c5 = (pow(2,-15) / 160) * AC5;
    c6 = AC6;
    mc = (pow(2,11) / pow(160,2)) * MC;
    md = MD / 160.0;
    x0 = AC1;
    x1 = 160.0 * pow(2,-13) * AC2;
    x2 = pow(160,2) * pow(2,-25) * VB2;
    y0 = c4 * pow(2,15);
    y1 = c4 * c3;
    y2 = c4 * b1;
    p0 = (3791.0 - 8.0) / 1600.0;
    p1 = 1.0 - 7357.0 * pow(2,-20);
    p2 = 3038.0 * 100.0 * pow(2,-36);*/
    return(1);
  }
  else{
    // Error reading calibration data; bad component or connection?
    return(0);
  }
}
char readUInt(char address, uint16_t &value)
// Read an unsigned integer (two bytes) from device
// address: register to start reading (plus subsequent register)
// value: external variable to store data (function modifies value)
{
  unsigned char data[2];

  data[0] = address;
  if (readBytes(data,2)){
    value = (((uint16_t)data[0]<<8)|(uint16_t)data[1]);
    return(1);
  }
  value = 0;
  return(0);
}
char readInt(char address, int16_t &value)
// Read a signed integer (two bytes) from device
// address: register to start reading (plus subsequent register)
// value: external variable to store data (function modifies value)
{
  unsigned char data[2];

  data[0] = address;
  if (readBytes(data,2))
  {
    value = (((int16_t)data[0]<<8)|(int16_t)data[1]);
    //if (*value & 0x8000) *value |= 0xFFFF0000; // sign extend if negative
    return(1);
  }
  value = 0;
  return(0);
}
char writeBytes(unsigned char *values, char length)
// Write an array of bytes to device
// values: external array of data to write. Put starting register in values[0].
// length: number of bytes to write
{
  char x;

  Wire.beginTransmission(BMP180_ADDRWRITE);
  Wire.write(values,length);
  _error = Wire.endTransmission();
  if (_error == 0)
    return(1);
  else
    return(0);
}
char readBytes(unsigned char *values, char length)
// Read an array of bytes from device
// values: external array to hold data. Put starting register in values[0].
// length: number of bytes to read
{
  char x;

  Wire.beginTransmission(BMP180_ADDRREAD);
  Wire.write(values[0]);
  _error = Wire.endTransmission();
  if (_error == 0)
  {
    Wire.requestFrom(BMP180_ADDR,length);
    for(x=0;x<length;x++)
    {
      values[x] = Wire.read();
    }
    return(1);
  }
  return(0);
}
