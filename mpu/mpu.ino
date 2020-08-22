#include<Wire.h>
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,GyX,AcY,GyY,AcZ,GyZ; // declare accellerometer and gyro variables and int16_t limits the variable to 16 bits
void setup(){
  Wire.begin(); // initiate i2c system and join as master
  Wire.beginTransmission(MPU_addr); 
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true); // 
  Serial.begin(9600); 
}
void loop(){ // main program loop
  Wire.beginTransmission(MPU_addr); 
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false); 
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  // <<8| is to bit shift the first 8 bits to
  // the left & combine it with the next 8 bits to form 16 bits
  AcX=Wire.read()<<8|Wire.read();      
  GyX=Wire.read()<<8|Wire.read();  
 // AcY=Wire.read()<<8|Wire.read();      
  //GyY=Wire.read()<<8|Wire.read();
 // AcZ=Wire.read()<<8|Wire.read();      
 // GyZ=Wire.read()<<8|Wire.read();
  Serial.print("AcX = "); Serial.print(AcX); 
  Serial.print(" | GyX = "); Serial.println(GyX); 
  //Serial.print("AcY = "); Serial.print(AcY); 
  //Serial.print(" | GyY = "); Serial.println(GyY);
  //Serial.print("AcZ = "); Serial.print(AcZ); 
 // Serial.print(" | GyZ = "); Serial.println(GyZ);
  delay(2000); 
}
