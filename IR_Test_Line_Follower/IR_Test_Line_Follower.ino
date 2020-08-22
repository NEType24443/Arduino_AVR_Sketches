
#define  sensor1 A0     // RIGHT most sensor
#define  sensor2 A1     
#define  sensor3 A2
#define  sensor4 A3    
#define  sensor5 A4     // LEFT most sensor
#define  sensorL A5
#define  sensorR 10

int ir[8];

void setup() {
  // put your setup code here, to run once:
  pinMode(sensor1,INPUT);
  pinMode(sensor2,INPUT);
  pinMode(sensor3,INPUT);
  pinMode(sensor4,INPUT);
  pinMode(sensor5,INPUT);
  pinMode(sensorL,INPUT);
  pinMode(sensorR,INPUT);
  //pinMode(sensorL, INPUT);
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  find_position();
  delay(1000);  
}
void find_position(){ 
   ir[1] = digitalRead(sensor1);
  ir[1]=(!ir[1]);
  Serial.print("Sens1: ");
  Serial.println(ir[1]);
  ir[2] = digitalRead(sensor2);
  ir[2]=(!ir[2]);
  Serial.print("Sens2: ");
  Serial.println(ir[2]);
  ir[3] = digitalRead(sensor3);
  ir[3]=(!ir[3]);
  Serial.print("Sens3: ");
  Serial.println(ir[3]);
  ir[4] = digitalRead(sensor4);
  ir[4]=(!ir[4]);
  Serial.print("Sens4: ");
  Serial.println(ir[4]);
  ir[5] = digitalRead(sensor5);
  ir[5]=(!ir[5]);
  Serial.print("Sens5: ");
  Serial.println(ir[5]);
  Serial.println();
  ir[6] = digitalRead(sensorL);
  ir[6]=(!ir[6]);
  Serial.println(ir[6]);
  ir[7] = digitalRead(sensorR);
  ir[7]=(!ir[7]);
  Serial.println(ir[7]);
}
