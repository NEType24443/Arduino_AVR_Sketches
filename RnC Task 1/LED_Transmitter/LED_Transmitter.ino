#define LED 7   // Transmitter LED
#define time_period 150   // 150 ms time period
void setup() {
  pinMode(LED, OUTPUT);
  Serial.begin(115200);
}
// Below is the string to transmit all characters except '? , ; , %, , ' are sent
const int str[] = {0x74,0x68,0x3b,0x65,0x20,0x71,0x75,0x69,0x3f,0x63,0x6b,0x20,0x3f,0x62,0x72,0x6f,0x3b,0x2c,0x77,0x6e,0x3b,0x2c,0x20,0x66,0x6f,0x78,0x20,0x6a,0x75,0x6d,0x70,0x3f,0x3f,0x3f,0x3f,0x3b,0x3f,0x3f,0x3f,0x3f,0x73,0x20,0x6f,0x76,0x65,0x72,0x20,0x74,0x68,0x3b,0x65,0x2c,0x20,0x6c,0x3b,0x61,0x7a,0x79,0x20,0x64,0x6f,0x67,0x2e,0x3f,0x25,0x2e,0x2e};
void loop() {
  int i = 0 ;
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
  delay(10);      //Start bit
  int delay_val = 0;
  for(char c; c != '\0';){
    c = str[i];
    i++;
    Serial.print(c);
    delay_val = pulseLengthEncoder(c); // Varying ON time
    if(delay_val){    //If value is there in Encoder Table then only transmit
      digitalWrite(LED, HIGH);
      delay(delay_val);   //ON time 
      digitalWrite(LED, LOW);
      delay(time_period - delay_val); // Corresponding OFF time
    }
  }
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW); // Stop bit - same as start
  while(true)delay(5000); //Infinite loop to not resend 
}
int pulseLengthEncoder(char c){
  switch (c){
    default:
      return(0);
    case ' ':
      return(10);
    case 'a':
      return(15);
    case 'b':
      return(20);
    case 'c':
      return(25);
    case 'd':
      return(30);
    case 'e':
      return(35);
    case 'f':
      return(40);
    case 'g':
      return(45);
    case 'h':
      return(50);
    case 'i':
      return(55);
    case 'j':
      return(60);
    case 'k':
      return(65);
    case 'l':
      return(70);
    case 'm':
      return(75);
    case 'n':
      return(80);
    case 'o':
      return(85);
    case 'p':
      return(90);
    case 'q':
      return(95);
    case 'r':
      return(100);
    case 's':
      return(105);
    case 't':
      return(110);
    case 'u':
      return(115);
    case 'v':
      return(120);
    case 'w':
      return(125);
    case 'x':
      return(130);
    case 'y':
      return(135);
    case 'z':
      return(140);
    case '.':
      return(145);
    }
}
