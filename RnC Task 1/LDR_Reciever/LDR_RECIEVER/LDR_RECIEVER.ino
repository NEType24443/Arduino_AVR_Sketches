#define DATA A0
#define THRESHOLD 900 //900 for stability and transmitter 
                      //should be less than 1cm away from reciever
void setup(){
  pinMode( DATA, INPUT);
  Serial.begin(115200);
}

uint16_t value = 0, pulselen = 0, ton = 0, toff = 0, i = 0;
char data, recieved[50];
bool stop_bit_flag = false; 

void loop() {
  value = analogRead(DATA);
  if (value >THRESHOLD){
    pulseLengthDecoder();
  }
  if (stop_bit_flag) {
    Serial.print("Recieved message = \"");
    for(i = 0; i < sizeof(recieved); ++i)
      Serial.print(recieved[i]);
    Serial.print("\"\n");
    Serial.println("erasing recieved...");
    for(i = 0; i < sizeof(recieved);  ++i )
      recieved[i] = (char)0;
    i = 0;
    stop_bit_flag = false;
  }
}

void pulseLengthDecoder(void){    // decodes the ON time into charactes
  ton = millis();   //time when LED turns on
  while (analogRead(DATA)>THRESHOLD); 
  toff = millis();    //time when LED turns OFF
  pulselen = toff - ton;
  Serial.print(pulselen);
  if(10<=pulselen && pulselen<15){    // all statements for finding value in a certain range
      data = ' ';recieved[i-2] = data;}
  else if(15<=pulselen && pulselen<20){    
      data = 'a';recieved[i-2] = data;}
  else if(20<=pulselen && pulselen<25){
      data = 'b';recieved[i-2] = data;}
  else if(25<=pulselen && pulselen<30){
      data = 'c';recieved[i-2] = data;}
  else if(30<=pulselen && pulselen<35){
      data = 'd';recieved[i-2] = data;}
  else if(35<=pulselen && pulselen<40){    
      data = 'e';recieved[i-2] = data;}
  else if(40<=pulselen && pulselen<45){    
      data = 'f';recieved[i-2] = data;}
  else if(45<=pulselen && pulselen<50){    
      data = 'g';recieved[i-2] = data;}
  else if(50<=pulselen && pulselen<55){    
      data = 'h';recieved[i-2] = data;}
  else if(55<=pulselen && pulselen<60){    
      data = 'i';recieved[i-2] = data;}
  else if(60<=pulselen && pulselen<65){    
      data = 'j';recieved[i-2] = data;}
  else if(65<=pulselen && pulselen<70){    
      data = 'k';recieved[i-2] = data;}
  else if(70<=pulselen && pulselen<75){    
      data = 'l';recieved[i-2] = data;}
  else if(75<=pulselen && pulselen<80){    
      data = 'm';recieved[i-2] = data;}
  else if(80<=pulselen && pulselen<85){    
      data = 'n';recieved[i-2] = data;}
  else if(85<=pulselen && pulselen<90){    
      data = 'o';recieved[i-2] = data;}
  else if(90<=pulselen && pulselen<95){    
      data = 'p';recieved[i-2] = data;}
  else if(95<=pulselen && pulselen<100){    
      data = 'q';recieved[i-2] = data;}
  else if(100<=pulselen && pulselen<105){    
      data = 'r';recieved[i-2] = data;}
  else if(105<=pulselen && pulselen<110){    
      data = 's';recieved[i-2] = data;}
  else if(110<=pulselen && pulselen<115){    
      data = 't';recieved[i-2] = data;}
  else if(115<=pulselen && pulselen<120){    
      data = 'u';recieved[i-2] = data;}
  else if(120<=pulselen && pulselen<125){    
      data = 'v';recieved[i-2] = data;}
  else if(125<=pulselen && pulselen<130){    
      data = 'w';recieved[i-2] = data;}
  else if(130<=pulselen && pulselen<135){    
      data = 'x';recieved[i-2] = data;}
  else if(135<=pulselen && pulselen<140){    
      data = 'y';recieved[i-2] = data;}
  else if(140<=pulselen && pulselen<145){    
      data = 'z';recieved[i-2] = data;}
  else if(145<=pulselen && pulselen<150){    
      data = '.';recieved[i-2] = data;}
  else if(990<=pulselen && pulselen<=1010){
    if (!i){
      Serial.println(" - start bit");
      i++;
    }
    else{
      Serial.println(" - stop bit");
      stop_bit_flag = true;
    }
  }
  if(i-1 && !stop_bit_flag){
    i++;
    Serial.print(" - ");
    Serial.println(data);   // printing the characters with its time duration
  }
  else i++;
}
