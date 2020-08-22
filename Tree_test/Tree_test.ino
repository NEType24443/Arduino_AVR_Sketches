#define SR 0
#define ST 1
#define RT 2
#define LT 3
#define RS 4
#define LS 5
#define UT 6
#define EN 7 

int dir[50] ={};
int curr_node;
void add_node(int toNode, int value){
       if(value==SR)  dir[2*toNode + 0] = value;
  else if(value==ST)  dir[2*toNode + 1] = value;
  else if(value==RT)  dir[2*toNode + 2] = value;
  else if(value==LT)  dir[2*toNode + 3] = value;
  else if(value==RS)  dir[2*toNode + 4] = value;
  else if(value==LS)  dir[2*toNode + 5] = value;
  else if(value==UT)  dir[2*toNode + 6] = value;
  else if(value==EN)  dir[2*toNode + 7] = value;
}
void setup()  {
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
