#include<SoftwareSerial.h>
#include<Streaming.h>
const unsigned char UBX_HEADER[] = { 0xB5, 0x62 };

struct NAV_POSLLH {
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  unsigned long iTOW;
  long lon;
  long lat;
  long height;
  long hMSL;
  unsigned long hAcc;
  unsigned long vAcc;
};

NAV_POSLLH posllh;
SoftwareSerial ss(D5,D0);

void calcChecksum(unsigned char* CK) {
  memset(CK, 0, 2);
  for (int i = 0; i < (int)sizeof(NAV_POSLLH); i++) {
    CK[0] += ((unsigned char*)(&posllh))[i];
    CK[1] += CK[0];
  }
}

bool processGPS() {
  static int fpos = 0;
  static unsigned char checksum[2];
  const int payloadSize = sizeof(NAV_POSLLH);

  while ( ss.available() ) {
    byte c = ss.read();
    if ( fpos < 2 ) {
      if ( c == UBX_HEADER[fpos] )
        fpos++;
      else
        fpos = 0;
    }
    else {
      if ( (fpos-2) < payloadSize )
        ((unsigned char*)(&posllh))[fpos-2] = c;

      fpos++;

      if ( fpos == (payloadSize+2) ) {
        calcChecksum(checksum);
      }
      else if ( fpos == (payloadSize+3) ) {
        if ( c != checksum[0] )
          fpos = 0;
      }
      else if ( fpos == (payloadSize+4) ) {
        fpos = 0;
        if ( c == checksum[1] ) {
          return true;
        }
      }
      else if ( fpos > (payloadSize+4) ) {
        fpos = 0;
      }
    }
  }
  return false;
}

void setup() 
{
  Serial.begin(9600);
  ss.begin(9600);
  pinMode(D1, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(D1, 0);
}

void loop() {
  if ( processGPS() ) 
    // do something with GPS data
    Serial<< "iTOW: "      << posllh.iTOW           <<"   "
          << "Longitude: " << posllh.lon            <<"   "  // /10000000.0f
          << "Latitude: "  << posllh.lat            <<"   "   // /10000000.0f
          << "Height: "    << posllh.height/1000.0f <<"   "
          << "hMSL: "      << posllh.hMSL/1000.0f   <<"   "
          << "hAccel: "    << posllh.hAcc/1000.0f   <<"   "
          << "vAccel: "    << posllh.vAcc/1000.0f   <<"   "
          << endl;
  
  digitalWrite(LED_BUILTIN,millis()%1000>950);
}
