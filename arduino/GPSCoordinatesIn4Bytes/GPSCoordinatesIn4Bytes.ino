/*
 * GPS - compress GPS data (lat, lng) only to 4 bytes
 * 
 * this sketch is designed for using in CZECH REPUBLIC
 * sum of latitude (2,5037025 deg) is divided by 15bits (32767sq)
 * sum of longitude (6,7684823 deg) is divided by 17bits (131071sq)
 * 
 * we have 32767 x 131071 squares
 * 
 * space between 48.5519972N - 51.0556997N
 *        AND    12.0906633E - 18.8591456E,
 *        
 *        which is the rectangle bounding outside the Czech Republic
 *        
 * we could divide to 0.000076409 (lat) and 0.00005164 (lon) parts, which each one represent one bit
 * 
 */


#include <Wire.h>
#include <sl868a.h>
#include <Arduino.h>
#include <HTS221.h>
#define dLat  0.000076409
#define dLng  0.00005164
#define lLat  48.5519972
#define lLng  12.0906633
#define hLat  51.0556997
#define hLng  18.8591456

#define SIGFOX_FRAME_LENGTH 12
#define INTERVAL 600
#define DEBUG 0

double latitude = 0;
double longitude = 0;
unsigned long previousSendTime = 0;

struct data {
  long gps;
  float temperature;
};    
    
void setup() {
    SerialUSB.begin(115200);
    smeGps.begin();
    smeHumidity.begin();
    SigFox.begin(19200);
    initSigfox();
    setStepUp(HIGH);
}


void initSigfox(){
  SigFox.print("+++");
  while (!SigFox.available()){
    delay(100);
  }
  while (SigFox.available()){
    byte serialByte = SigFox.read();
   }
 
}


String getSigfoxFrame(const void* data, uint8_t len){
  String frame = "";
  uint8_t* bytes = (uint8_t*)data;
  
  if (len < SIGFOX_FRAME_LENGTH){
    //fill with zeros
    uint8_t i = SIGFOX_FRAME_LENGTH;
    while (i-- > len){
      frame += "00";
    }
  }

  //0-1 == 255 --> (0-1) > len
  for(uint8_t i = len-1; i < len; --i) {
    if (bytes[i] < 16) {frame+="0";}
    frame += String(bytes[i], HEX);
  }
  
  return frame;
}

bool sendSigfox(const void* data, uint8_t len){
  String frame = getSigfoxFrame(data, len);
  String status = "";
  char output;
  if (DEBUG){
    SerialUSB.print("AT$SF=");
    SerialUSB.println(frame);
  }
  SigFox.print("AT$SF=");
  SigFox.print(frame);
  SigFox.print("\r");
  while (!SigFox.available());
  
  while(SigFox.available()){
    output = (char)SigFox.read();
    status += output;
    delay(10);
  }
  if (DEBUG){
    SerialUSB.print("Status \t");
    SerialUSB.println(status);
  }
  if (status == "OK\r"){
    //Success :)
    return true;
  }
  else{
    return false;
  }
}



bool inRange(double in) {
   if ((in>lLat and in<hLat) or (in>lLng and in<hLng))
      return true;
      else return false;
  }

unsigned long recalc() {
    double difLat;
    double difLng;
    unsigned int    ctvLat;
    unsigned int    ctvLng;
    unsigned long out;
    if (inRange(latitude) and inRange(longitude))
    { difLat=latitude-lLat;
      difLng=longitude-lLng;
      ctvLat=floor(difLat/dLat);
      ctvLng=floor(difLng/dLng);
      SerialUSB.println(ctvLat);
      SerialUSB.println(ctvLat,BIN);
      SerialUSB.println(ctvLng);
      SerialUSB.println(ctvLng,BIN);
  
      out = ctvLat << 17;
      out = out + ctvLng;
        SerialUSB.println(out,BIN);        
      return out;
      
    }


}


void loop() {
    // if (smeGps.ready()) {
        ledGreenLight(58);
        //latitude   = smeGps.getLatitude();
        // longitude  = smeGps.getLongitude();
        latitude = 50.1402078;
        longitude = 14.5077375;
        if (previousSendTime == 0)
        {     data frame;
              frame.gps = recalc();
              frame.temperature = smeHumidity.readTemperature();            
              bool answer = sendSigfox(&frame, sizeof(data));
                if (answer) {
                    ledBlueLight(HIGH);
                    ledRedLight(LOW);
                  } else {
                    ledBlueLight(LOW);
                    ledRedLight(HIGH);
                  }
                  
              previousSendTime=INTERVAL;
              delay(2000);
              ledBlueLight(LOW);
              ledRedLight(LOW);
        }

        else {
              previousSendTime=previousSendTime-1;
              delay(1000);
        }

        
      //}

    //else ledGreenLight(LOW);

}
