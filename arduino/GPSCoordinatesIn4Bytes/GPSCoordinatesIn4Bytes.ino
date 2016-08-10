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
#include <Arduino.h>
#include <HTS221.h>
#include "TinyGPS.h"

#define dLat  0.000076409
#define dLng  0.00005164
#define lLat  48.5519972
#define lLng  12.0906633
#define hLat  51.0556997
#define hLng  18.8591456

#define SIGFOX_FRAME_LENGTH 12
#define INTERVAL 6000
#define DEBUG 0

float latitude=0.0f;
float longitude=0.0f;
unsigned long previousSendTime = 0;
unsigned long age;

byte btn1=0;
byte btn2=0;
byte stats=0;

struct data {
  long gps;
/*  float temperature;
  byte humidity;
  byte btn1;
  byte btn2;
  byte stats;*/
  float lat=0;
  float lng=0;
};    


TinyGPS gps;
    
void setup() {
    SerialUSB.begin(115200);
    GPS.begin(9600);
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

   for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (GPS.available())
    {
      char c = GPS.read();
      if (gps.encode(c)); // Did a new valid sentence come in?
    }
  }
     
     unsigned long age;
     gps.f_get_position(&latitude, &longitude, &age);

// latitude   = smeGps.getLatitude();
// longitude  = smeGps.getLongitude();

    // if (inRange(latitude) and inRange(longitude))
      difLat=latitude-lLat;
      difLng=longitude-lLng;
      ctvLat=floor(difLat/dLat);
      ctvLng=floor(difLng/dLng);
      SerialUSB.println(latitude);
      SerialUSB.println(longitude);
      SerialUSB.println(ctvLat);
      SerialUSB.println(ctvLat,BIN);
      SerialUSB.println(ctvLng);
      SerialUSB.println(ctvLng,BIN);
  
      out = ctvLat << 17;
      out = out + ctvLng;
       SerialUSB.println(out,BIN);        
      return out;
      
/*    }
    else
    {
     SerialUSB.println("Mimo rozsah");
     SerialUSB.println(latitude);
     SerialUSB.println(longitude);
    }
    */

}


void loop() {
          
     /* if ((GPS.available()) && ((previousSendTime % 100)==0))
        {
           for (unsigned long start = millis(); millis() - start < 1000;)
          {
            char c = GPS.read();
            if (gps.encode(c)); // Did a new valid sentence come in?
        
          }
        
              
              gps.f_get_position(&latitude, &longitude, &age);
*/
//              latitude   = smeGps.getLatitude();
//              longitude  = smeGps.getLongitude();
//              stats = smeGps.getLockedSatellites();
        ledGreenLight(58);
  

      if (previousSendTime == 0)
        {     


              
              data frame;
              frame.gps = recalc();
              /*frame.temperature = smeHumidity.readTemperature();
              frame.humidity = smeHumidity.readHumidity();            
              frame.btn1=btn1;
              frame.btn2=btn2;
              frame.stats=stats;*/
              frame.lat = latitude;
              frame.lng = longitude;

              SerialUSB.print("lat:");
              SerialUSB.println(latitude,6);
              SerialUSB.print("lng:");
              SerialUSB.println(longitude,6);
              SerialUSB.print("locked:");
              SerialUSB.println(stats);
              
              /* SerialUSB.print("GPS:");
              SerialUSB.println(frame.gps);
              SerialUSB.print("btn1:");
              SerialUSB.println(frame.btn1);
              SerialUSB.print("btn2:");
              SerialUSB.println(frame.btn2);
              */
              bool answer = sendSigfox(&frame, sizeof(data));
                if (answer) {
                    ledBlueLight(HIGH);
                    ledRedLight(LOW);
                  } else {
                    ledBlueLight(LOW);
                    ledRedLight(HIGH);
                  }
              
              previousSendTime=INTERVAL;
              setStepUp(LOW);
              delay(2000);
              ledBlueLight(LOW);
              ledRedLight(LOW);
              btn1=0;
              btn2=0;
        }

        else {
              previousSendTime=previousSendTime-1;
                if (isButtonOnePressed()) {
                  btn1=++btn1;
                  ledRedLight(HIGH);
                  delay(1000);
                  }

                if (isButtonTwoPressed()) {
                  btn2=++btn2;
                  ledGreenLight(HIGH);
                  delay(1000);
                }
              delay(100);
              ledRedLight(LOW);
              ledGreenLight(LOW);
        }

         if (previousSendTime==500) setStepUp(HIGH);
      
}
