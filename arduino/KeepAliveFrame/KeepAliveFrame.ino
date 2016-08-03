/* === Sigfox Keep Alive Frame===
 *  
 *  it is not listed in received messages at Sigfox cloud
 *  Only when you have activated service callbacks (Device Type --> Callbacks --> Service) you could get this service notice.
 *  
 *  To send Keep Alive Frame press userButton1   
 *    
 *    
 */


#include <Wire.h>
#include <SmeSFX.h>
#include <Arduino.h>

char sfcCommandMsg[ ]="AT$SK";


void setup() {
    
    SerialUSB.begin(115200);
    sfxAntenna.begin();
    ledBlueLight(HIGH);
    sfxAntenna.setSfxConfigurationMode(); // enter in configuration Mode
    
}

void sendMsg()  {
        
        sfxAntenna.sfxSendConf(sfcCommandMsg,5); // send the data
        SerialUSB.println("SENDING DATA");
          ledGreenLight(LOW);
          
            while (!sfxAntenna.hasSfxAnswer()){
                    delay(1000);
                    ledRedLight(HIGH);
                    }
                       ledRedLight(LOW);
                       if (sfxAntenna.getSfxError() == SME_SFX_OK) {
                         SerialUSB.println("Command accepted !!");
                         SerialUSB.println((const char*)sfxAntenna.getLastReceivedMessage());
                          ledGreenLight(HIGH);          
                        }
                    
         
}

void loop() {
        ledGreenLight(HIGH);
        
                    
            if (isButtonOnePressed()) { //button #1
                sendMsg();
              }
       else {
              ledBlueLight(HIGH);
              delay(1000);
              ledBlueLight(LOW);
              
            }
      
}

