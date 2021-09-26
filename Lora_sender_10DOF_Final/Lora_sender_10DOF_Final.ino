#include "Arduino.h"
#include "LoRa_E32.h"
#include "MPU9250.h"
#include <Adafruit_BMP280.h>
#include <Wire.h>
Adafruit_BMP280 bmp;
MPU9250 mpu;
int status;
#define E32_TTL_1W
#define FREQUENCY_433


#include <SoftwareSerial.h>
SoftwareSerial mySerial(10,11); 
LoRa_E32 e32ttl(&mySerial); 
void printParameters(struct Configuration configuration);
void printModuleInformation(struct ModuleInformation moduleInformation);
//The setup function is called once at startup of the sketch
void setup()
{
  Serial.begin(9600);
  Wire.begin();
  delay(2000);
  
//...........Kurtama.................................................

pinMode(8, OUTPUT);//led pinmode on 
//*********************************************************

    if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }
    if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

   bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

//...........................................................
  e32ttl.begin();
}
struct Message {
//    char type[5];
//    char message[8];
    int Altitude;
    int Acceleration;
    int Roll;
} message;

int i = 0;
// The loop function is called in an endless loop
void loop()
{

//...................................................
if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 25) {
                      
            prev_ms = millis();
            //...........Kurtama....................
            //Serial.println(mpu.getRoll(), 2);
             float kurtarma_1 =mpu.getRoll();
             float kurtarma_1_2 = mpu.getAccY()*10.6;
             if(kurtarma_1<-90)
             {
              digitalWrite(8, HIGH);
              
             }
             else if(kurtarma_1_2<9.804){
               digitalWrite(8, HIGH);
             }
             else {
              digitalWrite(8, LOW);
             }
            double rakim = bmp.readAltitude(1013.25);
            double istenen_rakim=rakim-794;
            if(istenen_rakim> 50){
              digitalWrite(8, HIGH);
              
            }
             else {
              digitalWrite(8, LOW);
            }
            //...................................................    
            //****************MESAJ******************************
                //delay(2500);
                i++;
                struct Message {
                    //char type[5] = "Alti";
                    //char message[8] = "Altitu";
                    byte Altitude[4];
                    byte Acceleration[8];
                    byte Roll[16];
                } message;
              
              
               
                  
                *(float*)(message.Altitude) = bmp.readAltitude(1013.25);
                *(float*)(message.Acceleration) = mpu.getAccY()*10,6;
                *(float*)(message.Roll) = mpu.getRoll(),2;
                
                Serial.println(mpu.getAccY()*10,6);
                 
                ResponseStatus rs = e32ttl.sendFixedMessage(0,2,6,&message, sizeof(Message));
                Serial.println(rs.getResponseDescription());
            }
        }
     
     

}
