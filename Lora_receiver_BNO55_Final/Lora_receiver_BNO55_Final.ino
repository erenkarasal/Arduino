
#include "Arduino.h"
#include "LoRa_E32.h"
#include "Wire.h"
#define E32_TTL_1W
#define FREQUENCY_433

#include <SoftwareSerial.h>
SoftwareSerial mySerial(10,11); 
LoRa_E32 e32ttl(&mySerial);
// ----------------------------------------
void setup()
{
  Serial.begin(9600);
Wire.begin();
  delay(100);

  e32ttl.begin();

  
  Serial.println();
  Serial.println("Start listening!");


}

//----------------------------------------------
struct Message {
    char type[5];
    char message[8];
    byte Altitude[8];
      byte Acceleration[8];
};

//-----------------------------------------------
void loop()
{
  if (e32ttl.available()  > 1){

    ResponseStructContainer rsc = e32ttl.receiveMessage(sizeof(Message));
    struct Message message = *(Message*) rsc.data;
    Serial.println(message.type);
    
    Serial.print("Yükseklik:");
    Serial.print(*(float*)(message.Altitude));
    Serial.println(" m");
    
    Serial.print("İvme:");
    Serial.println(*(float*)(message.Acceleration));
    Serial.println("*m/s²");
    
    Serial.println(message.message);

    rsc.close();
  }
}
