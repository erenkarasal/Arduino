
#include "Arduino.h"
#include "LoRa_E32.h"
#define E32_TTL_1W
#define FREQUENCY_433


#include <SoftwareSerial.h>
SoftwareSerial mySerial(10,11); 
LoRa_E32 e32ttl(&mySerial); 


//--------------------------------------------------

void setup()
{
  Serial.begin(9600);

  delay(100);

  e32ttl.begin();

}
//----------------------------------------------------

struct Message {
    char type[5];
    char message[8];
    int temperature;
} message;

int i = 0;
//-----------------------------------------------------

void loop()
{
  delay(2500);
  i++;
  struct Message {
      char type[5] = "TEMP";
      char message[8] = "Kitchen";
      byte temperature[4];
  } message;

  *(float*)(message.temperature) = 20;

  ResponseStatus rs = e32ttl.sendFixedMessage(0,2,6,&message, sizeof(Message));
  Serial.println(rs.getResponseDescription());
}
