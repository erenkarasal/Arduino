#include "Arduino.h"
#include "LoRa_E32.h"
#include "MPU9250.h"
#include <Adafruit_BMP280.h>
#include <Wire.h>
#define E32_TTL_1W
#define FREQUENCY_433


Adafruit_BMP280 bmp;
MPU9250 mpu;
int status;

#include <SoftwareSerial.h>
SoftwareSerial mySerial(10,11); // Arduino RX <-- e32 TX, Arduino TX --> e32 RX
LoRa_E32 e32ttl(&mySerial);//.........................., 5, 7, 6
// -------------------------------------
void printParameters(struct Configuration configuration);
void printModuleInformation(struct ModuleInformation moduleInformation);
//The setup function is called once at startup of the sketch
void setup()
{
  Serial.begin(9600);
//  while (!Serial) {
//      ; // wait for serial port to connect. Needed for native USB
//    }
  delay(100);

  e32ttl.begin();

  Serial.println();
  Serial.println("Start listening!");
 //   e32ttl.setMode(MODE_2_POWER_SAVING);

}
struct Message {
//    char type[5];
//    char message[8];
    byte Altitude[4];
    byte Acceleration[8];
    byte Roll[16];
};

// The loop function is called in an endless loop
void loop()
{
  static uint32_t prev_ms = millis();
  if (millis()>prev_ms + 25){
    
  
  if (e32ttl.available()  > 1){

    ResponseStructContainer rsc = e32ttl.receiveMessage(sizeof(Message));
    struct Message message = *(Message*) rsc.data;
//  Serial.println(message.type);

    Serial.println(*(float*)(message.Altitude));
    Serial.println(*(float*)(message.Acceleration));
    Serial.println(*(float*)(message.Roll));
//   Serial.println(message.message);
//    free(rsc.data);
    rsc.close();
  }
  }
}
