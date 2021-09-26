#include "Wire.h"
#include "DFRobot_BMP280.h"
#include "DFRobot_BNO055.h"
#include "Arduino.h"
#include "LoRa_E32.h"
#define E32_TTL_1W
#define FREQUENCY_433

//-----------------------------------------------------------------------------------------
typedef DFRobot_BNO055_IIC    BNO;    // ******** use abbreviations instead of full names ********
typedef DFRobot_BMP280_IIC    BMP;
BNO   bno(&Wire, 0x28);    // input TwoWire interface and IIC address
BMP   bmp(&Wire, BMP::eSdo_low);
#define SEA_LEVEL_PRESSURE    1013.25f 
//-----------------------------------------------------------------------------------------
#include <SoftwareSerial.h>
SoftwareSerial mySerial(10,11); 
LoRa_E32 e32ttl(&mySerial); 


//--------------------------------------------------

void setup()
{
  Serial.begin(9600);
  Wire.begin(); 
  //--------------------------------------------------
  bno.reset();
  while(bno.begin() != BNO::eStatusOK) {
    Serial.println("bno begin faild");
     
    delay(2000);
  }
  Serial.println("bno begin success");
  //------------------------------------------------------------
   bmp.reset();
  Serial.println("bmp read data test");
  while(bmp.begin() != BMP::eStatusOK) {
    Serial.println("bmp begin faild");
    
  }
  Serial.println("bmp begin success");
  //-------------------------------------------------------------
  
 
  delay(100);
  e32ttl.begin();
  
}
//----------------------------------------------------

struct Message {
    //char type[5];
    //char message[8];
    int Altitude;
    int Acceleration;
} message;
int i = 0;
//----------------------------------------------------
#define printAxisData(sAxis) \
  Serial.print(" x: "); \
  Serial.print(sAxis.x); \
  Serial.print(" y: "); \
  Serial.print(sAxis.y); \
  Serial.print(" z: "); \
  Serial.println(sAxis.z)

//-----------------------------------------------------

void loop()
{
  
  static uint32_t prev_ms = millis();
  if (millis() > prev_ms + 25) {
                      
            prev_ms = millis();
  //******************************************************
  //delay(100);
  i++;  
  struct Message {
      //char type[5] = "BNO";
      //char message[8] = "BNO55";
      byte Altitude[8];
      byte Acceleration[8];
  } message;
  //------------------------------------------------------
  BNO::sAxisAnalog_t   sAccAnalog;
  sAccAnalog = bno.getAxis(BNO::eAxisAcc); 
  Serial.print("acc analog: (unit mg)       ");printAxisData(sAccAnalog);
  
  //-------------------------------------------------------
  uint32_t press = bmp.getPressure();
  *(float*)(message.Altitude) = bmp.calAltitude(SEA_LEVEL_PRESSURE, press);
  *(float*)(message.Acceleration) = bno.getAxis(BNO::eAxisAcc).y;

  ResponseStatus rs = e32ttl.sendFixedMessage(0,2,6,&message, sizeof(Message));
  Serial.println(rs.getResponseDescription());
  }
}
