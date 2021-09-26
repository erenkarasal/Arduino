#include "DFRobot_BNO055.h"
#include "Wire.h"
#include "DFRobot_BMP280.h"

typedef DFRobot_BNO055_IIC    BNO;    // ******** use abbreviations instead of full names ********
typedef DFRobot_BMP280_IIC    BMP;
BNO   bno(&Wire, 0x28);    // input TwoWire interface and IIC address
BMP   bmp(&Wire, BMP::eSdo_low);
#define SEA_LEVEL_PRESSURE    1013.25f 
// show last sensor operate status

void printLastOperateStatus(BNO::eStatus_t eStatus)
{
  switch(eStatus) {
  case BNO::eStatusOK:   Serial.println("everything ok"); break;
  case BNO::eStatusErr:  Serial.println("unknow error"); break;
  case BNO::eStatusErrDeviceNotDetect:   Serial.println("device not detected"); break;
  case BNO::eStatusErrDeviceReadyTimeOut:    Serial.println("device ready time out"); break;
  case BNO::eStatusErrDeviceStatus:    Serial.println("device internal status error"); break;
  default: Serial.println("unknow status"); break;
  }
}


void setup()
{
  Serial.begin(115200);
  bno.reset();
  while(bno.begin() != BNO::eStatusOK) {
    Serial.println("bno begin faild");
    printLastOperateStatus(bno.lastOperateStatus);
    delay(2000);
  }
  Serial.println("bno begin success");
  
   bmp.reset();
  Serial.println("bmp read data test");
  while(bmp.begin() != BMP::eStatusOK) {
    Serial.println("bmp begin faild");
    
  }
  Serial.println("bmp begin success");
}

//-------------------------------------------------------------------------------
#define printAxisData(sAxis) \
  Serial.print(" x: "); \
  Serial.print(sAxis.x); \
  Serial.print(" y: "); \
  Serial.print(sAxis.y); \
  Serial.print(" z: "); \
  Serial.println(sAxis.z)


void loop()
{
//  BNO::sAxisAnalog_t   sAccAnalog, sMagAnalog, sGyrAnalog, sLiaAnalog, sGrvAnalog;
//  BNO::sEulAnalog_t    sEulAnalog;
//  BNO::sQuaAnalog_t    sQuaAnalog;
//  
//  sAccAnalog = bno.getAxis(BNO::eAxisAcc);    // read acceleration
//  sMagAnalog = bno.getAxis(BNO::eAxisMag);    // read geomagnetic
//  sGyrAnalog = bno.getAxis(BNO::eAxisGyr);    // read gyroscope
//  sLiaAnalog = bno.getAxis(BNO::eAxisLia);    // read linear acceleration
//  sGrvAnalog = bno.getAxis(BNO::eAxisGrv);    // read gravity vector
//  sEulAnalog = bno.getEul();                  // read euler angle
//  sQuaAnalog = bno.getQua();                  // read quaternion
//  
//  Serial.println();
//  Serial.println("======== analog data print start ========");
//  Serial.print("acc analog: (unit mg)       "); printAxisData(sAccAnalog);
//  Serial.print("mag analog: (unit ut)       "); printAxisData(sMagAnalog);
//  Serial.print("gyr analog: (unit dps)      "); printAxisData(sGyrAnalog);
//  Serial.print("lia analog: (unit mg)       "); printAxisData(sLiaAnalog);
//  Serial.print("grv analog: (unit mg)       "); printAxisData(sGrvAnalog);
//  Serial.print("eul analog: (unit degree)   "); Serial.print(" head: "); Serial.print(sEulAnalog.head); Serial.print(" roll: "); Serial.print(sEulAnalog.roll);  Serial.print(" pitch: "); Serial.println(sEulAnalog.pitch);
//  Serial.print("qua analog: (no unit)       "); Serial.print(" w: "); Serial.print(sQuaAnalog.w); printAxisData(sQuaAnalog);
//  Serial.println("========  analog data print end  ========");
  print_altitude();
  
}


void print_altitude(){
  //float   temp = bmp.getTemperature();
  uint32_t    press = bmp.getPressure();
  float   alti = bmp.calAltitude(SEA_LEVEL_PRESSURE, press);

  //Serial.println();
  //Serial.println("======== start print ========");
  //Serial.print("temperature (unit Celsius): "); Serial.println(temp);
  //Serial.print("pressure (unit pa):         "); Serial.println(press);
  //Serial.print("pressure (unit pascal):      "); 
  Serial.print("Basınç: ");
  Serial.print(press/100);
  Serial.print(" Pa - ");
  //Serial.print("altitude (unit meter):      "); 
  Serial.print("Yükseklik:  ");
  Serial.print(alti);
  Serial.println(" m");
  //Serial.println("========  end print  ========");
  delay(1000);

}
