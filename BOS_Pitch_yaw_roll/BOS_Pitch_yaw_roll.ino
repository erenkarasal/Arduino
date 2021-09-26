 #include "Wire.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "LoRa_E32.h"
#include "Adafruit_BMP280.h"
#define E32_TTL_1W
#define FREQUENCY_433

Adafruit_BMP280 bmp;

#include <SparkFunMPU9250-DMP.h>
MPU9250_DMP imu;
double roll , pitch, yaw;
long int pre_ts=0;

#define RELAY_1 2
#define RELAY_2 3
#define RELAY_3 4

int selState = LOW;

int status;

double baslangic;
#include <Servo.h>
Servo servo1;

static const int RXPin = 7, TXPin = 8;
static const uint32_t GPSBaud = 9600;


TinyGPSPlus gps;
SoftwareSerial portgps(RXPin, TXPin);
SoftwareSerial portlora(10, 11);
LoRa_E32 e32ttl(&portlora);


typedef  struct {
  int yirtifa;
  float yroll;
  float yaccel;
  float ylat;
  float ylng;

 
  
} Signal;

Signal data;


void setup()
{
  

  Serial.begin(9600);
  portgps.begin(GPSBaud);
  delay(500);
  e32ttl.begin();
  delay(500);
  Wire.begin();
  
  servo1.attach(5);
  delay(500);
  servo1.write(0);

  pinMode(RELAY_1, OUTPUT);
  pinMode(RELAY_2, OUTPUT);
  pinMode(RELAY_3, OUTPUT);
  digitalWrite(RELAY_1,HIGH);
  digitalWrite(RELAY_2,HIGH);
  digitalWrite(RELAY_3,HIGH);

if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      Serial.println("Unable to communicate with MPU-9250");
      Serial.println("Check connections, and try again.");
      Serial.println();
      delay(3000);
    }
  }
    if (!bmp.begin()) {

    while (1);
  }

//............................................................


   bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

//...........................................................
baslangic = bmp.readAltitude(1013.25);
}

int a=0;
void loop()
{
portgps.listen(); 
  if (gps.altitude.isValid()) {
 

//    Serial.print("GPS Roket lat: ");
//    Serial.println(gps.location.lat(),6);
    data.ylat = gps.location.lat();

//   Serial.print("GPS Roket lng: ");
//   Serial.println(gps.location.lng(),6);
    data.ylng = gps.location.lng();
    
  }
//  else{
//    Serial.println( "error");
  //}
  smartDelay(1000);

  //  if (millis() > 5000 && gps.charsProcessed() < 10)
   // Serial.println(F("No GPS data received: check wiring"));

  Serial.println();
  
  portlora.listen(); 

 // ResponseStatus rs = e32ttl.sendFixedMessage(0, 1, 23, &data, sizeof(Signal));
  //Serial.println(rs.getResponseDescription());
  servo1.detach();

//    Serial.print(" İrtifa = ");
//    Serial.print(bmp.readAltitude(1013.25));
    data.yirtifa = bmp.readAltitude(1013.25);



//    Serial.print(" Accel =");
//    Serial.print (imu.calcAccel(imu.ay)*10);
      data.yaccel = imu.calcAccel(imu.ay);


//    Serial.print(" Roll = ");
//    Serial.print(roll,2);
    data.yroll = roll,2;
     

if ( imu.dataReady() )
  {
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    pitch_yaw_roll(millis()-pre_ts);
    pre_ts=millis();
    double egim_z = roll;
    double yukseklik = bmp.readAltitude(1013.25);

    if(egim_z > 20){
  
                
       digitalWrite(RELAY_1,LOW);
       digitalWrite(RELAY_2,LOW);
       digitalWrite(RELAY_3,LOW);
       selState=HIGH;
       Serial.println("solenoid çalıştı");
       
        }
        if(selState==HIGH){
          for(int i=0 ; i<20;i++){
            Serial.println(i);
            delay(1000);
            if(i==15){
              servo1.write(30);
              
              Serial.println("servo çalıştı");
            }
          }
        }
        


    
//    double yukseklik_sifirlama =yukseklik-baslangic;
//    Serial.println(yukseklik_sifirlama);
//    //kalman_filtering(yukseklik_sifirlama);  
//    
//    if(yukseklik_sifirlama>1 && yukseklik_sifirlama<1.5){
//      a++;
//      delay(1000);
//      
//      Serial.println(a);
//      if(a==2){
//        Serial.println("servo çalıştı");
//        servo1.attach(5);
//        servo1.write(90);
//                    
//                     
//       
//      }
//    }
 
      

               
              
    
  }

//------------------------------------------------ Kurtarma başlangıcı              
               
               double egim_y =pitch; 
//             double egim_x =yaw;
                          
               double yukseklik = bmp.readAltitude(1013.25);
               double yukseklik_sifirlama =yukseklik-731.5;
                              
               double ivme = imu.calcAccel(imu.ay)*10;
                
                
               
//               if(yukseklik_sifirlama>=0)
//               {
//                  if(egim_z<-90 || ivme <= -9.804 || egim_y<-90)
//                 {
//                  Serial.println("Selenoidler çalıştı ");
//                  while(selState == LOW) {
//                    selState=HIGH;
//                    delay(5000);
//                    break;             
//                  }
//            
//            
//            
//                   digitalWrite(RELAY,selState);
//                   Serial.println("role high");
//                 
//                  
//                 }
//                 
//                 else {
//                  Serial.println("Selenoidler çalışmıyor");
//                  
//                   if(yukseklik_sifirlama <= 0.5)
//                   {
//                    
//                    while(true)
//                    {
//                        if(yukseklik_sifirlama <= 0.5 )//bu saıre hız eklenecek
//                       {
//                        //digitalWrite(powerControl, HIGH);
//                        Serial.println("Servo çalıştı");
//                       servo1.write(90);
//                         
//                       }
//                       else 
//                       {
//                        Serial.println("S.çalışmadı");
//                        
//                       }
//                    }
//                    
//                 }
//                 else {
//                  Serial.println("S.calışmıyor");
//                 }
//                 }
//                
//                 
//              }
//              else {
//                Serial.println("Y.y.değil");
//              }
                
             //...................................................    Kurtarma bitişi

}
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (portgps.available())
      gps.encode(portgps.read());
  } while (millis() - start < ms);
}
void pitch_yaw_roll(long int dt){
  
float accelX = imu.calcAccel(imu.ax);
  float accelY = imu.calcAccel(imu.ay);
  float accelZ = imu.calcAccel(imu.az);
  float gyroX = imu.calcGyro(imu.gx)/57.3;
  float gyroY = imu.calcGyro(imu.gy)/57.3;
  float gyroZ = imu.calcGyro(imu.gz)/57.3;
  float magX = imu.calcMag(imu.mx);
  float magY = imu.calcMag(imu.my);
  float magZ = imu.calcMag(imu.mz);



  //Euler angle from accel

 
   pitch = atan2 (accelY ,( sqrt ((accelX * accelX) + (accelZ * accelZ))));
   roll = atan2(-accelX ,( sqrt((accelY * accelY) + (accelZ * accelZ))));

   // yaw from mag

   float Yh = (magY * cos(roll)) - (magZ * sin(roll));
   float Xh = (magX * cos(pitch))+(magY * sin(roll)*sin(pitch)) + (magZ * cos(roll) * sin(pitch));

   yaw =  atan2(Yh, Xh);


    roll = roll*57.3;
    pitch = pitch*57.3;
    yaw = yaw*57.3;
//     Serial.print("pitch :"  + String( pitch) );
//   Serial.print("roll :" + String( roll));
//   Serial.println("yaw :" + String( yaw ));

}
double kalman_filtering(double input){
  double kalman_old;  
    double cov_old;
  
    double kalman_new = kalman_old; // eski değer alınır
    double cov_new = cov_old + 0.1; //yeni kovaryans değeri belirlenir. 
  
    double kalman_gain = cov_new / (cov_new + 10); //kalman kazancı hesaplanır. 
    double kalman_calculated = kalman_new + (kalman_gain * (input - kalman_new)); //kalman değeri hesaplanır
  
    cov_new = (1 - kalman_gain) * cov_old; //yeni kovaryans değeri hesaplanır
    cov_old = cov_new; //yeni değerler bir sonraki döngüde kullanılmak üzere kaydedilir
  
    kalman_old = kalman_calculated;
    Serial.print("kalman");
    Serial.println(input);
}
