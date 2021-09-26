#include "MPU9250.h"
#include <Adafruit_BMP280.h>
#include <Wire.h>

Adafruit_BMP280 bmp;
MPU9250 mpu;
int status;

void setup() {
    Serial.begin(9600);
    Wire.begin();
    delay(2000);
    
    //pinMode(8, OUTPUT);//led pinmode on 
    
     
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

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void loop() {
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 25) {
            print_roll_pitch_yaw();
            print_accel();
            print_rakim();
            prev_ms = millis();
            
           
        }
    }
  
    
   
    // Serial.println(mpu.getRoll(),  2);
    //double rakim = bmp.readAltitude(1013.25);
   // double istenen_rakim=rakim-794;

    
//    if(istenen_rakim> 50){
//      digitalWrite(8, HIGH);
//      
//    }
//     else {
//      digitalWrite(8, LOW);
//    }
//    
//    
//     
//    double led_yanma_egim = mpu.getRoll();
//    
//    
//    float led_yanma_ivme = mpu.getAccY();
//  if(led_yanma_egim<-90){
//        digitalWrite(8, HIGH);
//       
//    }
//    else if (led_yanma_ivme<-9.804){
//      
//    }
//   else {
//      digitalWrite(8, LOW);
//     
//   }
//
//    
    
}

void print_roll_pitch_yaw() {
    //Serial.print("Yaw, Pitch, Roll: ");
    Serial.print("Yaw:");
    Serial.print(", ");
    Serial.print(mpu.getYaw(), 2);
    Serial.print("Pitch:");
    Serial.print(", ");
    Serial.print(mpu.getPitch(), 2);
    Serial.print("Roll:");
    Serial.print(", ");
    Serial.println(mpu.getRoll(), 2);
    
    
}
void print_accel(){
  Serial.print("Acceleration");
  
  //Serial.print(mpu.getAccX()*10,6);//sensörün üzerindeki x yönüne doğru hareket ettirince değişiyor
  //Serial.print(", ");
  Serial.println(mpu.getAccY()*10,6);//Dik yukarı kaldırınca (sensördekiş y hareketi )
  //Serial.print(", ");
  //Serial.println(mpu.getAccZ()*10,6);//rol hareketini yapınca değişiyor (sensördeki z hareketi)

}
void print_rakim(){
 float rakim = bmp.readAltitude(1013.25);
 Serial.print("Basınç : ");
 Serial.print(bmp.readPressure()/100);
 Serial.print("Rakım : ");
 Serial.println(rakim);
  
}
