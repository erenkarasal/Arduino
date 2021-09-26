
#include <Servo.h>
Servo servo1;
double roll , pitch, yaw;
#include <SparkFunMPU9250-DMP.h>

MPU9250_DMP imu;
long int pre_ts=0;

void setup() {
 Serial.begin(9600);
  servo1.attach(5);  // attaches the servo on pin 9 to the servo object
      servo1.write(0);              // tell servo to go to position in variable 'pos'
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

}

void loop() {
  if ( imu.dataReady() )
  {
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    pitch_yaw_roll(millis()-pre_ts);
    pre_ts=millis();
    
  }

  Serial.print(" datas = ");
    Serial.print(roll,2);

  double egim_z = roll;
               double egim_y =pitch; 
//             double egim_x =yaw;
if(egim_z > 20){
                servo1.write(90);
                Serial.println("döndü");
               }


               
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
     Serial.print("pitch :"  + String( pitch) );
   Serial.print("roll :" + String( roll));
   Serial.println("yaw :" + String( yaw ));

}
