#include <Arduino.h>
#include "MPU6050.h"
#include "helper_3dmath.h"
#include "uav.h"

uint16_t deltaT = 5000;

MPU6050 mpu;
UAV uav;

unsigned long int currentTime, cc;
float gyroData[3], gyroRate[3] = {0, 0, 0};
float accl_angle[3] = {0, 0, 0};

VectorFloat accelData;

uint8_t printCounter = 0;

void setup()
{
  Serial.begin(115200);
  Wire.begin(); // Initiate the Wire library and join the I2C bus as a master
  /*set clockspeed 400KHz for fast access default is 100 KHz
     if you want to use default one just comment out line 27
  */
  Wire.setClock(400000);
  // wait for some time to get ready MPU6050
  delay(500);

  pinMode(LED_BUILTIN, OUTPUT);   // set led builtin output
  digitalWrite(LED_BUILTIN, LOW); // turnoff leder++;
  
  // initilize mpu6050
  mpu.initialize();
  // delay(100); // wait for sometime
  delay(1000);

  mpu.read(); // to clear all data
  currentTime = micros() + deltaT;

  mpu.getAccelVector(&accelData);
  accelData.normalize();
  accl_angle[0] = asin(accelData.x);
  accl_angle[1] = asin(accelData.y);
}

void loop()
{
  while (currentTime > micros())
    ;
  currentTime = micros() + deltaT;
  mpu.read(); // 620us
  mpu.getGyroData(gyroData, deltaT);

  // 80us
  gyroRate[0] = gyroRate[0] * 0.7 + gyroData[0] * 0.3;
  gyroRate[1] = gyroRate[1] * 0.7 + gyroData[1] * 0.3;
  gyroRate[2] = gyroRate[2] * 0.7 + gyroData[2] * 0.3;


  mpu.getAccelVector(&accelData); // 40us
  accelData.normalize();  // 140us

  // 240us
  accl_angle[0] = (accl_angle[0] + gyroRate[1]) * 0.95 + (asin(accelData.x)) * 0.05;
  accl_angle[1] = (accl_angle[1] + gyroRate[0]) * 0.95 + (asin(accelData.y)) * 0.05;
  accl_angle[2] = (accl_angle[2] + gyroRate[2]);

  /* we convert rolto pitch and pitch to rol because MPU6050 may be yawed inclimbed
  */
  // 140us
  accl_angle[0] -= accl_angle[1] * sin(gyroRate[2]);
  accl_angle[1] += accl_angle[0] * sin(gyroRate[2]);



  printCounter++;
  if (printCounter % 5 == 0)
  {
    Serial.print(accl_angle[0]*180/PI); Serial.print("\t");
    Serial.print(accl_angle[1]*180/PI); Serial.print("\t");
    // Serial.print(angle[2]); Serial.print("\t");
    // Serial.println(printCounter);
    Serial.println(accl_angle[2]*180/PI);
    printCounter = 0;
  }
}