#include <Arduino.h>
#include <Wire.h>

#define LOOP_TIME 4000  // loop every 4000us

unsigned long int current_time;
int gyro[3], acc[3];
int temp;
int count = 0;


void setup_gyro();
void read_gyro();

void setup()
{

  Wire.begin();
  Wire.setClock(400000);
  // wait for some time to get ready MPU6050
  delay(500);
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);   
  digitalWrite(LED_BUILTIN, LOW); 

  setup_gyro(); // setup gyro
  current_time = micros() + LOOP_TIME; // loop every 4000us
}

void loop()
{

  read_gyro();

  if (count < 2000)
  {
    if (!(count % 25))
    {
      if (count > 2000 - 25)
      {
        digitalWrite(LED_BUILTIN, LOW);
      }
      else
      {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // blink the led during calibration
      }
    }
    count++;
  }

  Serial.print(gyro[0]); Serial.print(", ");
  Serial.print(gyro[1]); Serial.print(", ");
  Serial.print(gyro[2]); Serial.print(", ");

  Serial.print(acc[0]); Serial.print(", ");
  Serial.print(acc[1]); Serial.print(", ");
  Serial.print(acc[2]); Serial.print(", ");

  Serial.print(((double) temp) / 340 + 36.53); //Serial.print("°C");
  Serial.println("");
  


  while (current_time > micros())
    ;
  current_time = micros() + LOOP_TIME;
}

void setup_gyro()
{
  // set up the MPU6050
  Wire.beginTransmission(0x68); // start communicating with mpu6050
  Wire.write(0x6B);             // we want to write in 0x6B register which is powermanagement register
  Wire.write(0x00);             // write 0x00 to that register
  Wire.endTransmission();       // end the transmission

  Wire.beginTransmission(0x68); // start communicating with mpu6050
  Wire.write(0x1B);             // we want to write in 0x1B which is gyro configaration register
  Wire.write(0x08);             // set the gyro for 500 deg pre sec full scale
  Wire.endTransmission();       // end the transmission

  Wire.beginTransmission(0x68); // start communicating with mpu6050
  Wire.write(0x1C);             // we want to write in 0x1C register which is Accelerometer Configuration register
  Wire.write(0x10);             // set the accelerometer with +-8g
  Wire.endTransmission();       // end the transmission

  Wire.beginTransmission(0x68); // start communicating with mpu6050
  Wire.write(0x1A);             // we want to write in 0x1A which is used to gyro and accelerometer synchronizing
  Wire.write(0x03);             // write 0x03 in that register
  Wire.endTransmission();       // end the transmission

  // random check registers wheather the values is written correctly or not
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.requestFrom(0x68, 1);
  while (Wire.available() < 1)
    ;
  byte data = Wire.read();
  Wire.endTransmission();
  if (data != 0x08)
  {
    digitalWrite(13, HIGH);
    while (true) // if not written this loop continues untill a next reset takes place
      delay(10);
  }
  // random check registers wheather the values is written correctly or not
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.requestFrom(0x68, 1);
  while (Wire.available() < 1)
    ;
  data = Wire.read();
  Wire.endTransmission();
  if (data != 0x10)
  {
    digitalWrite(13, HIGH);
    while (true) // if not written this loop continues untill a next reset takes place
      delay(10);
  }
}

void read_gyro()
{

  Wire.beginTransmission(0x68); // start communicating with MPU6050
  Wire.write(0x3B);             // start reading from 0x3B
  Wire.endTransmission();       // end the transmission
  Wire.requestFrom(0x68, 14);   // request 14 bytes
  while (Wire.available() < 14)
    ;                                       // wait until 14 bytes are received
  acc[0] = Wire.read() << 8 | Wire.read();  // read acc x data
  acc[1] = Wire.read() << 8 | Wire.read();  // read acc y data
  acc[2] = Wire.read() << 8 | Wire.read();  // read acc z data
  temp = Wire.read() << 8 | Wire.read();    // read temperature data [we don't use it here]
  gyro[0] = Wire.read() << 8 | Wire.read(); // read gyro x data
  gyro[1] = Wire.read() << 8 | Wire.read(); // read gyro y data
  gyro[2] = Wire.read() << 8 | Wire.read(); // read gyro z data
}

// void calculate_angle()
// {

//   /*
//          1
//       --------- = 0.0000611   ///// 65.5 is the value for the gyro for 500°/sec configure and rotate 1°/sec and 250 is refresh rate
//       65.5*250
//   */
//   gyro[0] *= 0.0000611; // we dont need the raw value again so we can change the original data
//   gyro[1] *= 0.0000611; // we dont need the raw value again so we can change the original data
//   gyro[2] *= 0.0000611; // we dont need the raw value again so we can change the original data

//   angular_rate[0] = angular_rate[0] * 0.7 + gyro[0] * 0.3;
//   angular_rate[1] = angular_rate[1] * 0.7 + gyro[1] * 0.3;
//   angular_rate[2] = angular_rate[2] * 0.7 + gyro[2] * 0.3;

//   gravity = sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]); // calculate the gravity
//   acc[0] = asin(acc[0] / gravity) * 57.295779;                         // get pitch angle
//   acc[1] = asin(acc[1] / gravity) * 57.295779;                         // get roll angle

//   /*
//          1
//       --------- = 0.0000611   ///// 65.5 is the value for the gyro for 500°/sec configure and rotate 1°/sec and 250 is refresh rate
//       65.5*250
//   */
//   angle[0] = 0.96 * (angle[0] + angular_rate[0]) + 0.04 * acc[0]; // take a larger portion of gyro data and smaller portion of accelerometer data to avoid drift
//   angle[1] = 0.96 * (angle[1] + angular_rate[1]) + 0.04 * acc[1]; // take a larger portion of gyro data and smaller portion of accelerometer data to avoid drift
//   angle[2] = angle[2] + angular_rate[2];                          // yaw angle gyro based only it may drift

//   /*    π
//      -------- = 0.0174532925      //arduino sin() takes radian // we convert rolto pitch and pitch to rol because MPU6050 may be yawed inclimbed
//        180
//   */
//   angle[0] -= angle[1] * sin(angular_rate[2] * 0.0174532925);
//   angle[1] += angle[0] * sin(angular_rate[2] * 0.0174532925);
// }
