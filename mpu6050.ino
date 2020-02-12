#include <Wire.h>

//Declare variables
int IMU_Address = 104; //decimal conversion of B1101000
int cal_int = 0;

uint8_t X0, X1, X2, X3, X4, X5 = 0; 
byte gyro_mask = B00000000;
float gyro_pitch, gyro_roll, gyro_yaw = 0;
float gyro_pitch_cal, gyro_roll_cal, gyro_yaw_cal = 0;

#define WHO_AM_I 0x75
#define GYRO_CONFIG 0x1B
#define GYRO_XOUT_H 0x43

void setup() 
  {
    Serial.begin(9600);
    Wire.begin();
    Wire.beginTransmission(IMU_Address);
    Wire.write(GYRO_CONFIG);
    Wire.write(gyro_mask);
    Wire.endTransmission();
  
  Serial.println("Starting calibration");
  for(cal_int = 0; cal_int < 2000; cal_int++)
  {
    read_gyro();
    gyro_pitch_cal += gyro_pitch;
    gyro_roll_cal += gyro_roll;
    gyro_yaw_cal += gyro_yaw;
    delay(4);
  }
  Serial.println("Calibration complete");

  //Determine the average offset on each axis
  gyro_pitch_cal = gyro_pitch_cal/2000/131;
  gyro_roll_cal = gyro_roll_cal/2000/131;
  gyro_yaw_cal = gyro_yaw_cal/2000/131;

  Serial.print("Pitch Offset: ");
  Serial.print(gyro_pitch_cal);
  Serial.print("\t");

  Serial.print("Roll Offset: ");
  Serial.print(gyro_roll_cal);
  Serial.print("\t");

  Serial.print("Yaw Offset: ");
  Serial.println(gyro_yaw_cal);
  }

void loop() 
  {
    read_gyro();
    gyro_pitch = gyro_pitch/131 - gyro_pitch_cal;
    gyro_roll = gyro_roll/131 - gyro_roll_cal;
    gyro_yaw = gyro_yaw/131 - gyro_yaw_cal;

    Serial.print(gyro_pitch);
    Serial.print("\t");
    
    Serial.print(gyro_roll);
    Serial.print("\t");

    Serial.println(gyro_yaw);

    delay(100);
  }

void read_gyro()
  {
    Wire.beginTransmission(IMU_Address);
    Wire.write(GYRO_XOUT_H);
    Wire.endTransmission();
    Wire.requestFrom(IMU_Address, 6);
    
    if(Wire.available() <= 6)
      {
        X0 = Wire.read();
        X1 = Wire.read();

        X2 = Wire.read();
        X3 = Wire.read();

        X4 = Wire.read();
        X5 = Wire.read();
      }
      
    gyro_pitch = X0 << 8 | X1;
    gyro_roll = X2 << 8 | X3;
    gyro_yaw = X4 << 8 | X5;

  }
