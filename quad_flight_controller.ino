//Include the Wire.h library so I2C comms between the IMU and the Arduino are possible
#include <Wire.h>

int IMU_address = 104; //decimal conversion of B1101000
int cal_int = 0;
byte gyro_mask = 00000000; //Used to inhibit the self test, and set the Full Scale Range to 250 degrees per second
byte reset_mask = 10000000;
float gyro_pitch, gyro_roll, gyro_yaw = 0;
float gyro_pitch_cal, gyro_roll_cal, gyro_yaw_cal = 0;
uint8_t X0, X1, X2, X3, X4, X5 = 0;


#define GYRO_CONFIG 0x1B
#define PWR_MGMT_1 0x6B
#define GYRO_XOUT_H 0x43

///////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////
void setup() {
  Wire.begin();

  DDRB |= B00000000;
  DDRD |= B11110000;

  //Reset the IMU 
  Wire.beginTransmission(IMU_address);
  Wire.write(PWR_MGMT_1);
  Wire.write(reset_mask);
  Wire.endTransmission();

  delay(2000);

  //Set the full scale range for the gyro to be 250 degrees per second
  Wire.beginTransmission(IMU_address);
  Wire.write(GYRO_CONFIG);
  Wire.write(gyro_mask);
  Wire.endTransmission();

  //Calibrate gyros to determine the average bias per axis
  //Take 2000 samples
  for(cal_int = 0; cal_int < 2000; cal_int++)
  {
    read_gyro();
    gyro_pitch_cal += gyro_pitch;
    gyro_roll_cal += gyro_roll;
    gyro_yaw_cal += gyro_yaw;
    delay(4);
  }
  //Divide total number by 2000 to get per sample value, then divide by 131 (LSB) to get value in deg/sec
  gyro_pitch_cal = gyro_pitch_cal / 2000 / 131;
  gyro_roll_cal = gyro_roll_cal / 2000 / 131;
  gyro_yaw_cal = gyro_yaw_cal / 2000 / 131;

  //Enable Pin Change Interrupts for Digital input pins 8-11 (from receiver)
  //Enable Pin Change Interrupts for PCINT[0:7]
  PCICR |= (1 << PCIE0);
  //Enable PCI only for PCINT[0:3] which correspond to digital pins 8-11
  PCMSK0 |= (1 << PCINT0);
  PCMSK0 |= (1 << PCINT1);
  PCMSK0 |= (1 << PCINT2);
  PCMSK0 |= (1 << PCINT3);
  
}

void loop() {
  // put your main code here, to run repeatedly:

}

void read_gyro()
{
  Wire.beginTransmission(IMU_address);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission();

  Wire.requestFrom(IMU_address, 6);
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
