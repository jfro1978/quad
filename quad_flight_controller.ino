//Include the Wire.h library so I2C comms between the IMU and the Arduino are possible
#include <Wire.h>

int IMU_address = 104; //decimal conversion of B1101000
int cal_int = 0;
int start = 0;
byte gyro_mask = 00000000; //Used to inhibit the self test, and set the Full Scale Range to 250 degrees per second
byte reset_mask = 10000000;
float gyro_pitch, gyro_roll, gyro_yaw = 0;
float gyro_pitch_cal, gyro_roll_cal, gyro_yaw_cal = 0;
uint8_t X0, X1, X2, X3, X4, X5 = 0;
int receiver_channel_1, receiver_channel_2, receiver_channel_3, receiver_channel_4 = 0;
int pid_roll_setpoint, pid_pitch_setpoint, pid_yaw_setpoint = 0; 
int pid_error_roll, pid_error_pitch, pid_error_yaw = 0; 
int pid_p_roll_output, pid_i_roll_output, pid_d_roll_output = 0;
int pid_p_pitch_output, pid_i_pitch_output, pid_d_pitch_output = 0;
int pid_p_yaw_output, pid_i_yaw_output, pid_d_yaw_output = 0;
float pid_i_roll_output_prev, pid_i_pitch_output_prev, pid_i_yaw_output_prev = 0;
int pid_error_roll_prev, pid_error_pitch_prev, pid_error_yaw_prev = 0;
float pid_roll_output, pid_pitch_output, pid_yaw_output = 0;

///////////////////////////////////////////////////////////////////////////////////
//  Controller Gain Values
///////////////////////////////////////////////////////////////////////////////////
float pid_p_roll_gain = 1;
float pid_i_roll_gain = 1;
float pid_d_roll_gain = 1;
int max_roll_rate = 400; //Max roll rate

float pid_p_pitch_gain = 1;
float pid_i_pitch_gain = 1;
float pid_d_pitch_gain = 1;
int max_pitch_rate = 400; //Max pitch rate

float pid_p_yaw_gain = 1;
float pid_i_yaw_gain = 1;
float pid_d_yaw_gain = 1;
int max_yaw_rate = 400; //Max yaw rate

///////////////////////////////////////////////////////////////
// Gyro register addresses
///////////////////////////////////////////////////////////////
#define GYRO_CONFIG 0x1B
#define PWR_MGMT_1 0x6B
#define GYRO_XOUT_H 0x43
#define LSB 131

//Channel 1 - roll
//Channel 2 - pitch
//Channel 3 - throttle
//Channel 4 - yaw

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
  gyro_pitch_cal = gyro_pitch_cal / 2000 / LSB;
  gyro_roll_cal = gyro_roll_cal / 2000 / LSB;
  gyro_yaw_cal = gyro_yaw_cal / 2000 / LSB;

  //Enable Pin Change Interrupts for Digital input pins 8-11 (from receiver)
  //Enable Pin Change Interrupts for PCINT[0:7]
  PCICR |= (1 << PCIE0);
  //Enable PCI only for PCINT[0:3] which correspond to digital pins 8-11
  PCMSK0 |= (1 << PCINT0);
  PCMSK0 |= (1 << PCINT1);
  PCMSK0 |= (1 << PCINT2);
  PCMSK0 |= (1 << PCINT3);

  //Create loop that runs until the receiver input is valid and safe (for throttle and yaw controls only)
  while(receiver_channel_3 < 990 || receiver_channel_3 > 1020 || receiver_channel_4 < 1450 || receiver_channel_4 > 1550)
  {
    //The ESCs will beep if no input is received, so will give them a 1000us pulse which is zero input
    PORTD = PORTD | B11110000;
    delayMicroseconds(1000);
    PORTD = PORTD & B00001111;

    delay(3);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
//Main loop for flight controller
///////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  // Read values from gyro. Remember values need to be divided by LSB to get value in deg/sec
  read_gyro();
  //Subtract gyro bias off each axis. Result in deg/sec
  gyro_pitch = gyro_pitch/LSB - gyro_pitch_cal;
  gyro_roll = gyro_roll/LSB - gyro_roll_cal;
  gyro_yaw = gyro_yaw/LSB - gyro_yaw_cal;

  //Establish conditions for starting the flight
  //This condition will be for the throttle to be low and yaw moved max to the left, then back to centre
  if(receiver_channel_3 < 1050 && receiver_channel_4 < 1050)
  {
    start = 1;
  }
  if(start == 1 && receiver_channel_3 < 1050 && receiver_channel_4 > 1450)
  {
    start = 2;
    //Need code here to reset the PID controllers
  }

  //Establish conditions for stopping flight
  //This will be throttle low and yaw max to the right then back to centre
  if(start == 2 && receiver_channel_3 < 1050 && receiver_channel_3 > 1950)
  {
    start = 0;
  }

  //Establish PID setpoint for roll in degrees per second - channel 1
  pid_roll_setpoint = 0;
  //Include the next line once up n flying. It will prevent pitch roll n yaw when throttle is low
  //if(receiver_channel_3 > 1050){}
  if(receiver_channel_1 > 1508)
  {
    pid_roll_setpoint = (receiver_channel_1 - 1508) / 2; 
  }
  else if(receiver_channel_1 < 1492)
  {
    pid_roll_setpoint = (receiver_channel_1 - 1492) / 2; 
  }


  //Establish PID setpoint for pitch in degrees per second - channel 2
  pid_pitch_setpoint = 0;
  //Include the next line once up n flying. It will prevent pitch roll n yaw when throttle is low
  //if(receiver_channel_3 > 1050){}
  if(receiver_channel_2 > 1508)
  {
    pid_pitch_setpoint = (receiver_channel_2 - 1508) / 2; 
  }
  else if(receiver_channel_2 < 1492)
  {
    pid_pitch_setpoint = (receiver_channel_2 - 1492) / 2; 
  }


  //Establish PID setpoint for yaw in degrees per second - channel 4
  pid_yaw_setpoint = 0;
  //Include the next line once up n flying. It will prevent pitch roll n yaw when throttle is low
  //if(receiver_channel_3 > 1050){}
  if(receiver_channel_4 > 1508)
  {
    pid_yaw_setpoint = (receiver_channel_4 - 1508) / 2; 
  }
  else if(receiver_channel_4 < 1492)
  {
    pid_yaw_setpoint = (receiver_channel_4 - 1492) / 2; 
  }  

///////////////////////////////////////////////////////
// Calculate the output of the PID Controller
///////////////////////////////////////////////////////
calculate_pid();


}


/////////////////////////////////////////////////////
//Subroutine for reading values from gyro
/////////////////////////////////////////////////////
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


//////////////////////////////////////////////////////////
// Subroutine for determining PID controller outputs
//////////////////////////////////////////////////////////

void calculate_pid()
{
  //Code of calculating the PID output for Roll
  pid_error_roll = gyro_roll - pid_roll_setpoint;

  pid_p_roll_output = pid_error_roll * pid_p_roll_gain;

  pid_i_roll_output = pid_i_roll_output_prev + (pid_error_roll * pid_i_roll_gain);

  pid_d_roll_output = (pid_error_roll - pid_error_roll_prev) * pid_d_roll_gain;
 
  pid_roll_output = pid_p_roll_output + pid_i_roll_output + pid_d_roll_output;

  //Code of calculating the PID output for Pitch
  pid_error_pitch = gyro_pitch - pid_pitch_setpoint;

  pid_p_pitch_output = pid_error_pitch * pid_p_pitch_gain;

  pid_i_pitch_output = pid_i_pitch_output_prev + (pid_error_pitch * pid_i_pitch_gain);

  pid_d_pitch_output = (pid_error_pitch - pid_error_pitch_prev) * pid_d_pitch_gain;
 
  pid_pitch_output = pid_p_pitch_output + pid_i_pitch_output + pid_d_pitch_output;

  //Code of calculating the PID output for Yaw
  pid_error_yaw = gyro_yaw - pid_yaw_setpoint;

  pid_p_yaw_output = pid_error_yaw * pid_p_yaw_gain;

  pid_i_yaw_output = pid_i_yaw_output_prev + (pid_error_yaw * pid_i_yaw_gain);

  pid_d_yaw_output = (pid_error_yaw - pid_error_yaw_prev) * pid_d_yaw_gain;
 
  pid_yaw_output = pid_p_yaw_output + pid_i_yaw_output + pid_d_yaw_output;
}
