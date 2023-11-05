//Include the Wire.h library so I2C comms between the IMU and the Arduino are possible
#include <Wire.h>

int IMU_address = 104; //decimal conversion of B1101000
int cal_int = 0;
int start = 0;
byte gyro_mask = 00000000; //Used to inhibit the self test, and set the Full Scale Range to 250 degrees per second
byte reset_mask = 10000000;
float gyro_pitch, gyro_roll, gyro_yaw = 0;
float gyro_pitch_raw, gyro_roll_raw, gyro_yaw_raw = 0;
float pitch_raw, roll_raw, yaw_raw = 0;
float gyro_pitch_cal, gyro_roll_cal, gyro_yaw_cal = 0;
uint8_t X0, X1, X2, X3, X4, X5 = 0;
int receiver_channel_1, receiver_channel_2, receiver_channel_3, receiver_channel_4 = 0;
int pid_roll_setpoint, pid_pitch_setpoint, pid_yaw_setpoint = 0; 
float pid_error_roll, pid_error_pitch, pid_error_yaw = 0; 
float pid_p_roll_output, pid_i_roll_output, pid_d_roll_output = 0;
float pid_p_pitch_output, pid_i_pitch_output, pid_d_pitch_output = 0;
float pid_p_yaw_output, pid_i_yaw_output, pid_d_yaw_output = 0;
float pid_i_roll_output_prev, pid_i_pitch_output_prev, pid_i_yaw_output_prev = 0;
int pid_error_roll_prev, pid_error_pitch_prev, pid_error_yaw_prev = 0;
float pid_roll_output, pid_pitch_output, pid_yaw_output = 0;
int throttle = 0;
int esc_fr, esc_fl, esc_rr, esc_rl = 0;
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4 = 0;
unsigned long timer1, timer2, timer3, timer4 = 0;
unsigned long loop_timer, esc_loop_timer = 0;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4 = 0;

///////////////////////////////////////////////////////////////////////////////////
//  Controller Gain Values
///////////////////////////////////////////////////////////////////////////////////
float pid_p_roll_gain = 1.0; //was 1.0
float pid_i_roll_gain = 0.12; //was 0.12
float pid_d_roll_gain = 15; //was 15; 
int max_roll_rate = 250; //Max roll rate

float pid_p_pitch_gain = pid_p_roll_gain;
float pid_i_pitch_gain = pid_i_roll_gain;
float pid_d_pitch_gain = pid_d_roll_gain;
int max_pitch_rate = 250; //Max pitch rate

float pid_p_yaw_gain = 3;
float pid_i_yaw_gain = 0.02;
float pid_d_yaw_gain = 0;
int max_yaw_rate = 250; //Max yaw rate

///////////////////////////////////////////////////////////////
// Gyro register addresses
///////////////////////////////////////////////////////////////
#define GYRO_CONFIG 0x1B
//Need ACC_CONFIG register
  //Write data to this register to set the max values
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
  Serial.begin(9600);


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
  Serial.println("Starting gyro calibration.");
  for(cal_int = 0; cal_int < 2000; cal_int++)
  {
    read_gyro();
    gyro_pitch_cal += gyro_pitch_raw;
    gyro_roll_cal += gyro_roll_raw;
    gyro_yaw_cal += gyro_yaw_raw;
  }
  
  //Divide total number by 2000 to get per sample value, then divide by 131 (LSB) to get value in deg/sec
  gyro_pitch_cal = gyro_pitch_cal / 2000 / LSB;
  gyro_roll_cal = gyro_roll_cal / 2000 / LSB;
  gyro_yaw_cal = gyro_yaw_cal / 2000 / LSB;

  Serial.println("Completed gyro calibration.");
  Serial.print("Pitch axis bias (deg/s): ");
  Serial.println(gyro_pitch_cal);
  Serial.print("Roll axis bias (deg/s): ");
  Serial.println(gyro_roll_cal);
  Serial.print("Yaw axis bias (deg/s): ");
  Serial.println(gyro_yaw_cal);

  //Enable Pin Change Interrupts for Digital input pins 8-11 (from receiver)
  //Enable Pin Change Interrupts for PCINT[0:7]
  PCICR |= (1 << PCIE0);
  //Enable PCI only for PCINT[0:3] which correspond to digital pins 8-11
  PCMSK0 |= (1 << PCINT0);
  PCMSK0 |= (1 << PCINT1);
  PCMSK0 |= (1 << PCINT2);
  PCMSK0 |= (1 << PCINT3);

  //Create loop that runs until the receiver input is valid and safe (for throttle and yaw controls only)
  while(receiver_channel_3 < 950)
  {
    delay(3);
  }

  //Assign current time to 'loop_timer'
  loop_timer = micros(); 

  Serial.println("Entering main loop.");
}

///////////////////////////////////////////////////////////////////////////////////////////
//Main loop for flight controller
///////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  // Read values from gyro. (Note: values need to be divided by LSB to get result in deg/sec.)
  // This function assigns values to 'gyro_pitch_raw', 'gyro_roll_raw', and 'gyro_yaw_raw'.
  read_gyro();

  //Get the raw (unfiltered) gyro readings. Subtract the bias off each axis. Result in deg/sec 
  pitch_raw = gyro_pitch_raw/LSB - gyro_pitch_cal;
  roll_raw = gyro_roll_raw/LSB - gyro_roll_cal;
  yaw_raw = gyro_yaw_raw/LSB - gyro_yaw_cal;
  
  //Filter the raw gyro data using an 80/20 filter
  // Explanation: The previous pitch value contributes 80% of the value of the next calculation.
  //              The raw value only contributes 20% of the next calculated value.
  //              Using this technique, spikes in the raw gyro values can only have 20% influence on the gyro readings used in this iteration of the loop.
  gyro_pitch = (gyro_pitch * 0.5) + (pitch_raw * 0.5);
  gyro_roll = (gyro_roll * 0.5) + (roll_raw * 0.5);
  gyro_yaw = (gyro_yaw * 0.5) + (yaw_raw * 0.5);

  //Establish conditions for starting the flight
  //This condition will be for the throttle to be low and yaw moved max to the left, then back to centre
  if(receiver_channel_3 < 1050 && receiver_channel_4 < 1050)
  {
    start = 1;
  }

  // Establish conditions for resetting the PID controller. 
  // This scope will only be hit once when the motors are started. 
  if(start == 1 && receiver_channel_3 < 1050 && receiver_channel_4 > 1450)
  {
    start = 2;
    //Code to reset the PID controllers
    pid_i_roll_output_prev = 0;
    pid_error_roll_prev = 0;
    pid_i_pitch_output_prev = 0;
    pid_error_pitch_prev = 0;
    pid_i_yaw_output_prev = 0;
    pid_error_yaw_prev = 0;    
  }

  //Establish conditions for stopping flight
  //This will be throttle low and yaw max to the right then back to centre
  if(start == 2 && receiver_channel_3 < 1050 && receiver_channel_4 > 1950)
  {
    start = 0;
  }

  //Establish PID setpoint for roll in degrees per second - channel 1 (roll)
  // Roll to the right is +ve
  pid_roll_setpoint = 0;
  if(receiver_channel_3 > 1050)
  {
    if(receiver_channel_1 > 1508)
    {
      pid_roll_setpoint = (receiver_channel_1 - 1508) / 2; 
    }
    else if(receiver_channel_1 < 1492)
    {
      pid_roll_setpoint = (receiver_channel_1 - 1492) / 2; 
    }
  }

  //Establish PID setpoint for pitch in degrees per second - channel 2 (pitch)
  //Pitch up is positive
  pid_pitch_setpoint = 0;
  if(receiver_channel_3 > 1050)
  {
    if(receiver_channel_2 > 1508)
    {
      pid_pitch_setpoint = (1508 - receiver_channel_2) / 2; 
    }
    else if(receiver_channel_2 < 1492)
    {
      pid_pitch_setpoint = (1492 - receiver_channel_2) / 2; 
    }
  }

  //Establish PID setpoint for yaw in degrees per second - channel 4
  // Clockwise yaw - -ve
  pid_yaw_setpoint = 0;
  if(receiver_channel_3 > 1050)
  {
    if(receiver_channel_4 > 1508)
    {
      pid_yaw_setpoint = (1508 - receiver_channel_4) / 2; 
    }
    else if(receiver_channel_4 < 1492)
    {
      pid_yaw_setpoint = (1492 - receiver_channel_4) / 2; 
    }
  }
    
  if(start == 2)
  {
    ///////////////////////////////////////////////////////////////////
    // Set the outputs to the ESCs based on throttle setting and PID outputs
    ///////////////////////////////////////////////////////////////////
    throttle = receiver_channel_3;
    
    ///////////////////////////////////////////////////////
    // Calculate the output of the PID Controller
    ///////////////////////////////////////////////////////
    calculate_pid(); 

    //Set max limit for throttle
    if(throttle > 1800)
    {
      throttle = 1800;
    }

    // Set the PWM pulse width for each ESC
    esc_fr = throttle + pid_roll_output - pid_pitch_output + pid_yaw_output;
    esc_fl = throttle - pid_roll_output - pid_pitch_output - pid_yaw_output;
    esc_rr = throttle + pid_roll_output + pid_pitch_output - pid_yaw_output;     
    esc_rl = throttle - pid_roll_output + pid_pitch_output + pid_yaw_output;

    if(esc_fr < 1080) esc_fr = 1080;
    if(esc_rr < 1080) esc_rr = 1080;;
    
    if(esc_fl < 1080) esc_fl = 1080;
    if(esc_rl < 1080) esc_rl = 1080;

    if(esc_fr > 2000) esc_fr = 2000;
    if(esc_rr > 2000) esc_rr = 2000;
    if(esc_fl > 2000) esc_fl = 2000;
    if(esc_rl > 2000) esc_rl = 2000;
  }
  else
  {
    esc_fr = 1000;
    esc_fl = 1000;
    esc_rr = 1000;
    esc_rl = 1000;
  }

//   Serial.print("Front right: ");
//   Serial.print(esc_fr);
//   Serial.print(", Front left: ");
//   Serial.print(esc_fl);
//   Serial.print(", Rear right: ");
//   Serial.print(esc_rr);
//   Serial.print(", Rear left: ");
//   Serial.println(esc_rl);

  //Need to write the values to the ESCs
  //The refresh rate is 250Hz. That means the esc's need their pulse every 4ms.
  while((micros() - loop_timer) < 4000)
  {
    delay(1);
    //Keep looping until 4ms is expired
  }
    
  loop_timer = micros(); //Set the loop_timer variable to the current time  

  // Set the time that the PWM signal for each channel should go low
  timer_channel_1 = loop_timer + esc_fr; //Front right ESC - Digital pin 4 
  timer_channel_2 = loop_timer + esc_rr; //Rear right ESC - Digital pin 5
  timer_channel_3 = loop_timer + esc_rl; //Rear left ESC - Digital pin 6
  timer_channel_4 = loop_timer + esc_fl; //Front left ESC - Digital pin 7

  //Set all PWM outputs to ESCs to high
  PORTD |= B11110000;

  while(PORTD >= 16)
  {
    esc_loop_timer = micros();
    if(timer_channel_1 <= esc_loop_timer)PORTD &= B11101111;
    if(timer_channel_2 <= esc_loop_timer)PORTD &= B11011111;
    if(timer_channel_3 <= esc_loop_timer)PORTD &= B10111111;
    if(timer_channel_4 <= esc_loop_timer)PORTD &= B01111111;
  }
} //End of main loop

//ISR that runs each time the input (on  any channel) from the receiver changes
ISR(PCINT0_vect){
  //check whether each pin is high or low, and if it's the same as the last sample (last_channel_x)
  //Channel 1
  if((PINB & B00000001) && (last_channel_1 == 0)){
    timer1 = micros();
    last_channel_1 = 1;
  }
  else if(!(PINB & B00000001) && (last_channel_1 == 1)){
    receiver_channel_1 = micros() - timer1;
    last_channel_1 = 0;
  }

  //Channel 2
  if((PINB & B00000010) && (last_channel_2 == 0)){
    timer2 = micros();
    last_channel_2 = 1;
  }
  else if(!(PINB & B00000010) && (last_channel_2 == 1)){
    receiver_channel_2 = micros() - timer2;
    last_channel_2 = 0;
  }

  //Channel 3
  if((PINB & B00000100) && (last_channel_3 == 0)){
    timer3 = micros();
    last_channel_3 = 1;
  }
  else if(!(PINB & B00000100) && (last_channel_3 == 1)){
    receiver_channel_3 = micros() - timer3;
    last_channel_3 = 0;
  }

  //Channel 4
  if((PINB & B00001000) && (last_channel_4 == 0)){
    timer4 = micros();
    last_channel_4 = 1;
  }
  else if(!(PINB & B00001000) && (last_channel_4 == 1)){
    receiver_channel_4 = micros() - timer4;
    last_channel_4 = 0;
  }
}

/////////////////////////////////////////////////////
// Function for reading values from gyro
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
      
    gyro_pitch_raw = X0 << 8 | X1;
    gyro_roll_raw = X2 << 8 | X3;
    gyro_yaw_raw = X4 << 8 | X5;
}


//////////////////////////////////////////////////////////
// Subroutine for determining PID controller outputs
//////////////////////////////////////////////////////////

void calculate_pid()
{
  //Code of calculating the PID output for Roll
  pid_error_roll = gyro_roll - pid_roll_setpoint;

  //Determine proportional output
  pid_p_roll_output = pid_error_roll * pid_p_roll_gain;

  //Determine integral output
  pid_i_roll_output = pid_i_roll_output_prev + (pid_error_roll * pid_i_roll_gain);

  //Determine differential output
  pid_d_roll_output = (pid_error_roll - pid_error_roll_prev) * pid_d_roll_gain;

  //Determine overall PID output
  pid_roll_output = pid_p_roll_output + pid_i_roll_output + pid_d_roll_output;

  if(pid_roll_output > max_roll_rate)
  {
    pid_roll_output = max_roll_rate;
  }
  else if(pid_roll_output < max_roll_rate * -1)
  {
    pid_roll_output = max_roll_rate * -1;
  }

  pid_i_roll_output_prev = pid_i_roll_output;

  pid_error_roll_prev = pid_error_roll;

  //Code of calculating the PID output for Pitch
  pid_error_pitch = gyro_pitch - pid_pitch_setpoint;

  pid_p_pitch_output = pid_error_pitch * pid_p_pitch_gain;

  pid_i_pitch_output = pid_i_pitch_output_prev + (pid_error_pitch * pid_i_pitch_gain);
//  Serial.print("Pitch I output: ");
//  Serial.println(pid_i_pitch_output);

  pid_d_pitch_output = (pid_error_pitch - pid_error_pitch_prev) * pid_d_pitch_gain;
 
  pid_pitch_output = pid_p_pitch_output + pid_i_pitch_output + pid_d_pitch_output;
//  Serial.print("Total Pitch output: ");
//  Serial.println(pid_pitch_output);

  if(pid_pitch_output> max_pitch_rate)
  {
    pid_pitch_output = max_pitch_rate;
  }
  else if(pid_pitch_output < max_pitch_rate * -1)
  {
    pid_pitch_output = max_pitch_rate * -1;
  }

  pid_i_pitch_output_prev = pid_i_pitch_output;

  pid_error_pitch_prev = pid_error_pitch;

  //Code of calculating the PID output for Yaw
  pid_error_yaw = gyro_yaw - pid_yaw_setpoint;

  pid_p_yaw_output = pid_error_yaw * pid_p_yaw_gain;

  pid_i_yaw_output = pid_i_yaw_output_prev + (pid_error_yaw * pid_i_yaw_gain);

  pid_d_yaw_output = (pid_error_yaw - pid_error_yaw_prev) * pid_d_yaw_gain;
 
  pid_yaw_output = pid_p_yaw_output + pid_i_yaw_output + pid_d_yaw_output;

  if(pid_yaw_output> max_yaw_rate)
  {
    pid_yaw_output = max_yaw_rate;
  }
  else if(pid_yaw_output < max_yaw_rate * -1)
  {
    pid_yaw_output = max_yaw_rate * -1;
  }

  pid_i_yaw_output_prev = pid_i_yaw_output;

  pid_error_yaw_prev = pid_error_yaw;
}
