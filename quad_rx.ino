//Sketch to receive the value of the joystick on the transmitter, and apply that value as a PWM to the ESCs
//Arduino pinouts
//07 - NRF24L01+ CE
//08 - NRF24L01+ CSN
//11 = NRF24L01+ MOSI
//12 - NRF24L01+ MISO
//13 - NRF24L01+ SCK

//Pins for connection to the PWM input of the individual ESCs

#include <SPI.h>
#include <RF24.h>
#include <Servo.h>

int packet[1];
int PWM_signal_recvd;
int PWM_signal_to_write;
RF24 radio(7, 8); //CE and CSN pins on the Arduino
Servo ESC;

const byte address[6] = "00001";

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.startListening();

  ESC.attach(9, 1000, 2000);
}

//void function to apply speed to motors

void loop() {
  if (radio.available()){
    radio.read(packet, sizeof(packet));
  }
  PWM_signal_recvd = packet[0] - 125;

  PWM_signal_to_write = map(PWM_signal_recvd, 0, 130, 0, 180);
  ESC.write(PWM_signal_to_write);

  Serial.println(PWM_signal_to_write);
}
