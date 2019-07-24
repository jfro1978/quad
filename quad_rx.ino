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

int packet[1];
RF24 radio(7, 8); //CE and CSN pins on the Arduino

const byte address[6] = "00001";

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.startListening();

}

//void function to apply speed to motors

void loop() {
  if (radio.available()){
    radio.read(packet, sizeof(packet));
  }
  Serial.print(packet[0]);
}
