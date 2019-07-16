//Sketch to transmit the value of the joystick to control the rotor speed of the quad
//Arduino pinouts

//07 - NRF24L01+ CE
//08 - NRF24L01+ CSN
//11 - NRF24L01+ MOSI
//12 - NRF24L01+ MISO
//13 - NRF24L01+ SCK
//A0 - Joystick Y

#include <SPI.h>
#include <RF24.h>
int packet[1];
RF24 radio(7,8); // CE and CSN pins respectively on the Arduino

const byte address[6] = "00001";

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.setPayloadSize(32);
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.stopListening();

}

void loop() {
  int potValue0 = analogRead(A0);
  packet[0] = map(potValue0, 0, 1023, 0, 255);
  radio.write(&packet, sizeof(packet));
  Serial.print(packet[0]);
}
