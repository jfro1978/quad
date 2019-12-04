//Declaration of variables
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
int pulse_channel_1, pulse_channel_2, pulse_channel_3, pulse_channel_4;
unsigned long timer1, timer2, timer3, timer4;

void setup() {
  //Setup the receiver interrupts (Port B0-3) as inputs (0 - input ; 1 - output)
  //The Arduino defaults to inputs, so this isn't required.
  //DDRB |= B00000000;
  //It could alternatively be done this way:
  /*
  DDRB |= (1 << PORTB0);
  DDRB |= (1 << PORTB1);
  DDRB |= (1 << PORTB2);
  DDRB |= (1 << PORTB3);
  */

  //Setup these inputs as PCIs
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0);
  PCMSK0 |= (1 << PCINT1);
  PCMSK0 |= (1 << PCINT2);
  PCMSK0 |= (1 << PCINT3);

  Serial.begin(9600);
}

void loop() {
  print_outputs();
  delay(200);
}

//ISR
ISR(PCINT0_vect){
  //check whether each pin is high or low, and if it's the same as the last sample (last_channel_x)
  //Channel 1
  if((PINB & B00000001) && (last_channel_1 == 0)){
    timer1 = micros();
    last_channel_1 = 1;
  }
  else if(!(PINB & B00000001) && (last_channel_1 == 1)){
    pulse_channel_1 = micros() - timer1;
    last_channel_1 = 0;
  }

  //Channel 2
  if((PINB & B00000010) && (last_channel_2 == 0)){
    timer2 = micros();
    last_channel_2 = 1;
  }
  else if(!(PINB & B00000010) && (last_channel_2 == 1)){
    pulse_channel_2 = micros() - timer2;
    last_channel_2 = 0;
  }

  //Channel 3
  if((PINB & B00000100) && (last_channel_3 == 0)){
    timer3 = micros();
    last_channel_3 = 1;
  }
  else if(!(PINB & B00000100) && (last_channel_3 == 1)){
    pulse_channel_3 = micros() - timer3;
    last_channel_3 = 0;
  }

  //Channel 4
  if((PINB & B00001000) && (last_channel_4 == 0)){
    timer4 = micros();
    last_channel_4 = 1;
  }
  else if(!(PINB & B00001000) && (last_channel_4 == 1)){
    pulse_channel_4 = micros() - timer4;
    last_channel_4 = 0;
  }

//End of ISR
}

void print_outputs(){
  Serial.print("Channel 1: ");
  Serial.print(pulse_channel_1);
  Serial.print("\t");

  Serial.print("Channel 2: ");
  Serial.print(pulse_channel_2);
  Serial.print("\t");

  Serial.print("Channel 3: ");
  Serial.print(pulse_channel_3);
  Serial.print("\t");

  Serial.print("Channel 4: ");
  Serial.println(pulse_channel_4);
}
