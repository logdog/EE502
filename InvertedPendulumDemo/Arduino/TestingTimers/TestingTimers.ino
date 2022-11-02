void setupTimer2() {
  TCCR2B =  0;    // disable timer2
  TCNT2 = 0x00;   // reset counter2 value
  OCR2A = 0x00;   // count up to this value, then reset back to zero
  OCR2B = 0xfe;   // creates 16us pulse. when TNT2>OCR2B, OC0B goes high

  // set timer2 into WGM mode 7 (Fast PWM)
  // enable OC0B output
  // run timer on clk/1024 prescaler (CS = 101)
  TCCR2A = _BV(COM2B0) | _BV(COM2B1) | _BV(WGM20) | _BV(WGM21);
  TCCR2B = _BV(WGM22) | _BV(CS22) | _BV(CS20);

  // set pin PD5 as output
  DDRD |= _BV(3); // OC2B -> PD3 -> Pin 3
}

void pulse2() {
  TCNT2 = OCR2B - 1;
}


void setupTimer0() {
  TCCR0B =  0;    // disable timer2
  TCNT0 = 0x00;   // reset counter2 value
  OCR0A = 0x00;   // count up to this value, then reset back to zero
  OCR0B = 0xfe;   // creates pulse. when TNT0>OCR0B, OC0B goes high

  // set timer0 into WGM mode 7 (Fast PWM)
  // enable OC0B output
  // run timer on clk/1024 prescaler (CS = 011)
  TCCR0A = _BV(COM0B0) | _BV(COM0B1) | _BV(WGM00) | _BV(WGM01);
  TCCR0B = _BV(WGM02) | _BV(CS01) | _BV(CS00);

  DDRD |= _BV(5); // OC0B -> PD5 -> Pin 5
}

void pulse0() {
  TCNT0 = OCR0B - 1;
}

void setup() {
  setupTimer0();
  setupTimer2();

  Serial.begin(115200);
}

void loop() {
  Serial.println("go");
  pulse0();
  pulse2();
}
