/*             PINOUT
* Encoder Pin A ---> Pin 2 = Port D2
* Encoder Pin B ---> Pin 3 = Port D3
*/


/* Encoder Setup */
volatile int16_t encoderPos;

void setupEncoder() {
  // enable pull-up resistors
  PORTD |= _BV(PORTD3) | _BV(PORTD2);

  // Any logical change on INT0 (pin2) generates an interrupt request
  // Any logical change on INT1 (pin3) generates an interrupt request
  EICRA |= _BV(ISC10) | _BV(ISC00);

  // enable pins 2 and 3for interrupts
  EIMSK |=  _BV(INT1) | _BV(INT0);

  // without this code, the ISR will run once and increment encoderPos
  // so our encoderPos would be set at 1 instead of 0
  delay(1);
  encoderPos = 0;
}

#define AB_MASK	(_BV(2) | _BV(3))
#define A1_B1 	(_BV(2) | _BV(3))
#define A1_B0 	(_BV(2))
#define A0_B1 	(_BV(3))
#define A0_B0 	0x00

// triggers on rising and falling edge of D2
ISR(INT0_vect) {
	switch(PIND & AB_MASK) {
		case A1_B0: 
		case A0_B1: encoderPos++; break;

		case A1_B1:
		case A0_B0: encoderPos--; break;

		default: break;
	}
}

// triggers on rising and falling edge of D3
ISR(INT1_vect) {
	switch(PIND & AB_MASK) {
		case A1_B0: 
		case A0_B1: encoderPos--; break;

		case A1_B1:
		case A0_B0: encoderPos++; break;

		default: break;
	}
}

void setup() {
	setupEncoder();

	// initialize COM port
	Serial.begin(115200);
}

void loop() {
	// put your main code here, to run repeatedly:
	delay(1000);
	Serial.print("Encoder Pos: "); Serial.println(encoderPos);
}
