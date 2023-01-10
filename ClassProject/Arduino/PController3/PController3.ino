/*             PINOUT
 * Encoder Pin A ---> Pin 2  = Port D2
 * Encoder Pin B ---> Pin 6  = Port D6
 *
 * Motor Step    ---> Pin 3  = Port D3
 * Motor Dir     ---> Pin 4  = Port D4
 * Motor Enable  ---> Pin 5  = Port D5
 * 
 * Pot           ---> PinA0
 */

#define ENCODER_PIN_A 2
#define ENCODER_PIN_B 6

#define STEPPER_STEP_PIN 3
#define STEPPER_DIR_PIN 4
#define STEPPER_EN_PIN 5

#define DRIVE_RADIUS 0.01287 // should be between 12.5 and 13 mm. Depends on thickness of belt
#define STEPPER_PPR 800.0   // 200 * 8 = 1600 pulses per revolution
#define PULSE_WIDTH 16       // 16 us pulse width

/* Encoder Setup */
volatile int16_t encoderPos;

void setupEncoder()
{
  // enable pull-up resistors
  PORTD |= _BV(PORTD6) | _BV(PORTD2);

  // Any logical change on INT0 (pin2) generates an interrupt request
  EICRA |= _BV(ISC00);

  // enable pins 2  for interrupts
  EIMSK |= _BV(INT0);

  // without this code, the ISR will run once and increment encoderPos
  // so our encoderPos would be set at 1 instead of 0
  delay(1);

  // now, the pendulum is not directly hanging down
  // need to offset so encoderPos=600 means up
  encoderPos = 0;
}

double encoderRad() {
    return (2*PI/1200.0)*encoderPos;
}

#define AB_MASK (_BV(2) | _BV(6))
#define A1_B1 (_BV(2) | _BV(6))
#define A1_B0 (_BV(2))
#define A0_B1 (_BV(6))
#define A0_B0 0x00

// triggers on rising and falling edge of D2
// encoder has 1200 targetPositions using this method
ISR(INT0_vect)
{
  switch (PIND & AB_MASK)
  {
  case A1_B0:
  case A0_B1:
    encoderPos++;
    break;

  case A1_B1:
  case A0_B0:
    encoderPos--;
    break;

  default:
    break;
  }
}

// setup interrupt (every 50 microseconds)
void setupTimer1()
{
  cli(); // 16-bit register writes require interrupts to be disabled
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  // (16MHz / 2 * prescaler) = 1000kHz
  // 1000kHz / (1+OCR1A) = ?? kHz
  OCR1A = 99;

  // CTC mode, prescaler=8
  TCCR1B |= _BV(WGM12) | _BV(CS11);

  // enable output compare interrupt
  TIMSK1 |= _BV(OCIE1A);
  sei();
}

void setupTimer2()
{
  /* setup the timer2 */
  TCCR2B = 0;   // disable timer2
  TCNT2 = 0x00; // reset counter2 value
  OCR2A = 0x00; // count up to this value, then reset back to zero
  OCR2B = 0xfe; // creates 16us pulse. when TNT2>OCR2B, OC0B goes high

  // set timer2 into WGM mode 7 (Fast PWM)
  // enable OC0B output
  // run timer on clk/1024 prescaler (CS = 101)
  TCCR2A = _BV(COM2B0) | _BV(COM2B1) | _BV(WGM20) | _BV(WGM21);
  TCCR2B = _BV(WGM22) | _BV(CS22) | _BV(CS20);

  // set pin PD3 as output (step)
  DDRD |= _BV(3); // OC2B -> PD3 -> Pin 3
}


// cause the motor to step by sending a 16us pulse
void step()
{
  TCNT2 = OCR2B - 1;
}

// set the motor's direction
void updateDirection(uint8_t direction)
{
  digitalWrite(STEPPER_DIR_PIN, direction);
}

void setup()
{
    pinMode(STEPPER_DIR_PIN, OUTPUT);
    pinMode(STEPPER_EN_PIN, OUTPUT);
    pinMode(A0, INPUT);
    pinMode(7,OUTPUT);

    // turn motor off
    digitalWrite(STEPPER_EN_PIN, HIGH);

    // configure encoder
    setupEncoder();
    setupTimer2();

    Serial.begin(115200);

    // enable the main loop
    setupTimer1();
}

enum State {
    STOPPED,
    STARTING,
    RUNNING
};

unsigned long now = 0;
unsigned long lastTime;

double acceleration = 0.0;
double velocity = 0.0;
double targetPosition = 0.0;
double cartPosition = 0.0;
double Ts = 50e-6;

double theta;
uint32_t counter = 10000;
double K=80;

State state = STOPPED;

void updateStepperMotor() {
    if (velocity > 0 && targetPosition > cartPosition) {
        updateDirection(1);
        step();
        cartPosition += (2*PI/STEPPER_PPR)*DRIVE_RADIUS;
    }
    else if (velocity < 0 && targetPosition < cartPosition) {
        updateDirection(0);
        step();
        cartPosition -= (2*PI/STEPPER_PPR)*DRIVE_RADIUS;
    }
}

bool light;
// runs every Ts (0.000_050) seconds
ISR(TIMER1_COMPA_vect)
{

    digitalWrite(7, light);
    light = !light;
    // get angle from the vertical
    theta = encoderRad() - PI;


    //  =========== State Machine =============== //
    // STOPPED -> Turn off the motor
    if (state == STOPPED) {

        // turn off the motor
        digitalWrite(STEPPER_EN_PIN, HIGH); 

        // reset the state vector
        acceleration = 0;
        velocity = 0;
        targetPosition = 0;
        cartPosition = 0;

        // check if we should turn on the machine
        if (abs(theta) < 0.05) {
            state = STARTING;
            lastTime = millis();
        }
    }

    // STARTING -> get ready to start the machine
    else if (state == STARTING) {
        // turn on the stepper motor
        digitalWrite(STEPPER_EN_PIN, LOW); 
        if (millis() - lastTime > 10) {
            state = RUNNING;
        }
    }

    // RUNNING -> Implement all feedback, integration, etc.
    else if (state == RUNNING) {

        // P Controller Feedback
        acceleration = -K*theta;

        // update velocity, target position
        velocity += acceleration * Ts;
        targetPosition += velocity * Ts;

        updateStepperMotor();

        // check if we should turn off
        if (abs(theta) > 0.25) {
            state = STOPPED;
        }
    }

    // UPDATE GAIN
    // runs once per second
    if (counter == 0) {
        counter = 1000;
        K = 200.0 * (analogRead(A0) / 1024.0);

        // report information back to serial monitor
        if (abs(theta) < 0.2) {
            Serial.println(theta*180/PI);
        }
    }
    counter--;
}

// idle
void loop(){ }