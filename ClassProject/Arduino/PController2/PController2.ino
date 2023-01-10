
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

#define SHAFT_RADIUS 0.006
#define STEPPER_PPR 200.0
#define PULSE_WIDTH 16

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

#define AB_MASK (_BV(2) | _BV(6))
#define A1_B1 (_BV(2) | _BV(6))
#define A1_B0 (_BV(2))
#define A0_B1 (_BV(6))
#define A0_B0 0x00

// triggers on rising and falling edge of D2
// encoder has 1200 positions using this method (2400 if we use other signal too)
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
  OCR1A = 49;

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

// calculate useconds until next step
// velocity is the cart's velocity in m/s
// step mode = 8 means 8 microsteps per tick
unsigned long timeUntilNextStep(float velocity)
{
  float v = abs(velocity);
  uint8_t stepMode = 8;

  // if (v < 1e-3)
  // {
  //   return 4000000000;
  // }
  return (unsigned long)(1e6 * 2.0 * PI * SHAFT_RADIUS / (v * stepMode * STEPPER_PPR)) - PULSE_WIDTH;
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
  setupTimer1();
  setupTimer2();
  setupEncoder();

  pinMode(A0, INPUT);
  pinMode(STEPPER_DIR_PIN, OUTPUT);
  pinMode(STEPPER_EN_PIN, OUTPUT);

  // turn motor on
  digitalWrite(STEPPER_EN_PIN, LOW);

  /* begin Serial Communication */
  Serial.begin(115200);
  Serial.println("Custom Controller Testing");
}

bool isRunning = false;
unsigned long now = 0;
unsigned long nextStepTime;
unsigned long lastStepTime;
const float Ts = 50e-6;

float maxVelocity = 1.00;
float acceleration = 0.0; // typical values: 0.1 - 1
float velocity = 0;
float cartPosition;

uint8_t encoderIndex;

uint16_t updateGainCounter;
float K_Proportional;

// runs every Ts (0.000_050) seconds
ISR(TIMER1_COMPA_vect)
{
  now += 50;
  velocity += acceleration * Ts;

  // change the gain (every 4096 loops == 4096 * 50e-6 = 240ms)
  if (updateGainCounter++ % 4096 == 0) {
    K_Proportional = 2.0 * analogRead(A0) / 1023.0; // analogRead returns 0 to 1023
  }

  //cap velocity
  if (velocity > maxVelocity)
  {
    velocity = maxVelocity;
  }
  else if (-velocity > maxVelocity)
  {
    velocity = -maxVelocity;
  }

  // move correct direction
  if (velocity > 0)
  {
    updateDirection(0);
  }
  else
  {
    updateDirection(1);
  }

  nextStepTime = lastStepTime + timeUntilNextStep(velocity);


  if (now > nextStepTime)
  {
    step();
    lastStepTime = now;

    if (velocity > 0)
    {
      cartPosition += 1.0/8;
    }
    else
    {
      cartPosition -= 1.0/8;
    }
  }


  if (isRunning && abs(encoderPos) < 75)
  {
    // Feedback control. Set the acceleration
    acceleration = K_Proportional*encoderPos;
  }
  else if (!isRunning && abs(encoderPos) < 25)
  {
    // system is not running, but will be shortly
    digitalWrite(STEPPER_EN_PIN, LOW);
    delay(100);
    isRunning = true;
    velocity = 0;
    acceleration = 0;
  }
  else
  {
    // systen is not running. remain off
    digitalWrite(STEPPER_EN_PIN, HIGH);
    velocity = 0;
    acceleration = 0;
    isRunning = false;
  }
}

// idle
void loop(){ }