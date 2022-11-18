
/*             PINOUT
 * Encoder Pin A ---> Pin 2  = Port D2
 * Encoder Pin B ---> Pin 6  = Port D6
 *
 * Motor Sleep   ---> Pin 3  = Port D3
 * Motor Step    ---> Pin 4  = Port D4
 * Motor Dir     ---> Pin 5  = Port D5
 *
 * Motor M2      ---> Pin 8  = Port B0
 * Motor M1      ---> Pin 9  = Port B1
 * Motor M0      ---> Pin 10 = Port B2
 * 
 * Pot           ---> PinA0
 */

#define ENCODER_PIN_A 2
#define ENCODER_PIN_B 6

#define STEPPER_STEP_PIN 3
#define STEPPER_DIR_PIN 4
#define STEPPER_SLEEP_PIN 5

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

// 10kHz interrupt
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
// step mode = 16 means 16 microsteps per tick
unsigned long timeUntilNextStep(float velocity, uint8_t stepMode)
{
  float v = abs(velocity);

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

void updateStepMode(uint8_t mode)
{ //  PB2,  PB1,  PB0
  switch (mode)
  { //   M0,   M1,   M2
  case 1:
    PORTB &= ~0x7;
    break; //  Low,  Low,  Low   Full Step                 working
  case 2:
    PORTB &= ~0x3;
    PORTB |= 0x4;
    break; //  Low, High, High   Half Step                 working
  case 4:
    PORTB &= ~0x5;
    PORTB |= 0x2;
    break; // High,  Low, High   Quarter Step              working
  case 8:
    PORTB &= ~0x1;
    PORTB |= 0x6;
    break; // High, High,  Low   Eighth Step               working
  case 16:
    PORTB |= 0x7;
    break; // High, High, High   Sixteenth Step            working
  default:
    PORTB &= ~0x7;
    break;
  }
}

// solved velocity equation so that > 500us spacing between pulses (loop runs every 50 us)
// except for stepMode 1 (can step as fast as possible in stepMode 1)
uint8_t determineStepMode(float velocity)
{
  float v = abs(velocity);
  if (v < 0.02)
  {
    return 16;
  }
  if (v < 0.04)
  {
    return 8;
  }
  if (v < 0.08)
  {
    return 4;
  }
  if (v < 0.16)
  {
    return 2;
  }
  return 1;
}

// linear acceleration in m/s^2
// linear velocity in m/s


void setup()
{
  setupTimer1();
  setupTimer2();
  setupEncoder();

  pinMode(A0, INPUT);

  // turn motor on
  digitalWrite(STEPPER_SLEEP_PIN, HIGH);

  // set M0, M1, M2 as outputs
  DDRB |= _BV(2) | _BV(1) | _BV(0);

  /* begin Serial Communication */
  Serial.begin(115200);
  Serial.println("Custom Controller Testing");
}

bool isRunning = false;
unsigned long now = 0;
unsigned long nextStepTime;
unsigned long lastStepTime;
uint8_t stepMode;
const float Ts = 50e-6;

float maxVelocity = 1.00;
float acceleration = 0.0; // typical values: 0.1 - 1
float prevAcceleration = 0.0;
float velocity = 0;
float cartPosition;

#define DT_STEPS 20
int16_t prevEncoderPos[DT_STEPS] = {0};
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

  // microstepping
  stepMode = determineStepMode(velocity);
  updateStepMode(stepMode);
  nextStepTime = lastStepTime + timeUntilNextStep(velocity, stepMode);


  if (now > nextStepTime)
  {
    step();
    lastStepTime = now;

    if (velocity > 0)
    {
      cartPosition += 1.0 / stepMode;
    }
    else
    {
      cartPosition -= 1.0 / stepMode;
    }
  }


  
  if (isRunning && abs(encoderPos) < 75)
  {
    // Feedback control. Set the acceleration
    // K_propertional typical values: 0.5 +/- 0.1

    //acceleration = 3*K_Proportional*encoderPos + 300.0 * (encoderPos - getPreviousEncoderPos()) - 0.9*prevAcceleration;// + 2 * (encoderPos - getPreviousEncoderPos()); // + 10 * (encoderPos - prevEncoderPos[49]);
    acceleration = K_Proportional*encoderPos;
    prevAcceleration = acceleration;
    
    acceleration *= -1;

    prevEncoderPos[encoderIndex] = encoderPos;
    encoderIndex = (encoderIndex+1) % DT_STEPS;
    // shift
  }
  else if (!isRunning && abs(encoderPos) < 25)
  {
    // system is not running, but will be shortly
    digitalWrite(STEPPER_SLEEP_PIN, HIGH);
    delay(100);
    isRunning = true;
    velocity = 0;
    acceleration = 0;
  }
  else
  {
    // systen is not running. remain off
    digitalWrite(STEPPER_SLEEP_PIN, LOW);
    velocity = 0;
    acceleration = 0;
    isRunning = false;

    for(int i=0; i < DT_STEPS; i++) {
      prevEncoderPos[i] = 0;
    }
  }



}

// idle
void loop(){ }


int16_t getPreviousEncoderPos() {
  uint8_t i;
  if (encoderIndex == 0) {
    i = DT_STEPS - 1;
  }
  else {
    i = encoderIndex-1;
  }
  
  return prevEncoderPos[i];
}
