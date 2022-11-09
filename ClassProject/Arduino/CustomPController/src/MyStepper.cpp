#include "MyStepper.h"

MyStepper::MyStepper() {
    this->maxSpeed = 2000;
    this->maxAcceleration = 10000;

    // set direction, sleep pins as output
    pinMode(STEPPER_DIR_PIN, OUTPUT);
    pinMode(STEPPER_SLEEP_PIN, OUTPUT);

    // immediately put to sleep
    turnOff();

    /* setup the timer2 */
    TCCR2B =  0;    // disable timer2
    TCNT2 = 0x00;   // reset counter2 value
    OCR2A = 0x00;   // count up to this value, then reset back to zero
    OCR2B = 0xfe;   // creates 16us pulse. when TNT2>OCR2B, OC0B goes high

    // set timer2 into WGM mode 7 (Fast PWM)
    // enable OC0B output
    // run timer on clk/1024 prescaler (CS = 101)
    TCCR2A = _BV(COM2B0) | _BV(COM2B1) | _BV(WGM20) | _BV(WGM21);
    TCCR2B = _BV(WGM22) | _BV(CS22) | _BV(CS20);

    // set pin PD3 as output (step)
    DDRD |= _BV(3); // OC2B -> PD3 -> Pin 3

    // set acceleration
    setAcceleration(0);
}

MyStepper::~MyStepper() {}

void MyStepper::setAcceleration(double acceleration) {
    this->acceleration = abs(acceleration);
    direction = (acceleration >= 0) ? 1 : 0;
    updateDirection();

    stepNumber = 0;
    lastStepTime = micros();
    nextStepTime = computeNextStepTime();
}

void MyStepper::setMaxSpeed(double maxSpeed) {
    this->maxSpeed = maxSpeed;
}

void MyStepper::setMaxAcceleration(double acceleration) {
    this->maxAcceleration = acceleration;
}

void MyStepper::turnOff() {
    digitalWrite(STEPPER_SLEEP_PIN, LOW);
}

void MyStepper::turnOn() {
    digitalWrite(STEPPER_SLEEP_PIN, HIGH);
}

void MyStepper::updateDirection() {
    digitalWrite(STEPPER_DIR_PIN, direction);
}

void MyStepper::step() {
  TCNT2 = OCR2B - 1;
}

void MyStepper::run() {
    unsigned long currentTimeUS = micros();
    if (currentTimeUS >= nextStepTime) {
        // step (15.8 microsecond pulse, logical HIGH)
        step();

        // update nextStepTime
        lastStepTime = currentTimeUS;
        nextStepTime = computeNextStepTime();
    } 
}

unsigned long MyStepper::computeNextStepTime(int k) {
    return (unsigned long) (1e6 * sqrt(2*k/acceleration));
}

unsigned long MyStepper::computeNextStepTime() {
    unsigned long candidateTime = (unsigned long) (1e6 * sqrt((2*++stepNumber)/acceleration));
    if (candidateTime - lastStepTime)
    return ;
}