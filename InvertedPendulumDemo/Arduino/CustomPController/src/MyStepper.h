#ifndef MYSTEPPER_HPP
#define MYSTEPPER_HPP

#include "Arduino.h"

// since we are using timers,
// we do not have flexibility over which pin does what
#define STEPPER_STEP_PIN  3
#define STEPPER_DIR_PIN   4
#define STEPPER_SLEEP_PIN 5

class MyStepper {
public:
    MyStepper();
    ~MyStepper();

    void run();
    void setAcceleration(double acceleration);
    void setMaxAcceleration(double acceleration);
    void setMaxSpeed(double maxSpeed);
    void turnOff();
    void turnOn();

private:
    long position;
    double acceleration;
    double velocity;
    double maxSpeed;
    double maxAcceleration;
    uint8_t direction; // 1 is forward, 0 is reverse

    void step();
    void updateDirection();

    unsigned long computeNextStepTime(int k);
    unsigned long computeNextStepTime();
    unsigned long nextStepTime;
    unsigned long lastStepTime;
    unsigned long stepNumber;
};


#endif