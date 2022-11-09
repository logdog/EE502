#include <AccelStepper.h>
#include <Encoder.h>

/* Hardware Pins */
#define SLEEP_PIN   8
#define STEP_PIN    7
#define DIR_PIN     6

#define ENC_PIN_A   2
#define ENC_PIN_B   5

/* Gantry Physical Range */
#define MAX_POS 2000
#define MIN_POS -2000

/* Standard Speed and Acceleration Values */
#define MAX_SPEED     2500
#define ACCELERATION  5000

#define FEEDBACK_GAIN -150

/* Stepper Motor and Encoder */
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
Encoder enc(ENC_PIN_A, ENC_PIN_B);

/* Global Variables */
float theta = 270;
bool inverted;
long t1, t2;

enum State {Idle, Countdown, Balancing, Done}; 
enum State state = Idle;

void resetAll() {
  Serial.println("RESET");
  enc.write(1800); // Set the encoder to equal 270 degrees being down (270*2400/360 = 1800)
  stepper.setCurrentPosition(0);
  digitalWrite(SLEEP_PIN, LOW); // put to sleep
}

void updateEncoder() {
  float value = (float) enc.read();
  float angle = value * 0.15; //  Note: 360/2400 = 0.15

  // put angle in range [0, 360) with x-axis equal to 0 degrees
  while (angle >= 360) angle -= 360;
  while (angle < 0) angle += 360;

  theta = angle;
  inverted = (angle < 180);
}

void setup()
{  
   /* Encoder Needs Pullups */
   pinMode(ENC_PIN_A, INPUT_PULLUP);
   pinMode(ENC_PIN_B, INPUT_PULLUP);

   pinMode(SLEEP_PIN, OUTPUT);

   /* Set Unlimited Speed and Acceleration, Initially */
   stepper.setMaxSpeed(MAX_SPEED); 
   stepper.setAcceleration(ACCELERATION);
   stepper.moveTo(0);

   /* High Baud Rate */
   Serial.begin(115200);
   Serial.println("P Controller Demo");

   resetAll();
}

float bestSpeed;

void loop()
{  
   long pos = stepper.currentPosition();
   updateEncoder();

   /* state machine logic */
   switch(state) {
    
    case Idle: {
      
      digitalWrite(SLEEP_PIN, LOW); // put to sleep
      bestSpeed = 0;
      
      if (abs(theta-90) < 10) {
        t1 = millis();
        state = Countdown;
      }

      if (Serial.available() > 0) {
        byte b = Serial.read();
        // RESET
        if (b == 'r') {
          resetAll();
        }
      }

      break;
    } 
    
    case Countdown: {
      long dt = millis() - t1;
      Serial.println("Get ready!");

      // wake up
      digitalWrite(SLEEP_PIN, HIGH);
      if (dt > 100) {
        state = Balancing;
        t2 = millis();
      }
      break;
    }

    // where the magic happens
    case Balancing: {
      stepper.setAcceleration(FEEDBACK_GAIN*(theta-90));
      stepper.move( FEEDBACK_GAIN*(theta-90));
      stepper.run();

      bestSpeed = max(stepper.speed(), bestSpeed);

      // we failed
      if (abs(theta-90) > 45) {
        Serial.println("We lasted ");
        Serial.print( (millis() - t2)/1000.0 );
        Serial.println(" seconds");
        Serial.println("Best Speed: ");
        Serial.print(bestSpeed);
        Serial.println("\n");
        state = Done;
      }
      break;
    }

    // send back to the 0 position
    case Done: {
      
      Serial.println("Returning to home...");
      delay(3000);

      stepper.setMaxSpeed(500);
      stepper.setAcceleration(1000);
      stepper.setSpeed(50);
      stepper.moveTo(0);

      // blocking function call
      stepper.runToPosition();
      
      Serial.println("Successfully returned to home");

      // return values to normal
      stepper.setMaxSpeed(MAX_SPEED);
      stepper.setAcceleration(ACCELERATION);
      
      state = Idle;
    }
    break;
   }
}
