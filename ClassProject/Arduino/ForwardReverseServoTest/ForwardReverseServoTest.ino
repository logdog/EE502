
#include <AccelStepper.h>

// using a driver to control stepper. STEP=, DIR=
AccelStepper stepper(AccelStepper::DRIVER,7,6);
uint32_t startTime;
void setup()
{  
   pinMode(6,OUTPUT);
   //pinMode(7,OUTPUT);
   stepper.setMaxSpeed(1000); 
   stepper.setAcceleration(1000);
   stepper.moveTo(0);
   startTime = millis();

   Serial.begin(9600);
}

int targetPos = 0;


void loop()
{  
   if (millis() - startTime > 250 && Serial.available() > 0) {
      startTime = millis();
      byte readChar = Serial.read();
      if (readChar == 'l' || readChar == 'L') {
        targetPos += 1500;
      }
      else if (readChar == 'r' || readChar == 'R') {
        targetPos -= 1500;
      }
      stepper.moveTo(targetPos);
      Serial.println(targetPos);
   }
   
   stepper.run();
}

//
//void loop() {
//  stepper.runSpeed();
//}

//void loop()
//{  
//   uint32_t dt = millis() - startTime;
//   if (dt < 1000) {
//    stepper.setSpeed(50);
//   }
//   else if (dt < 2000) {
//      stepper.setSpeed(0);
//   }
//   else if (dt < 3000) {
//      stepper.setSpeed(-50);
//   }
//   else if (dt < 4000) {
//     stepper.setSpeed(0);
//   }
//   else if (dt < 5000) {
//     startTime = millis();
//     return;
//   }
//   
//   stepper.runSpeed();
//   Serial.println(dt);
//}
