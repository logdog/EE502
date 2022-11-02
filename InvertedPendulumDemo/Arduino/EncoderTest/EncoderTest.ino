#include <Encoder.h>

// interrupt pins are 2 and 3
// but pins 3 and 4 don't work
Encoder enc(2,5);

void setup() {
  Serial.begin(115200);
  Serial.println("Encoder Test");

  pinMode(2, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
}

long pos  = -999;
long lastTime = millis();

void loop() {
  long newPos;
  newPos = enc.read();
  if (newPos != pos) {
    pos = newPos;
  }


  if (millis() - lastTime > 1000) {
    Serial.print("Encoder Value = ");
    Serial.print(pos);
    Serial.println();
    lastTime = millis();
  }
  
}
