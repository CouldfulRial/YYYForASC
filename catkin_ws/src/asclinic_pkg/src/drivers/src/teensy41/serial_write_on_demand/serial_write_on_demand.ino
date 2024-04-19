/*
 * Read from the motor encoder on demand.
 */

#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

Encoder motor1(9, 10);
Encoder motor2(11, 12);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

long pos1 = -999;
long pos2 = -999;

void loop() {
  // put your main code here, to run repeatedly:
  long newpos1, newpos2;
  float speed1, speed2;
  char command_char;
  unsigned long curr_time;
  if (Serial.available() > 0) {
    command_char = Serial.read();
    if (command_char == '*') {
      // return position
      newpos1 = motor1.readAndReset();
      newpos2 = motor2.readAndReset();
      Serial.print("<");
      Serial.print(newpos1);
      Serial.print(",");
      Serial.print(newpos2);
      Serial.print(">"); 
    }
    else if (command_char == 's') {
      // return speed
      motor1.write(0);
      motor2.write(0);
      delay(10);
      newpos1 = motor1.readAndReset();
      newpos2 = motor2.readAndReset();
      speed1 = newpos1 * 0.0003125 / (10 * 1.667e-5);
      speed2 = newpos1 * 0.0003125 / (10 * 1.667e-5);
      Serial.print("[");
      Serial.print(speed1);
      Serial.print(",");
      Serial.print(speed2);
      Serial.print("]");
    }
    else if (command_char == 't') {
      newpos1 = motor1.readAndReset();
      newpos2 = motor2.readAndReset();
      curr_time = millis();
      Serial.print("{");
      Serial.print(newpos1);
      Serial.print(",");
      Serial.print(newpos2);
      Serial.print(",");
      Serial.print(curr_time);
      Serial.print("}");
    }
  }
}
