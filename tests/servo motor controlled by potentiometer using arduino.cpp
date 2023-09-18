#include <Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

int servo1_pin = 9;
int servo2_pin = 10;
int servo3_pin = 11;
int servo4_pin = 12;

int pot1_pin = A0;
int pot2_pin = A1;
int pot3_pin = A2;
int pot4_pin = A3;

void setup() {
  servo1.attach(servo1_pin);
  servo2.attach(servo2_pin);
  servo3.attach(servo3_pin);
  servo4.attach(servo4_pin);
}
void loop() {
  int val1 = analogRead(pot1_pin);
  int val2 = analogRead(pot2_pin);
  int val3 = analogRead(pot3_pin);
  int val4 = analogRead(pot4_pin);

  int servo1_pos = map(val1, 0, 1023, 0, 179);
  int servo2_pos = map(val2, 0, 1023, 0, 179);
  int servo3_pos = map(val3, 0, 1023, 0, 179);
  int servo4_pos = map(val4, 0, 1023, 0, 179);

  servo1.write(servo1_pos);
  servo2.write(servo2_pos);
  servo3.write(servo3_pos);
  servo4.write(servo4_pos);
  
}
