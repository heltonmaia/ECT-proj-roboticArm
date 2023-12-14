#include <VarSpeedServo.h>

#define gripper_speed 30
#define rotating_base_speed 30 
#define arms_speed 15
#define arm1_pin 3
#define arm2_pin 4
#define rotating_base_pin 5
#define gripper_pin 2

VarSpeedServo gripper;
VarSpeedServo rotating_base;
VarSpeedServo arm1;
VarSpeedServo arm2;
int rotating_base_angle = 0;
int gripper_angle = 0;
int arm1_angle = 0;
int arm2_angle = 0;

void setup() {
  rotating_base.attach(rotating_base_pin);
  gripper.attach(gripper_pin);
  arm1.attach(arm1_pin);
  arm2.attach(arm2_pin);

  Serial.begin(9600);
}

void loop() {

    if(Serial.available() >= 4){

    angulo_base = Serial.read();
    angulo_garra = Serial.read();
    angulo_haste1 = Serial.read();
    angulo_haste2 = Serial.read();

    base.slowmove(angulo_base, b_speed);
    garra.slowmove(angulo_garra, g_speed);
    haste1.slowmove(angulo_haste1, h_speed);
    haste2.slowmove(angulo_haste2, h_speed);
    
  }

}
