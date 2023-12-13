#include <VarSpeedServo.h>

#define g_speed 30
#define b_speed 20
#define h_speed 10
#define haste1_pin 3
#define haste2_pin 4
#define base_pin 5
#define garra_pin 2

VarSpeedServo garra;
VarSpeedServo base;
VarSpeedServo haste1;
VarSpeedServo haste2;
int angulo_base = 0;
int angulo_garra = 0;
int angulo_haste1 = 0;
int angulo_haste2 = 0;

void setup() {
  base.attach(base_pin);
  garra.attach(garra_pin);
  haste1.attach(haste1_pin);
  haste2.attach(haste2_pin);

  Serial.begin(9600);
}

void loop() {
    base.slowmove(0, h_speed);
    //haste1.slowmove(120, h_speed);
    //haste2.slowmove(150, h_speed);
    

}
