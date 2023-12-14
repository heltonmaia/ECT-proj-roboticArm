#include <VarSpeedServo.h>

#define g_speed 30
#define b_speed 30 
#define h_speed 15
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
