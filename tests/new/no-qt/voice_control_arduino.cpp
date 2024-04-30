#include <VarSpeedServo.h>

#define garra_pin 2
#define g_speed 45

VarSpeedServo garra;

void setup() {
  garra.attach(garra_pin);
  Serial.begin(9600);
}

void loop() {
  int comando;

  if(Serial.available() > 0){

    comando = Serial.read();

    if(comando == '0'){
      garra.slowmove(10, g_speed);
    }

    if(comando == '1'){
      garra.slowmove(120, g_speed);
    }

  }

}
