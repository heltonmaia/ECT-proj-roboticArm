#include <VarSpeedServo.h>

VarSpeedServo garra;
VarSpeedServo haste1;
VarSpeedServo haste2;
VarSpeedServo base;

int garra_pin = 2;
int haste1_pin = 3;
int haste2_pin = 4;
int base_pin = 5;

void setup() {
  garra.attach(garra_pin);
  haste1.attach(haste1_pin);
  haste2.attach(haste2_pin);
  base.attach(base_pin);
  Serial.begin(9600);
}

void loop() {
  int comando;
  int garra_pos;
  int haste1_pos;
  int haste2_pos;
  int base_pos;

  // Testa se tem alguma porta com comunicação ativa
  if(Serial.available() > 0){

    // Recebe o comando mandado pelo Python e transforma num angulo
    comando = Serial.read();
    
    if(comando == '0'){
      garra_pos = 10;
      garra.slowmove(garra_pos, 45);
    }

      if(comando == '1'){
      garra_pos = 120;
      garra.slowmove(garra_pos, 45);
    }

    if(comando == '2'){
      base_pos = 0;
      base.slowmove(base_pos, 30);
    }

    if(comando == '3'){
      base_pos = 60;
      base.slowmove(base_pos, 30);
    }

    if(comando == '4'){
      haste1_pos = 110;
      haste1.slowmove(haste1_pos, 60);
      haste2_pos = 90;
      haste2.slowmove(haste2_pos, 60);
    }

    if(comando == '5'){
      haste1_pos = 60;
      haste1.slowmove(haste1_pos, 60);
      haste2_pos = 150;
      haste2.slowmove(haste2_pos, 60);
    }
    
  }

}
