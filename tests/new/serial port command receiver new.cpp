#include <VarSpeedServo.h>

#define h_speed 15
#define g_speed 45
#define b_speed 30
#define garra_pin 2
#define haste1_pin 3
#define haste2_pin 4
#define base_pin 5

VarSpeedServo garra;
VarSpeedServo haste1;
VarSpeedServo haste2;
VarSpeedServo base;

void setup() {
  garra.attach(garra_pin);
  haste1.attach(haste1_pin);
  haste2.attach(haste2_pin);
  base.attach(base_pin);
  Serial.begin(9600);
}

void loop() {
  int comando;

  // Testa se tem alguma porta com comunicação ativa
  if(Serial.available() > 0){

    // Recebe o comando mandado pelo Python e transforma num angulo
    comando = Serial.read();
    
    //FECHADA
    if(comando == '0'){
      garra.slowmove(10, g_speed);
    }

    //ABERTA
      if(comando == '1'){
      garra.slowmove(120, g_speed);
    }

    //ACIMA
    if(comando == '+'){
      haste1.slowmove(160, h_speed);
      haste2.slowmove(123, h_speed);
    }

    if(comando == '*'){
      haste1.slowmove(170, h_speed);
      haste2.slowmove(109, h_speed);
    }

    if(comando == ')'){
      haste1.slowmove(180, h_speed);
      haste2.slowmove(95, h_speed);
    }

    //ABAIXO
    if(comando == '3'){
      haste1.slowmove(132, h_speed);
      haste2.slowmove(151, h_speed);
    }

    if(comando == '4'){
      haste1.slowmove(114, h_speed);
      haste2.slowmove(165, h_speed);
    }

    if(comando == '5'){
      haste1.slowmove(95, h_speed);
      haste2.slowmove(180, h_speed);
    }

    //DIRETA
    if(comando == '\x18'){
      base.slowmove(112, b_speed);
    }

    if(comando == '\x17'){
      base.slowmove(135, b_speed);
    }

    if(comando == '\x16'){
      base.slowmove(157, b_speed);
    }

    if(comando == '\x15'){
      base.slowmove(180, b_speed);
    }

    //ESQUERDA
    if(comando == '\x1f'){
      base.slowmove(67, b_speed);
    }

    if(comando == ' '){
      base.slowmove(45, b_speed);
    }

    if(comando == '!'){
      base.slowmove(22, b_speed);
    }

    if(comando == '"'){
      base.slowmove(0, b_speed);
    }
    
  }

}
