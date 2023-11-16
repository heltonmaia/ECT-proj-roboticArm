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

void stopServos() {
  haste1.stop();
  haste2.stop();
  base.stop();
}

void loop() {
  int comando;

  // Testa se tem alguma porta com comunicação ativa
  if(Serial.available() > 0){

    // Recebe o comando mandado pelo Python e transforma num angulo
    comando = Serial.read();
    
    //FECHADA
    if(comando == '0'){
      garra.slowmove(5, g_speed);
    }

    //ABERTA
    else if(comando == '1'){
      garra.slowmove(120, g_speed);
    }

    //DIRETA
    else if(comando == '2'){
      base.slowmove(180, b_speed);
    }

    //ESQUERDA
    else if(comando == '3'){
      base.slowmove(0, b_speed);
    }

    //ACIMA
    // haste1 - de 160 até 180
    // haste2 - de 123 até 95
    else if(comando == '4'){
      haste1.slowmove(180, h_speed);
      haste2.slowmove(95, h_speed);
    }

    //ABAIXO
    //haste1 - de 132 até 95
    //haste2 - de 151 até 180
    else if(comando == '5'){
      haste1.slowmove(95, h_speed);
      haste2.slowmove(180, h_speed);
    }

    //PARADA
    else if (comando == '6'){
      stopServos();
    }

    else {
      Serial.println("No command detected...");
    }
    
  }

}
