#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Valores mínimos e máximos dos servos
#define servoMIN 100
#define servoMAX 460

Adafruit_PWMServoDriver driver = Adafruit_PWMServoDriver();
int posicaoAtual[4];
// A posição inicial de cada servo eh 90 graus
int ang = 90;

// Função slowmove
void slowmove(unsigned int servoIndice, unsigned int delay_ms, unsigned int angulo_destino){

  if(posicaoAtual[servoIndice - 12] == angulo_destino) return;

  if(angulo_destino > posicaoAtual[servoIndice - 12]){

    for(int i = posicaoAtual[servoIndice - 12]; i < angulo_destino; ++i){
      driver.setPWM(servoIndice, 0, i);
      delay(delay_ms);
    }

  } else {

    for(int i = posicaoAtual[servoIndice - 12]; i > angulo_destino; --i){
      driver.setPWM(servoIndice, 0, i);
      delay(delay_ms);
    }

  }

  posicaoAtual[servoIndice - 12] = angulo_destino;

}

void setup() {
  Serial.begin(9600);
  // Inicia o driver
  driver.begin();
  // Frequencia PWM
  driver.setPWMFreq(60);

  unsigned int pwm = map(ang, 0, 180, servoMIN, servoMAX);

  // Armazena as posições atuais de cada servo no array de posições
  for(int i = 12; i < 16; ++i){
    driver.setPWM(i, 0, pwm);
    posicaoAtual[i - 12] = ang;
  }
}

void loop() {
  
  // Move os servos
  slowmove(12, 50, 90, 0);
  slowmove(13, 50, 90, 0);
  slowmove(14, 50, 90, 0);
  slowmove(15, 50, 90, 0);

}
