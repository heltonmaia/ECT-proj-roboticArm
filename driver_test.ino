#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Valores mínimos e máximos dos servos
#define servoMIN 100
#define servoMAX 460

Adafruit_PWMServoDriver driver = Adafruit_PWMServoDriver();

// Função slowmove
void slowmove(unsigned int servoIndice, unsigned int delay_ms, unsigned int angulo_destino, unsigned int angulo_origem){

  if(angulo_destino > angulo_origem){
    for(int i = angulo_origem; i < angulo_destino; ++i){
      driver.setPWM(servoIndice, 0, i);
      delay(delay_ms);
    }
  } else {
    for(int i = angulo_origem; i > angulo_destino; --i){
      driver.setPWM(servoIndice, 0, i);
      delay(delay_ms);
    }
  }
}

// Função para mover a posição de um servo informando o angulo em graus
void mover(unsigned int servoIndice, unsigned int angulo){

  // Move para a posição desejada
  driver.setPWM(servoIndice, 0, pwm);
}

void setup() {
  Serial.begin(9600);
  // Inicia o driver
  driver.begin();
  // Frequencia PWM
  driver.setPWMFreq(60);
}

void loop() {
  
  // Move os servos
  slowmove(12, 50, 90, 0);
  slowmove(13, 50, 90, 0);
  slowmove(14, 50, 90, 0);
  slowmove(15, 50, 90, 0);

}
