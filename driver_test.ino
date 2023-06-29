#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Valores mínimos e máximos dos servos
#define servoMIN 100
#define servoMAX 460

Adafruit_PWMServoDriver driver = Adafruit_PWMServoDriver();

// Função para mover a posição de um servo informando o angulo em graus
void mover(unsigned int servoIndice, unsigned int angulo){

  // Usa a função map pra converter graus para pwm
  unsigned int pwm = map(angulo, 0, 180, servoMIN, servoMAX);

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
  mover(12, 90);
  mover(13, 90);
  mover(14, 90);
  mover(15, 90);

}
