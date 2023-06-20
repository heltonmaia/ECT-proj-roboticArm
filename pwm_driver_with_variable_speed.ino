#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <VarSpeedServo.h>

// Valores mínimos e máximos dos servos
#define servoMIN 100
#define servoMAX 460

// Cria os objetos (driver + servos)
Adafruit_PWMServoDriver driver = Adafruit_PWMServoDriver();
VarSpeedServo servo1, servo2;

// Indice dos servos no driver (valores de 0 a 15)
unsigned int servo1_pin = 12;
unsigned int servo2_pin = 15;

// Função para mover a posição de um servo informando o angulo em graus
void mover(VarSpeedServo& servo, unsigned int velocidade, unsigned int angulo){

  // Usa a função map pra converter graus para pwm
  unsigned int pwm = map(angulo, 0, 180, servoMIN, servoMAX);

  // Move para a posição desejada
  servo.slowmove(pwm, velocidade);
}

void setup() {
  // Baudrate
  Serial.begin(9600);

  // Inicia o driver
  driver.begin();

  // Frequencia PWM
  driver.setPWMFreq(60);

  // Associa os servos aos seus respectivos indices no PWM Driver
  servo1.attach(servo1_pin);
  servo2.attach(servo2_pin);
}

void loop() {
  
  // Move o servo 1 para angulo 90° com velocidade em 30%
  mover(servo1, 90, 30);

  // Move o servo 2 para angulo de 120° com velocidade de 45%
  mover(servo2, 120, 45);

}
