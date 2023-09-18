#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <VarSpeedServo.h>

// Valores mínimos e máximos dos servos
#define servoMIN 100
#define servoMAX 460

// Cria os objetos (driver + servos)
Adafruit_PWMServoDriver driver = Adafruit_PWMServoDriver();
VarSpeedServo garra, haste1, haste2, base;

// Indice dos servos no driver (valores de 0 a 15)
unsigned int garra_pin = 12;
unsigned int haste1_pin = 13;
unsigned int haste2_pin = 14;
unsigned int base_pin = 15;

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
  garra.attach(garra_pin);
  haste1.attach(haste1_pin);
  haste2.attach(haste2_pin);
  base.attach(base_pin);
}

void loop() {
  
  // Move os servos
  mover(garra, 30, 90);
  mover(haste1, 30, 90);
  mover(haste2, 30, 90);
  mover(base, 30, 90);

}
