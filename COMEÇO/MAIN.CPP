#include <Arduino.h>
#include "Drone.h"

// Defina os pinos do controle remoto
const int pinPWM_Read1 = ; // Pino 1 do controle remoto
const int pinPWM_Read2 = ; // Pino 2 do controle remoto
const int pinPWM_Read3 = ; // Pino 3 do controle remoto
const int pinPWM_Read4 = ; // Pino 4 do controle remoto

// Defina os pinos dos motores
const int pinMotor1 = 25;
const int pinMotor2 = 26;
const int pinMotor3 = 27;
const int pinMotor4 = 14;

// Defina os canais PWM para os motores
const int chMotor1 = 0;
const int chMotor2 = 1;
const int chMotor3 = 2;
const int chMotor4 = 3;

// Defina a frequência e resolução do PWM
const int freq = 500;
const int resolution = 8;

Drone drone;

void setup() {
  // Inicia a comunicação serial
  Serial.begin(115200);

  // Configura o PWM para os motores
  drone.setupPWM(freq, resolution, pinMotor1, chMotor1);
  drone.setupPWM(freq, resolution, pinMotor2, chMotor2);
  drone.setupPWM(freq, resolution, pinMotor3, chMotor3);
  drone.setupPWM(freq, resolution, pinMotor4, chMotor4);

  // Configura o drone
  drone.MainControlSetup(115200, pinPWM_Read1, pinPWM_Read2, pinPWM_Read3, pinPWM_Read4);
}

void loop() {
  // Função principal de controle de voo
  drone.MainControlLoop(pinPWM_Read1, pinPWM_Read2, pinPWM_Read3, pinPWM_Read4);

  // Controla a velocidade dos motores
  drone.controlSpeed(drone.mot1(), chMotor1);
  drone.controlSpeed(drone.mot1(), chMotor2);
  drone.controlSpeed(drone.mot1(), chMotor3);
  drone.controlSpeed(drone.mot1(), chMotor4);

  // Adicione outras funções de controle e visualização de dados conforme necessário
  drone.DisplaySerialMpuData();
  drone.DisplayPlotterMpuData();
}
