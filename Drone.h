// Arquivo DRONE.h
#ifndef DRONE_H
#define DRONE_H

#include "Arduino.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


class Drone{
private:
    Adafruit_MPU6050 mpu;
  
public:

    Drone();


    bool calibration; //Calibração dos controles.


    uint8_t INPin1,INPin2, INPin3, INPin4;     //Valores ods pinos.


    uint32_t LoopTimer; // Definição do timer que vai controlar a freq de controle do motor.


    int ThrottleIdle, ThrottleCutOff, //Valor inicial e de desiligamento das helices.
        CH1, CH2, CH3, CH4, //Estado dos controles.
        MotorVeloci1, MotorVeloci2, MotorVeloci3, MotorVeloci4, // Variáveis de controle da velocidade dos motores.
        speed1, speed2, speed3, speed4, //Valor das velocidades.
        count, Timer1, Timer2; //Outros


        //Kalman
    float KalmanGain, KalmanAngleRoll, KalmanUncertaintyAngleRoll, KalmanAnglePitch, KalmanUncertaintyAnglePitch, //Ajuste dos valores.
        //PID
          PRateRoll, PRatePitch, PRateYaw, IRateRoll,  IRatePitch, IRateYaw, DRateRoll, DRatePitch, DRateYaw, //Variaveis PID.
          PAngleRoll, PAnglePitch, IAngleRoll, IAnglePitch, DAngleRoll, DAnglePitch, //PID para o controlador.
          DesiredAngleRoll, DesiredAnglePitch, //Angulos desejados.
          ErrorAngleRoll  , ErrorAnglePitch, //Variaveis de erro.
          //Variaveis de erro prévio...
          PrevErrorAngleRoll, PrevErrorAnglePitch, //ANGLE MODE
          PrevItermAngleRoll, PrevItermAnglePitch, 
          PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw, //RATE MODE
          PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw,
          // Variáveis de controle do controlador PID interno baseado na taxa de variação angular...
          RateRoll, InputRoll, DesiredRateRoll, ErrorRateRoll,
          RatePitch, InputPitch, DesiredRatePitch, ErrorRatePitch,
          RateYaw, InputYaw, DesiredRateYaw, ErrorRateYaw,
          InputThrottle,
          // Variáveis de controle do sensor giroscópio
          RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw,
          AceX, AceY, AceZ, Temp,
          GyrXc, GyrYc, GyrZc,
          AngleRoll, AnglePitch;
        


	void Kalman1D(float &KalmanState,float &KalmanUncertainty, const float &KalmanInput, 
					      const float &KalmanMeasurement);
    void CalibrarMPU();
    void MPUgetSignalsLoop();
    void MPUconfigSetup();
    void setupPWM(const int &freq, const int &resolution, const int &pin, const int &ch);
    void controlSpeed(int &speed, int ch);
    void readPWMSetup(const uint8_t &PinX);
    int  readPWMLoop(const uint8_t &PinX);
    void reset_pid();
    void pid_equation(const float &Error, const  float &P , const float &I, const float &D, 
					  float &PrevError, float &PrevIterm, float &Angle);
    void MainControlSetup(const int &serial, const int &pin1, const int &pin2, const int &pin3, 
                          const int &pin4, const int &pin5, const int &pin6, const int &pin7, 
                          const int &pin8);
    void MainControlLoop();


    //Funções de facilidade
    void pid_angle();
    void pid_rate();
    void DisplaySerialMpuData();
	void DisplayPlotterMpuData();
    void readPWMLoop_SM(int pinX, int ch);
};

#endif
