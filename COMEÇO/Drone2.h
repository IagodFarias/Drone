// Arquivo DRONE.h
#ifndef DRONE_H
#define DRONE_H

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


class Drone{
private:
    Adafruit_MPU6050 mpu;
  
   
 

public:
//  Métodos que vão ser utilizados

	void Kalman1D(float KalmanState,float KalmanUncertainty,
                  float KalmanInput,float KalmanMeasurement);
	void DisplaySerialMpuData();
	void DisplayPlotterMpuData();
    void CalibrarMPU();
    void MPUgetSignalsLoop();
    void MPUconfigSetup();
    void setupPWM(int freq, int resolution, int pin, int ch);
    void controlSpeed(int sped, int ch);
    void readPWMSetup(int pin1);
    int  readPWMLoop(int pinX);
    int mot1();
    void readPWMLoop_SM(int pinX, int ch);
    void reset_pid();
    void pid_equation(float Error, float P        , float I, 
     				  float D    , float PrevError, float PrevIterm);
    void MainControlSetup(int serial, int pin1, int pin2, int pin3, int pin4);
    void MainControlLoop(int pinPWM_Read1, int pinPWM_Read2, 
                         int pinPWM_Read3, int pinPWM_Read4);

};
// Diretiva usada para o pré-processador após um bloca ifndef
#endif
