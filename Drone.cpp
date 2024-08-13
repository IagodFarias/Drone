// ESP32 Guide: https://randomnerdtutorials.com/esp32-mpu-6050-accelerometer-gyroscope-arduino/
// ESP8266 Guide: https://randomnerdtutorials.com/esp8266-nodemcu-mpu-6050-accelerometer-gyroscope-arduino/
// Arduino Guide: https://randomnerdtutorials.com/arduino-mpu-6050-accelerometer-gyroscope/
#include "Drone.h"

//*****************************************ORGNIAZAÇÃO DO DRONE*******************************************
Drone::Drone()
{	
	//ESTADO DE CALIBRAÇÃO INICIAL
	calibration = false;

	//VALORES DOS MOTORES
	ThrottleIdle   = 257; // Valor mínimo para velocidade dos motores durante o voo
	ThrottleCutOff = 257; // Valor para parar os motores

	//GIROSCOPIO
	GyrXc = GyrYc = GyrZc = 0;

	//FILTRO DE KALMAN
	KalmanGain = 0;
	KalmanAngleRoll  = 0, KalmanUncertaintyAngleRoll  = 2*2;
	KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2*2;

	//PID
		//PID Interno
		ErrorRateRoll = ErrorRatePitch = ErrorRateYaw = 0;

		//DADOS PARA O PID
			// Variáveis de erro prévio...
			PrevErrorAngleRoll = PrevErrorAnglePitch = 0; // Angle
			PrevItermAngleRoll = PrevItermAnglePitch = 0;

			PrevErrorRateRoll = PrevErrorRatePitch = PrevErrorRateYaw = 0; // Rate
			PrevItermRateRoll = PrevItermRatePitch = PrevItermRateYaw = 0;
	
		//Constantes PID
			//PID Angle
			PAngleRoll = PAnglePitch = 2; // P

			IAngleRoll = IAnglePitch = 0; // I

			DAngleRoll = DAnglePitch = 0; // D

		//PID Rate 
		PRateRoll = PRatePitch = 0.6f;  PRateYaw   = 1; // P
		IRateRoll = IRatePitch = 3.5f;  IRateYaw   = 6; // I
		DRateRoll = DRatePitch = 0.03f; DRateYaw  = 0;  // D

	//OUTROS
	Timer1 = Timer2 = count = 0;
	
}
//***********************************FILTRO KALMAN DE 1 DIMENSÃO******************************************
void Drone::Kalman1D(float &KalmanState,float &KalmanUncertainty, const float &KalmanInput, 
					 const float &KalmanMeasurement)
{
	KalmanState       = KalmanState + 0.004*KalmanInput;
	KalmanUncertainty = KalmanUncertainty + 0.004*0.004*4*4;
	KalmanGain        = KalmanUncertainty*1/(1*KalmanUncertainty + 3*3);
	KalmanState       = KalmanState + KalmanGain*(KalmanMeasurement - KalmanState);
	KalmanUncertainty = (1 - KalmanGain)*KalmanUncertainty;
}
//********************************************************************************************************
void Drone::updatePID(int adress,float mod)
{	
	switch(adress)
	{
		case 0:
			PAngleRoll  += mod;
			PAnglePitch += mod;
			break;
		case 1:
			IAngleRoll  += mod;
			IAnglePitch += mod;
			break;
		case 2:
			DAngleRoll  += mod;
			DAnglePitch += mod;
			break;
		case 3:
			PRateRoll  += mod;
			PRatePitch += mod;
			break;
		case 4:
			IRateRoll  += mod;
			IRatePitch += mod;
			break;
		case 5:
			DRateRoll  += mod;
			DRatePitch += mod;
			break;
		case 6:
			PRateYaw += mod;
			break;
		case 7:
			IRateYaw += mod;
			break;
		case 8:
			DRateYaw += mod;
			break;
		case 65:
			PAngleRoll = PAnglePitch = 2;
			IAngleRoll = IAnglePitch = 0;
			DAngleRoll = DAnglePitch = 0;
			PRateRoll  = PRatePitch  = 0.6;
			IRateRoll  = IRatePitch  = 3.5;
			DRateRoll  = DRatePitch  = 0.03;
			PRateYaw                 = 1;
			IRateYaw                 = 6;
			DRateYaw                 = 0;
			break;
  			

	}
	Serial.printf("PAngleRoll: %0.2f\nIAngleRoll: %0.2f\nDAngleRoll: %0.2f\n", PAngleRoll, IAngleRoll, DAngleRoll);
	Serial.printf("PRateRoll: %0.2f\nIRateRoll: %0.2f\nDRateRoll: %0.2f\n", PRateRoll, IRateRoll, DRateRoll);
	Serial.printf("PRateYaw: %0.2f\nIRateYaw: %0.2f\nDRateYaw: %0.2f\n", PRateYaw, IRateYaw, DRateYaw);
}
//********************************************************************************************************
void Drone::MainControlSetup(const int &serial, const int &pin1, const int &pin2, const int &pin3, 
                             const int &pin4,   const int &pin5, const int &pin6, const int &pin7, 
                             const int &pin8) 
{
	INPin1 = pin1;
  	INPin2 = pin2;
  	INPin3 = pin3;
  	INPin4 = pin4;
  
  	Serial.begin(serial);
	readPWMSetup(pin1);
	readPWMSetup(pin2);
	readPWMSetup(pin3);
	readPWMSetup(pin4);

	setupPWM(250, 10, pin5, 0);
	setupPWM(250, 10, pin6, 1);
	setupPWM(250, 10, pin7, 2);
	setupPWM(250, 10, pin8, 3);

	MPUconfigSetup();
	CalibrarMPU();
}
//********************************************************************************************************
void Drone::MainControlLoop()
{
	//##############################FILTRO DE KALMAN############################

	//Filtrando os Angulos de Pitch e Roll
	Kalman1D(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);   
  
  	Kalman1D(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch); 

	//##############################LEITURA DO CONTROLE############################

	CH1 = readPWMLoop(INPin1); // obtenção dos comandos vindos do controle remoto...
	CH2 = readPWMLoop(INPin2); // ...
	CH3 = readPWMLoop(INPin3); // ...
	CH4 = readPWMLoop(INPin4); // ...

	//Visualização do sinal do controle no Monitor Serial
	//Serial.printf("Canal 1: %d\nCanal 2: %d\nCanal 3: %d\nCanal 4: %d\n", CH1, CH2,CH3,CH4);

	DesiredAngleRoll  = 0.10*(CH1 - 1500); // Valores desejados de ângulo obtidos...
	DesiredAnglePitch = 0.10*(CH2 - 1500); // a partir do controle remoto

	InputThrottle     = CH3; 			   // Potencia dos motores para movimento vertical
	DesiredRateYaw    = 0.15*(CH4 - 1500); // Valor do ângulo para giro no eixo Z

	ErrorAngleRoll    = DesiredAngleRoll  - KalmanAngleRoll;  // Calculo do erro de ângulo Roll e Pitch...
	ErrorAnglePitch   = DesiredAnglePitch - KalmanAnglePitch; // ...

	//##############################APLICAÇÃO DO PID############################

	pid_angle(); //Faz o PID nos angulo

	pid_rate(); //Faz o PID na variação dos angulos
	
	//##############################CONTROLE DOS MOTORES############################

	if (InputThrottle > 1800) InputThrottle = 1800; // Medida de segurança para manter a potência máxima de subida a 80% 

	MotorVeloci1 = 1.024*(InputThrottle - InputRoll - InputPitch - InputYaw); // Cálculo da entrada para os motores a partir...
	MotorVeloci2 = 1.024*(InputThrottle - InputRoll + InputPitch + InputYaw); // dos inputs vindos do controle remoto que... 
	MotorVeloci3 = 1.024*(InputThrottle + InputRoll + InputPitch - InputYaw); // passaram pela função PID...
	MotorVeloci4 = 1.024*(InputThrottle + InputRoll - InputPitch + InputYaw); // ...
	
	//##############################MEDIDAS DE SEGURANÇA############################

	if (MotorVeloci1 > 2000) MotorVeloci1 = 1999; // Medida de segurança para evitar extrapolação da potência máxima dos motores
	if (MotorVeloci2 > 2000) MotorVeloci2 = 1999; // ... 
	if (MotorVeloci3 > 2000) MotorVeloci3 = 1999; // ... 
	if (MotorVeloci4 > 2000) MotorVeloci4 = 1999; // ...
	
	if (MotorVeloci1 < ThrottleIdle) MotorVeloci1 = ThrottleIdle; // Medida de segurança para manter o drone voando com...
	if (MotorVeloci2 < ThrottleIdle) MotorVeloci2 = ThrottleIdle; // uma potência mínima...
	if (MotorVeloci3 < ThrottleIdle) MotorVeloci3 = ThrottleIdle; // ...
	if (MotorVeloci4 < ThrottleIdle) MotorVeloci4 = ThrottleIdle; // ...

	speed1 = map(MotorVeloci1,1000,2000,257,385);
	speed2 = map(MotorVeloci2,1000,2000,257,385);
	speed3 = map(MotorVeloci3,1000,2000,257,385);
	speed4 = map(MotorVeloci4,1000,2000,257,385);

	if (CH3 < 1050) // Medida de segurança para ter certeza do desligamento dos motores
	{ 
		speed1 = ThrottleCutOff; 
		speed2 = ThrottleCutOff;
		speed3 = ThrottleCutOff; 
		speed4 = ThrottleCutOff;
		reset_pid(); // Toda vez que o drone pousar o PID precisa ser resetado
	}

	controlSpeed(speed1,0);
	controlSpeed(speed2,1);
	controlSpeed(speed3,2);
	controlSpeed(speed4,3);
  	
	//Print dos valores para visualização do input aos motores
	//Serial.printf("Motor1: %d, Motor2 %d, Motor3 %d, Motor4 %d \n", MotorVeloci1, MotorVeloci2, MotorVeloci3, MotorVeloci4);

	//Print dos valores para visualização do input aos motores
	// Serial.printf("Motor1: %d, Motor2 %d, Motor3 %d, Motor4 %d \n", speed1, speed2, speed3, speed4);

	while (micros() - LoopTimer < 4000); // tempo de espera para novo loop de controle... 
	LoopTimer = micros(); 				 // a frequência usada é de 250Hz
}
//********************************************************************************************************
void Drone::reset_pid() 
{
	PrevErrorRateRoll = 0; PrevErrorRatePitch = 0; PrevErrorRateYaw = 0;
	PrevItermRateRoll = 0; PrevItermRatePitch = 0; PrevItermRateYaw = 0;	

	PrevErrorAngleRoll = 0; PrevErrorAnglePitch = 0; 
	PrevItermAngleRoll = 0; PrevItermAnglePitch = 0;
}
//********************************************************************************************************
void Drone::MPUconfigSetup() 
{
	if (!mpu.begin()) 
	{	
		while (1) 
		{
			Serial.println("error");
	  		yield();
		}
	}
  
	mpu.setAccelerometerRange(MPU6050_RANGE_8_G);//2G, 4G, 8G, 16G
	mpu.setGyroRange(MPU6050_RANGE_500_DEG);     //250deg/s, 500deg/s, 1000deg/s, 2000deg/s
	mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);  //5Hz, 10Hz, 21Hz, 44Hz, 94Hz, 184Hz, 260Hz
}
//********************************************************************************************************
void Drone::CalibrarMPU()
{
 	for (int Passo = 0 ; Passo < 2000; Passo++) 
	{
		Serial.printf("Calibrando: %d\n", Passo);

		MPUgetSignalsLoop();

		RateCalibrationRoll  += RateRoll;
		RateCalibrationPitch += RatePitch;
		RateCalibrationYaw   += RateYaw;
		delay(1);
	}
	
	calibration = true;
}
//********************************************************************************************************
// void Drone::setCalibrationValues()
// {
// }
//********************************************************************************************************
void Drone::MPUgetSignalsLoop() 
{	
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  AceX = a.acceleration.x / 9.81; // m/s^2...
  AceY = a.acceleration.y / 9.81; // ...
  AceZ = a.acceleration.z / 9.81; // ...
 
  RateRoll  = g.gyro.x * 57.3; // °/s...
  RatePitch = g.gyro.y * 57.3; // ...
  RateYaw   = g.gyro.z * 57.3; // ...
  
  Temp = temp.temperature; // °C
  
  AngleRoll  =  atan(AceY/sqrt(AceX*AceX + AceZ*AceZ))*1/(3.142/180);
  AnglePitch = -atan(AceX/sqrt(AceY*AceY + AceZ*AceZ))*1/(3.142/180);
  
  if (calibration = true){
	RateRoll  -= (RateCalibrationRoll/2000)  - 1.2;
	RatePitch -= (RateCalibrationPitch/2000) - 1.5;
	RateYaw   -= (RateCalibrationYaw/2000)   - 0;
	AceX -= 0.02;
	AceY -= 0.03;
	AceZ -= 0.13;
  }
}
//********************************************************************************************************
void Drone::DisplaySerialMpuData()
{	
	if ((millis() - Timer1) >= 800)
	{
		Timer1 = millis();
		
		Serial.printf("AceX %f AceY %f, AceZ %f \n", AceX, AceY, AceZ);
	
		Serial.printf("RateRoll: %f RatePitch %f RateYaw %f \n", RateRoll, RatePitch, RateYaw);
		
		Serial.printf("Angulo Roll %f\n Angulo Pitch %f\n", AngleRoll, AnglePitch);
	}
}
//********************************************************************************************************
void Drone::DisplayPlotterMpuData()
{
	Kalman1D(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);     // chamada da função do filtro afim...
  
  	Kalman1D(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch); // ...

	if ((millis() - Timer2) >= 10){
		Timer2 = millis();
		
		Serial.printf("Angle_Roll: %f, Angle_Pitch: %f, Kalman_Angle_Roll: %f, Kalman_Angle_Pitch: %f\n", 
		AngleRoll, AnglePitch, KalmanAngleRoll, KalmanAnglePitch);

		//Serial.println(GyrZ);
	}
}
//********************************************************************************************************
void Drone::setupPWM(const int &freq, const int &resolution, const int &pin, const int &ch)
{
  pinMode(pin, OUTPUT); // Definição do pino de saída do PWM de controle do motor
  ledcSetup(ch, freq, resolution);// ...
  ledcAttachPin(pin, ch); // Funções para definição do PWM na ESP32
}
//********************************************************************************************************
void Drone::controlSpeed(int &speed, int ch)
{
	// Função para controlar a velocidade dos motores
	// min_sped=257 max_sped=511 para resolução = 10 bits
	ledcWrite(ch, speed); // Função para mudança do PWM na ESP32
}
//********************************************************************************************************
void Drone::readPWMSetup(const uint8_t &PinX)
{
	// Função para adequar os pinos que vão receber o PWM do controlador
	pinMode(PinX, INPUT);
}
//********************************************************************************************************
void Drone::readPWMLoop_SM(int pinX, int ch)
{
	// Função para ler o valor do PWM enviado pelo controlador e printa-lo no monitor serial
	// Precisa ser usada para cada valor que precisar ser lido, logo no máximo 4 vezes
	Serial.print("CH");
	Serial.print(ch);	
	Serial.print(" = ");	
	Serial.println(pulseIn(pinX, HIGH));
	Serial.println(" ");

}
//********************************************************************************************************
int Drone::readPWMLoop(const uint8_t &PinX)
{
	return pulseIn(PinX, HIGH); 
}
//********************************FUNÇÃO GERAL PARA O CONTROLADOR PID*************************************
void Drone::pid_equation(const float &Error, const float &P , const float &I, const float &D, 
						 float &PrevError, float &PrevIterm, float &Angle)
{
	float Pterm = P*Error;
	float Iterm = PrevIterm + I*(Error + PrevError)*0.004/2;

	if      (Iterm >  400) Iterm = 400;
	else if (Iterm < -400) Iterm = -400;

	float Dterm     = D*(Error - PrevError)/0.004;
	float PIDOutput = Pterm + Iterm + Dterm;

	if      (PIDOutput >  400) PIDOutput =  400;
	else if (PIDOutput < -400) PIDOutput = -400;

	Angle     = PIDOutput;   //Obtendo os Angulos roll, pitch e yaw
	PrevError = Error;
	PrevIterm = Iterm;

}
void Drone::pid_angle()
{
	pid_equation(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll, PrevItermAngleRoll, DesiredAngleRoll); 

	pid_equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch, DesiredAnglePitch);
}
//********************************************************************************************************
void Drone::pid_rate()
{
	ErrorRateRoll  = DesiredRateRoll  - RateRoll;  // Cálculo dos erros de roll, pitch e yaw...
	ErrorRatePitch = DesiredRatePitch - RatePitch; // ...
	ErrorRateYaw   = DesiredRateYaw   - RateYaw;   // ...

	pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll, InputRoll);

	pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch, InputPitch);

	pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw, InputYaw);

}
//********************************************************************************************************

