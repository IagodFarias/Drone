// ESP32 Guide: https://randomnerdtutorials.com/esp32-mpu-6050-accelerometer-gyroscope-arduino/
// ESP8266 Guide: https://randomnerdtutorials.com/esp8266-nodemcu-mpu-6050-accelerometer-gyroscope-arduino/
// Arduino Guide: https://randomnerdtutorials.com/arduino-mpu-6050-accelerometer-gyroscope/
#include "Drone.h"
#include "esp32-hal-ledc.h"


//*******************************ORGANIZAÇÃO DAS VARIÁVEIS NECESSÁRIAS******************************************
int MotorVeloci1, MotorVeloci2, MotorVeloci3, MotorVeloci4; // Variáveis de controle da velocidade dos motores

int CH1, CH2, CH3, CH4; // Variáveis que receberam os estado do controle remoto

int ThrottleIdle   = 1180; // Valor mínimo para velocidade dos motores durante o voo
int ThrottleCutOff = 1000; // Valor para parar os motores

int count = 0; // para que é usado?
int Timer1 = 0, Timer2 = 0; // para que é usado?

float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw; // Variáveis de controle do sensor giroscópio...
float AceX, AceY, AceZ, Temp; 										 // ...
float GyrXc = 0; 													 // ...
float GyrYc = 0; 													 // ...
float GyrZc = 0; 													 // ...
float AngleRoll, AnglePitch; 										 // ...

bool calibration = false; // Variável de controle do estado de calibração

float KalmanGain; 											   // Variáveis de controle do filtro Kalman...
float KalmanAngleRoll  = 0, KalmanUncertaintyAngleRoll  = 2*2; //...
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2*2; //...
float Kalman1DOutput[] = {0,0}; 							   // Saída do filtro de Kalman

uint32_t LoopTimer; // Definição do timer que vai controlar a freq de controle do motor

float RateRoll         , RatePitch         , RateYaw;                  // Variáveis de controle do controlador PID interno...
float InputRoll        , InputThrottle     , InputPitch    , InputYaw; // baseado na taxa de variação angular...
float DesiredRateRoll  , DesiredRatePitch  , DesiredRateYaw;		   // ...
float ErrorRateRoll = 0, ErrorRatePitch = 0, ErrorRateYaw = 0;		   // ...


float PrevErrorRateRoll = 0, PrevErrorRatePitch = 0, PrevErrorRateYaw = 0; // Variáveis de armazenamento do erro anterior para  PID "RATE MODE" ...
float PrevItermRateRoll = 0, PrevItermRatePitch = 0, PrevItermRateYaw = 0; // ...

float PIDReturn[] = {0, 0, 0}; // Saída da função PID

float PRateRoll = 0.6,  PRatePitch = PRateRoll, PRateYaw  = 2;  // Definição das variáveis necessárias para o...
float IRateRoll = 3.5,  IRatePitch = IRateRoll, IRateYaw  = 12; // controlador PID "ANGLE MODE", junto com os valores de...
float DRateRoll = 0.03, DRatePitch = DRateRoll, DRateYaw  = 0;   // P, I e D, para o ajuste do controlador

float DesiredAngleRoll, DesiredAnglePitch;     // Variáveis de angulo desejado para PID "ANGLE MODE"...
float ErrorAngleRoll  , ErrorAnglePitch;       // Variáveis de erro...
float PrevErrorAngleRoll = 0, PrevErrorAnglePitch = 0; // Variáveis de erro prévio...
float PrevItermAngleRoll = 0, PrevItermAnglePitch = 0; // ...

float PAngleRoll = 2, PAnglePitch = PAngleRoll; // Constantes P, I e D para configuração do controledor...
float IAngleRoll = 0, IAnglePitch = IAngleRoll; // ...
float DAngleRoll = 0, DAnglePitch = DAngleRoll; // ...

//***********************************FILTRO KALMAN DE 1 DIMENSÃO******************************************
void Drone::Kalman1D(float KalmanState,float KalmanUncertainty,float KalmanInput,float KalmanMeasurement){

	KalmanState       = KalmanState + 0.004*KalmanInput;
	KalmanUncertainty = KalmanUncertainty + 0.004*0.004*4*4;
	KalmanGain        = KalmanUncertainty*1/(1*KalmanUncertainty + 3*3);
	KalmanState       = KalmanState + KalmanGain*(KalmanMeasurement - KalmanState);
	KalmanUncertainty = (1 - KalmanGain)*KalmanUncertainty;
	
	Kalman1DOutput[0] = KalmanState;
	Kalman1DOutput[1] = KalmanUncertainty;
}
//********************************FUNÇÃO GERAL PARA O CONTROLADOR PID*************************************
void Drone::pid_equation(float Error, float P , float I, float D, float PrevError, float PrevIterm){

	float Pterm = P*Error;
	float Iterm = PrevIterm + I*(Error + PrevError)*0.004/2;

	if      (Iterm >  400) Iterm = 400;
	else if (Iterm < -400) Iterm = -400;

	float Dterm     = D*(Error - PrevError)/0.004;
	float PIDOutput = Pterm + Iterm + Dterm;

	if      (PIDOutput >  400) PIDOutput =  400;
	else if (PIDOutput < -400) PIDOutput = -400;

	PIDReturn[0] = PIDOutput;
	PIDReturn[1] = Error;
	PIDReturn[2] = Iterm;
}
//********************************************************************************************************
void Drone::MainControlSetup(int serial, int pin1, int pin2, int pin3, int pin4){
	
	Serial.begin(serial);
	readPWMSetup(pin1);
	readPWMSetup(pin2);
	readPWMSetup(pin3);
	readPWMSetup(pin4);

	MPUconfigSetup();
	CalibrarMPU();
}



// FUNÇÃO DE CONTROLE DE VOO DO DRONE

//********************************************************************************************************
void Drone::MainControlLoop(int pinPWM_Read1, int pinPWM_Read2, int pinPWM_Read3, int pinPWM_Read4){
	
	// RateRoll  -= RateCalibrationRoll;  // Atualização da taxa de variação angular com... 
	// RatePitch -= RateCalibrationPitch; // os valores vindos do sensor giroscópio depois...
 	// RateYaw   -= RateCalibrationYaw;   // de passada a calibração

	//##############################FILTRO DE KALMAN############################
	Kalman1D(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);     // chamada da função do filtro afim...

	KalmanAngleRoll             = Kalman1DOutput[0];                                // de obter o ângulo sem ruído
	KalmanUncertaintyAngleRoll  = Kalman1DOutput[1];                                // ...
  
  	Kalman1D(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch); // ...

	KalmanAnglePitch            = Kalman1DOutput[0];                                // ...
	KalmanUncertaintyAnglePitch = Kalman1DOutput[1];                                // ...

	//##############################LEITURA DO CONTROLE############################
	CH1 = readPWMLoop(pinPWM_Read1); // obtenção dos comandos vindos do controle remoto...
	CH2 = readPWMLoop(pinPWM_Read2); // ...
	CH3 = readPWMLoop(pinPWM_Read3); // ...
	CH4 = readPWMLoop(pinPWM_Read4); // ...

	DesiredAngleRoll  = 0.10*(CH1 - 1500); // Valores desejados de ângulo obtidos...
	DesiredAnglePitch = 0.10*(CH2 - 1500); // a partir do controle remoto

	InputThrottle     = CH3; 			   // Potencia dos motores para movimento vertical
	DesiredRateYaw    = 0.15*(CH4 - 1500); // Valor do ângulo para giro no eixo Z

	ErrorAngleRoll  = DesiredAngleRoll  - KalmanAngleRoll; 	// Calculo do erro de ângulo Roll e Pitch...
	ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch; // ...

	//##############################APLICAÇÃO DO PID############################
	pid_equation(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll, PrevItermAngleRoll); 

	DesiredRateRoll        = PIDReturn[0]; // Chamada da função PID para obtenção da taxa de variação angular desejada...
	PrevErrorAngleRoll     = PIDReturn[1]; // ... 
	PrevItermAngleRoll     = PIDReturn[2]; // ...

	pid_equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch);

	DesiredRatePitch       = PIDReturn[0]; // ... 
	PrevErrorAnglePitch    = PIDReturn[1]; // ...
	PrevItermAnglePitch    = PIDReturn[2]; // ...

	ErrorRateRoll  = DesiredRateRoll  - RateRoll;  // Cálculo dos erros de roll, pitch e yaw...
	ErrorRatePitch = DesiredRatePitch - RatePitch; // ...
	ErrorRateYaw   = DesiredRateYaw   - RateYaw;   // ...

	pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);

	InputRoll          = PIDReturn[0]; // Chamada da função PID para obtenção do ângulo de roll...
	PrevErrorRateRoll  = PIDReturn[1]; // ...
	PrevItermRateRoll  = PIDReturn[2]; // ...

	pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);

	InputPitch         = PIDReturn[0]; // Chamada da função PID para obtenção do ângulo de pitch...
	PrevErrorRatePitch = PIDReturn[1]; // ...
	PrevItermRatePitch = PIDReturn[2]; // ...

	pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);

	InputYaw           = PIDReturn[0]; // Chamada da função PID para obtenção do ângulo de yaw...
	PrevErrorRateYaw   = PIDReturn[1]; // ... 
	PrevItermRateYaw   = PIDReturn[2]; // ...

	//##############################CONTROLE DOS MOTORES############################
	if (InputThrottle > 1800) InputThrottle = 1800; // Medida de segurança para manter a potência máxima de subida a 80% 

	MotorVeloci1 = 1.024*(InputThrottle - InputRoll - InputPitch - InputYaw); // Cálculo da entrada para os motores a partir...
	MotorVeloci2 = 1.024*(InputThrottle - InputRoll + InputPitch + InputYaw); // dos inputs vindos do controle remoto que... 
	MotorVeloci3 = 1.024*(InputThrottle + InputRoll + InputPitch - InputYaw); // passaram pela função PID...
	MotorVeloci4 = 1.024*(InputThrottle + InputRoll - InputPitch + InputYaw); // ...

	Serial.print("Motor1:");   //Print dos valores para visualização do input aos motores
	Serial.print(MotorVeloci1);
	Serial.print(",");
	Serial.print("Motor2:");
	Serial.print(MotorVeloci2);
	Serial.print(",");
	Serial.print("Motor3:");
	Serial.print(MotorVeloci3);
	Serial.print(",");
	Serial.print("Motor4:");
	Serial.println(MotorVeloci4);
	
	//##############################MEDIDAS DE SEGURANÇA############################
	if (MotorVeloci1 > 2000) MotorVeloci1 = 1999; // Medida de segurança para evitar extrapolação da potência máxima dos motores
	if (MotorVeloci2 > 2000) MotorVeloci2 = 1999; // ... 
	if (MotorVeloci3 > 2000) MotorVeloci3 = 1999; // ... 
	if (MotorVeloci4 > 2000) MotorVeloci4 = 1999; // ...
	
	// if (MotorVeloci1 < ThrottleIdle) MotorVeloci1 = ThrottleIdle; // Medida de segurança para manter o drone voando com...
	// if (MotorVeloci2 < ThrottleIdle) MotorVeloci2 = ThrottleIdle; // uma potência mínima...
	// if (MotorVeloci3 < ThrottleIdle) MotorVeloci3 = ThrottleIdle; // ...
	// if (MotorVeloci4 < ThrottleIdle) MotorVeloci4 = ThrottleIdle; // ...

	if (CH3 < 1050) { // Medida de segurança para ter certeza do desligamento dos motores
		MotorVeloci1 = ThrottleCutOff; 
		MotorVeloci2 = ThrottleCutOff;
		MotorVeloci3 = ThrottleCutOff; 
		MotorVeloci4 = ThrottleCutOff;
		reset_pid(); // Toda vez que o drone pousar o PID precisa ser resetado
	}

	while (micros() - LoopTimer < 4000); // tempo de espera para novo loop de controle... 
	LoopTimer = micros(); 				 // a frequência usada é de 250Hz
}
//********************************************************************************************************
void Drone::reset_pid() {

	PrevErrorRateRoll = 0; PrevErrorRatePitch = 0; PrevErrorRateYaw = 0;
	PrevItermRateRoll = 0; PrevItermRatePitch = 0; PrevItermRateYaw = 0;	

	PrevErrorAngleRoll = 0; PrevErrorAnglePitch = 0; 
	PrevItermAngleRoll = 0; PrevItermAnglePitch = 0;
}
//********************************************************************************************************
void Drone::MPUconfigSetup() {
	
	if (!mpu.begin()) {
		
		while (1) {
			Serial.println("error");
	  		yield();
		}
	}
  
	mpu.setAccelerometerRange(MPU6050_RANGE_8_G);//2G, 4G, 8G, 16G
	mpu.setGyroRange(MPU6050_RANGE_500_DEG);     //250deg/s, 500deg/s, 1000deg/s, 2000deg/s
	mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);  //5Hz, 10Hz, 21Hz, 44Hz, 94Hz, 184Hz, 260Hz
}
//********************************************************************************************************
void Drone::CalibrarMPU(){

 	for (int Passo = 0 ; Passo < 2000; Passo++) {

		Serial.print("Calibrando: ");
		Serial.println(Passo);

		MPUgetSignalsLoop();

		RateCalibrationRoll  += RateRoll;
		RateCalibrationPitch += RatePitch;
		RateCalibrationYaw   += RateYaw;
		delay(1);
	}
	
	calibration = true;
}
//********************************************************************************************************
void Drone::MPUgetSignalsLoop() {
	
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
	RateRoll  -= (RateCalibrationRoll/2000)  + 8;
	RatePitch -= (RateCalibrationPitch/2000) + 5.8;
	RateYaw   -= (RateCalibrationYaw/2000)   - 0.9;
	AceX -= 0.02;
	AceY -= 0.03;
	AceZ -= 0.13;
  }
}
//********************************************************************************************************
void Drone::DisplaySerialMpuData(){	
	
	if ((millis() - Timer1) >= 800){
		Timer1 = millis();
		Serial.print("AceX: ");
		Serial.print(AceX);
		Serial.print(" ");
		Serial.print("AceY: ");
		Serial.print(AceY);
		Serial.print(" ");
		Serial.print("AceZ: ");
		Serial.print(AceZ);
		Serial.print(" ");
		
		Serial.println(" ");
		
		Serial.print("RateRoll: ");
		Serial.print(RateRoll);
		Serial.print(" ");
		Serial.print("RatePitch: ");
		Serial.print(RatePitch);
		Serial.print(" ");
		Serial.print("RateYaw: ");
		Serial.print(RateYaw);
		
		Serial.println(" ");
		Serial.println(" ");
		
		Serial.print("Angulo Roll: ");
		Serial.print(AngleRoll);
		
		Serial.println(" ");
		
		Serial.print("Angulo Pitch: ");
		Serial.print(AnglePitch);
		
		Serial.println(" ");
	}
	
	
}
//********************************************************************************************************
void Drone::DisplayPlotterMpuData(){

	// Kalman1D(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);     // chamada da função do filtro afim...

	// KalmanAngleRoll             = Kalman1DOutput[0];                                // de obter o ângulo sem ruído
	// KalmanUncertaintyAngleRoll  = Kalman1DOutput[1];                                // ...
  
  	// Kalman1D(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch); // ...

	// KalmanAnglePitch            = Kalman1DOutput[0];                                // ...
	// KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

	if ((millis() - Timer2) >= 10){
		Timer2 = millis();
		Serial.print("Angle_Roll:");
		Serial.print(AngleRoll);
		Serial.print(",");
		Serial.print("Angle_Pitch:");
		Serial.print(AnglePitch);
		Serial.print(",");
		Serial.print("Kalman_Angle_Roll:");
		Serial.print(KalmanAngleRoll);
		Serial.print(",");
		Serial.print("Kalman_Angle_Pitch:");
		Serial.println(KalmanAnglePitch);
		
		//Serial.println(GyrZ);
	}
}
//********************************************************************************************************
void Drone::setupPWM(int freq, int resolution, int pin, int ch){
  pinMode(pin, OUTPUT); // Definição do pino de saída do PWM de controle do motor

  ledcAttachPin(pin, ch); // Funções para definição do PWM na ESP32
  ledcSetup(ch, freq, resolution);// ...
}
//********************************************************************************************************
// Função para controlar a velocidade dos motores
// min_sped=64 max_sped=127 para resolução = 8 bits
void Drone::controlSpeed(int speed, int ch){
	ledcWrite(ch, speed); // Função para mudança do PWM na ESP32
}
int Drone::mot1(){
	return MotorVeloci1;
}
//********************************************************************************************************
// Função para adequar os pinos que vão receber o PWM do controlador
void Drone::readPWMSetup(int pin1){
	pinMode(pin1, INPUT);
}
//********************************************************************************************************
// Função para ler o valor do PWM enviado pelo controlador e printa-lo no monitor serial
// Precisa ser usada para cada valor que precisar ser lido, logo no máximo 4 vezes
void Drone::readPWMLoop_SM(int pinX, int ch){

	Serial.print("CH");
	Serial.print(ch);	
	Serial.print(" = ");	
	Serial.println(pulseIn(pinX, HIGH));
	Serial.println(" ");
}
//********************************************************************************************************
int Drone::readPWMLoop(int pinX){

	return pulseIn(pinX, HIGH); 
}
//********************************************************************************************************
