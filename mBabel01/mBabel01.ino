#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include "UltraSonicSensor.h"

#define M_COUNT 13
#define STEPMODE DOUBLE
#define NUM_SENSOR 4
#define SPR 200

#define TRIG_PIN 42
#define NUM_SENSOR 4
#define DIST_MIN 20
#define DIST_THRESHOLD 40

int sensorPins[] = {18,  19,  2,  3}; //INTERRUPT PINS
int sensorCounter = 0;
float sensorAverage = 0.0;

UltraSonicSensorManager *sensors = new UltraSonicSensorManager(TRIG_PIN,sensorPins, NUM_SENSOR, sensorUpdate);

Adafruit_MotorShield S0(0x60);
Adafruit_MotorShield S1(0x61); 
Adafruit_MotorShield S2(0x62);
Adafruit_MotorShield S3(0x63);
Adafruit_MotorShield S4(0x64);
Adafruit_MotorShield S5(0x65);
Adafruit_MotorShield S6(0x66);

Adafruit_StepperMotor *myStepper01 = S0.getStepper(200, 1);
Adafruit_StepperMotor *myStepper02 = S0.getStepper(200, 2);
Adafruit_StepperMotor *myStepper03 = S1.getStepper(200, 1);
Adafruit_StepperMotor *myStepper04 = S1.getStepper(200, 2);
Adafruit_StepperMotor *myStepper05 = S2.getStepper(200, 1);
Adafruit_StepperMotor *myStepper06 = S2.getStepper(200, 2);
Adafruit_StepperMotor *myStepper07 = S3.getStepper(200, 1);
Adafruit_StepperMotor *myStepper08 = S3.getStepper(200, 2);
Adafruit_StepperMotor *myStepper09 = S4.getStepper(200, 1);
Adafruit_StepperMotor *myStepper10 = S4.getStepper(200, 2);
Adafruit_StepperMotor *myStepper11 = S5.getStepper(200, 1);
Adafruit_StepperMotor *myStepper12 = S5.getStepper(200, 2);
Adafruit_StepperMotor *myStepper13 = S6.getStepper(200, 1);

Adafruit_StepperMotor *motor[M_COUNT] = 
{
	myStepper01,
	myStepper02,
	myStepper03,
	myStepper04,
	myStepper05,
	myStepper06,
	myStepper07,
	myStepper08,
	myStepper09,
	myStepper10,
	myStepper11,
	myStepper12,
	myStepper13
};



AccelStepper stepper01(motor[0], STEPMODE);
AccelStepper stepper02(motor[1], STEPMODE);
AccelStepper stepper03(motor[2], STEPMODE);
AccelStepper stepper04(motor[3], STEPMODE);
AccelStepper stepper05(motor[4], STEPMODE);
AccelStepper stepper06(motor[5], STEPMODE);
AccelStepper stepper07(motor[6], STEPMODE);
AccelStepper stepper08(motor[7], STEPMODE);
AccelStepper stepper09(motor[8], STEPMODE);
AccelStepper stepper10(motor[9], STEPMODE);
AccelStepper stepper11(motor[10], STEPMODE);
AccelStepper stepper12(motor[11], STEPMODE);
AccelStepper stepper13(motor[12], STEPMODE);

AccelStepper steppers[M_COUNT] = 
{
	stepper01, 
	stepper02, 
	stepper03, 
	stepper04, 
	stepper05, 
	stepper06, 
	stepper07, 
	stepper08, 
	stepper09, 
	stepper10, 
	stepper11, 
	stepper12, 
	stepper13
};


enum Modes {A, B, C, D, E};
int currentMode = 10;

void checkMode() {
	int mode = sensorCounter;

	setMode(mode);
}

void setup()
{
	// pinMode(18, INPUT);
	// pinMode(19, INPUT);
	// pinMode(20, INPUT);
	// pinMode(21, INPUT);
	// pinMode(42, OUTPUT);
	Serial.begin(57600);
	sensors->begin();
	Serial.println("<< ULTRASONICSOUND SENSORS >>");

	S0.begin();
	S1.begin();
	S2.begin();
	S3.begin();
	S4.begin();
	S5.begin();
	S6.begin();

	setMode(A);
}

void loop()
{
	sensors->update();
	checkMode();


	//UPDATE MODE
	switch(currentMode){
		case A:
			// Serial.println("LOOP case A");
			animationA();
		break;
		case B:
			animationB();
		break;
		case C:
			animationC();
		break;
		case D:
			animationD();
		break;
		case E:
			animationE();
		break;
	}
}

void setMode(int mode) {
	if (mode == currentMode) return;
	currentMode = mode;

	switch(currentMode){
		case A:
			// Serial.println("SETUP case A");
			initAnimateA();
		break;
		case B:
			initAnimateB();
		break;
		case C:
			initAnimateC();
		break;
		case D:
			initAnimateD();
		break;
		case E:
			initAnimateE();
		break;
	}
}

void sensorUpdate(){
sensorCounter = 0;
sensorAverage = 0;
  for( int i=0 ; i<NUM_SENSOR; i++ ){
    Serial.print("s");
    Serial.print(i);
    Serial.print(":: ");
    Serial.print( sensors->sensors[i]->distance );
    Serial.print("cm\t\t");

    if(sensors->sensors[i]->distance < DIST_THRESHOLD){
      sensorCounter++;
      sensorAverage += sensors->sensors[i]->distance;

    }
  }

  if(sensorCounter > 0){
    sensorAverage /= sensorCounter;
    sensorAverage = ( sensorAverage - DIST_MIN ) / ( DIST_THRESHOLD - DIST_MIN ); 
    sensorAverage = 1.0 - sensorAverage;
    sensorAverage = min(sensorAverage, 1);
    sensorAverage = max(sensorAverage, 0);
  }else{
    sensorAverage = 0;
  }

  Serial.print(":: ");
  Serial.print(sensorCounter);
  Serial.print("\tavrg\t");
  Serial.println(sensorAverage);
  

}

void initAnimateA() {}

void animationA() {}


void initAnimateB(){
	// steppers[0].setSpeed(60);
	// Serial.println("initAnimateA");

	for(int i=0; i<M_COUNT; i++){
		steppers[i].setSpeed(130 - i*8); 
	}

}

void animationB() {
	
	for(int i=0; i<M_COUNT; i++){
	   steppers[i].runSpeed(); 
	}

	// steppers[1].runSpeed(); 
}


void initAnimateC(){
	for(int i=0; i<M_COUNT; i++){

		steppers[i].setMaxSpeed(130.0 - i*5);
  		steppers[i].setAcceleration(100.0);
  		steppers[i].moveTo(SPR*2);
	}
}

void animationC() {
	for(int i=0; i<M_COUNT; i++){
		if (steppers[i].distanceToGo() == 0)
		 	steppers[i].moveTo(-steppers[i].currentPosition());

	   	steppers[i].run(); 
	}	
}

void initAnimateD(){
	for(int i=0; i<M_COUNT; i++){

		steppers[i].setMaxSpeed(100 - i*3);
  		steppers[i].setAcceleration(100.0);
  		steppers[i].moveTo(SPR*1);
	}
}

void animationD() {
	for(int i=0; i<M_COUNT; i++){
		if (steppers[i].distanceToGo() == 0)
		 	steppers[i].moveTo(-steppers[i].currentPosition());

	   	steppers[i].run(); 
	}	
}

void initAnimateE(){
	for(int i=0; i<M_COUNT; i++){
		steppers[i].setSpeed(130 - i*8); 
	}
}

void animationE() {
	for(int i=0; i<M_COUNT; i++){
	   steppers[i].runSpeed(); 
	}
}




