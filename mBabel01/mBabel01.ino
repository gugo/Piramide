#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include "UltraSonicSensor.h"
#include <Bounce2.h>

#define M_COUNT 9
// #define STEPMODE DOUBLE	
#define STEPMODE MICROSTEP
#define NUM_SENSOR 4
#define SPR 200
#define TURN 1000

#define TRIG_PIN 42
#define NUM_SENSOR 4
#define DIST_MIN 20
#define DIST_THRESHOLD 60

int sensorPins[] = {19, 3, 18, 2}; //INTERRUPT PINS
int sensorCounter = 0;
float sensorAverage = 0.0;

int endStopPins[] = {31,33,35,37,39,41,43,45,47}; //Metti tutti pin degli endstops
bool endStopHome[M_COUNT];

// UltraSonicSensorManager *sensors = new UltraSonicSensorManager(TRIG_PIN,sensorPins, NUM_SENSOR, sensorUpdate);

Adafruit_MotorShield S0(0x60);
Adafruit_MotorShield S1(0x61); 
Adafruit_MotorShield S2(0x62);
Adafruit_MotorShield S3(0x63);
Adafruit_MotorShield S4(0x64);

Adafruit_StepperMotor *myStepper01 = S0.getStepper(200, 1);
Adafruit_StepperMotor *myStepper02 = S0.getStepper(200, 2);
Adafruit_StepperMotor *myStepper03 = S1.getStepper(200, 1);
Adafruit_StepperMotor *myStepper04 = S1.getStepper(200, 2);
Adafruit_StepperMotor *myStepper05 = S2.getStepper(200, 1);
Adafruit_StepperMotor *myStepper06 = S2.getStepper(200, 2);
Adafruit_StepperMotor *myStepper07 = S3.getStepper(200, 1);
Adafruit_StepperMotor *myStepper08 = S3.getStepper(200, 2);
Adafruit_StepperMotor *myStepper09 = S4.getStepper(200, 1);

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
};


enum Modes {A, B, C, D, E};
int currentMode = 10;

void initEndStops(){
	for(int i=0; i < M_COUNT; i++ ){
		pinMode(endStopPins[i],INPUT_PULLUP);
	}
}



void setup()
{
	Serial.begin(57600);
	// sensors->begin();
	// Serial.println("<< ULTRASONICSOUND SENSORS >>");

	S0.begin();
	S1.begin();
	S2.begin();
	S3.begin();
	S4.begin();

	// setMode(B);
	
	initEndStops();
	gugoToHome();
	// testRun();
	Serial.println("OUT FROM THE WHILE");
}

void checkMode() {
	// int sensore = sensorCounter;
	// sensorCounter = 0;
	// int mode = sensorCounter;
	// // if (sensorCounter > 0) {

	// // }

	// setMode(mode);

	
}



void loop()
{
	setMode(D);

	// for (int i = 0; i < M_COUNT; i++)
	// {
	// 	Serial.print("button :: ");
	// 	Serial.print(i);
	// 	Serial.print("\t - \t");
	// 	Serial.print("state :: ");
	// 	Serial.print(digitalRead(endStopPins[i]));
	// 	Serial.println(";");
	// }

	// UPDATE MODE
	switch(currentMode){
		case A:
			// Serial.println("LOOP case A");
			animationA();
		break;
		case B:
			// Serial.println("LOOP case B");
			animationB();
		break;
		case C:
			// Serial.println("LOOP case C");
			animationC();
		break;
		case D:
			// Serial.println("LOOP case D");
			animationD();
		break;
		case E:
			// Serial.println("LOOP case E");
			animationE();
		break;
	}
}

void gugoToHome(){

	Bounce debouncer = Bounce();


	for (int i = 0; i < M_COUNT; ++i) {

		endStopHome[i] = true;

		debouncer.attach(endStopPins[i]);
  		debouncer.interval(1);

		steppers[i].setSpeed(400);

		while(endStopHome[i]) {

			Serial.print("WHILE: ");
			Serial.println(i);
			debouncer.update();

			int value = debouncer.read();

			if ( value == HIGH ) {
				steppers[i].runSpeed();
			} else {
				delay(200);
				steppers[i].setCurrentPosition(0);
				steppers[i].setSpeed(1);
				endStopHome[i] = false;
			}
		}
	}
}

void setMode(int mode) {
	if (mode == currentMode) return;
	currentMode = mode;
		switch(currentMode){
		case A:
			Serial.println("SETUP case A");
			initAnimateA();
		break;
		case B:
			Serial.println("SETUP case B");
			initAnimateB();
		break;
		case C:
			Serial.println("SETUP case C");
			initAnimateC();
		break;
		case D:
			Serial.println("SETUP case D");
			initAnimateD();
		break;
		case E:
			Serial.println("SETUP case E");
			initAnimateE();
		break;
		}
}

void testRun() {
	steppers[2].setSpeed(200);
	int run = 0;

		while(run < TURN*20) {
			steppers[2].runSpeed();
			run++;
			Serial.println(run);
            }
    motor[2]->release();
    delay(5000);
}

void initAnimateA() {
	for(int i=0; i<M_COUNT; i++){
		motor[i]->release();
	}
	Serial.println("Init A");
}

void animationA() {
	// for(int i=0; i<M_COUNT; i++){
	//    steppers[i].runSpeed(); 
	// }
}


void initAnimateB(){

	for(int i=0; i<M_COUNT; i++){
		steppers[i].setSpeed(60-(i*4)); 
	}
	Serial.println("Init B");
}

void animationB() {
	
	for(int i=0; i<M_COUNT; i++){
	   steppers[i].runSpeed(); 
	}
}


void initAnimateC(){
	for(int i=0; i<M_COUNT; i++){

		steppers[i].setAcceleration(1.0);
		steppers[i].setMaxSpeed(150);
  		steppers[i].moveTo(SPR*2);
  	}
  	Serial.println("Init C");
}

void animationC() {
	for(int i=0; i<M_COUNT; i++){
		// if (steppers[i].distanceToGo() == 0)
		//  	steppers[i].moveTo(-steppers[i].currentPosition());

	   	steppers[i].run(); 
	}	
}

void initAnimateD(){
	for(int i=0; i<M_COUNT; i++){

  		steppers[i].setAcceleration(10.0);
  		steppers[i].setMaxSpeed(40);
  		if(i%2==0){
			steppers[i].moveTo(TURN); 

		} else{
			steppers[i].moveTo(-TURN); 

		}
	}
	Serial.println("Init D");
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
		steppers[i].setSpeed(40); 
	}
	Serial.println("Init E");
}

void animationE() {
	for(int i=0; i<M_COUNT; i++){
	   steppers[i].runSpeed(); 
	}
}

// void sensorUpdate(){
// sensorCounter = 0;
// sensorAverage = 0;
//   for( int i=0 ; i<NUM_SENSOR; i++ ){
//     Serial.print("s");
//     Serial.print(i);
//     Serial.print(":: ");
//     Serial.print( sensors->sensors[i]->distance );
//     Serial.print("cm\t\t");

//     if(sensors->sensors[i]->distance < DIST_THRESHOLD){
//       sensorCounter++;
//       sensorAverage += sensors->sensors[i]->distance;

//     } 
//   }

//   if(sensorCounter > 0){
//     sensorAverage /= sensorCounter;
//     sensorAverage = ( sensorAverage - DIST_MIN ) / ( DIST_THRESHOLD - DIST_MIN ); 
//     sensorAverage = 1.0 - sensorAverage;
//     sensorAverage = min(sensorAverage, 1);
//     sensorAverage = max(sensorAverage, 0);
//   }else{
//     sensorAverage = 0;
//   }

//   Serial.print(":: ");
//   Serial.print(sensorCounter);
//   Serial.print("\tavrg\t");
//   Serial.println(sensorAverage);
  
// // sensorCounter = 0;
// }




