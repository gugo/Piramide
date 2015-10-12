#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include "UltraSonicSensor.h"
#include <Bounce2.h>

#define M_COUNT 4
#define STEPMODE DOUBLE
#define NUM_SENSOR 4
#define SPR 200

#define TRIG_PIN 42
#define NUM_SENSOR 4
#define DIST_MIN 20
#define DIST_THRESHOLD 60

int sensorPins[] = {19, 3, 18, 2}; //INTERRUPT PINS
int sensorCounter = 0;
float sensorAverage = 0.0;

int endStopPins[] = {31,33,35,37}; //Metti tutti pin degli endstops
bool endStopHome[M_COUNT];

UltraSonicSensorManager *sensors = new UltraSonicSensorManager(TRIG_PIN,sensorPins, NUM_SENSOR, sensorUpdate);

Adafruit_MotorShield S5(0x65);
Adafruit_MotorShield S6(0x66);

Adafruit_StepperMotor *myStepper01 = S5.getStepper(200, 1);
Adafruit_StepperMotor *myStepper02 = S5.getStepper(200, 2);
Adafruit_StepperMotor *myStepper03 = S6.getStepper(200, 1);
Adafruit_StepperMotor *myStepper04 = S6.getStepper(200, 2);


Adafruit_StepperMotor *motor[M_COUNT] = 
{
	myStepper01,
	myStepper02,
	myStepper03,
	myStepper04,
};

AccelStepper stepper01(motor[0], STEPMODE);
AccelStepper stepper02(motor[1], STEPMODE);
AccelStepper stepper03(motor[2], STEPMODE);
AccelStepper stepper04(motor[3], STEPMODE);


AccelStepper steppers[M_COUNT] = 
{
	stepper01, 
	stepper02, 
	stepper03, 
	stepper04, 
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
	sensors->begin();
	Serial.println("<< ULTRASONICSOUND SENSORS >>");

	S5.begin();
	S6.begin();


	initEndStops();
	// gotoHome();
	gugoToHome();
	Serial.println("OUT FROM THE WHILE");

	// setMode(A);
	
}

void loop()
{
	// Serial.print(sensorCounter);
	// sensors->update();
	// setMode(sensorCounter);
	setMode(C);
	
	// Serial.println(digitalRead(endStopPins[0]));

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
  		debouncer.interval(5);

		steppers[i].setSpeed(40);

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
		steppers[i].setSpeed(400); 
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

		steppers[i].setMaxSpeed(150);
  		steppers[i].setAcceleration(100.0);
  		if(i%2==0){
			steppers[i].moveTo(SPR*20); 

		} else{
			steppers[i].moveTo(-SPR*20); 

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
  
// sensorCounter = 0;
}

