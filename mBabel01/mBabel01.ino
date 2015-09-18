#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

#define M_COUNT 13
#define STEPMODE DOUBLE

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

void setup()
{
	S0.begin();
	S1.begin();
	S2.begin();
	S3.begin();
	S4.begin();
	S5.begin();
	S6.begin();


	for(int i=0; i<M_COUNT; i++){
	   steppers[i].setSpeed(200 - i*20); 
	}

}

void loop()
{
	for(int i=0; i<M_COUNT; i++){
	   steppers[i].runSpeed(); 
	}

}

