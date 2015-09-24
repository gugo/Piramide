#include "UltraSonicSensor.h"

UltraSonicSensorManager *UltraSonicSensorManager::_instance(NULL);

void interruptCallbackA(){
  UltraSonicSensorManager* ussm = UltraSonicSensorManager::instance();
  ussm->echoCallBack(0);
}

void interruptCallbackB(){
  UltraSonicSensorManager* ussm = UltraSonicSensorManager::instance();
  ussm->echoCallBack(1);
}

void interruptCallbackC(){
  UltraSonicSensorManager* ussm = UltraSonicSensorManager::instance();
  ussm->echoCallBack(2);
}

void interruptCallbackD(){
  UltraSonicSensorManager* ussm = UltraSonicSensorManager::instance();
  ussm->echoCallBack(3);
}

UltraSonicSensor::UltraSonicSensor(int trigPin, int echoPin){

  this->trigPin = trigPin;
  this->echoPin = echoPin;

  distance = 500;
}

void UltraSonicSensor::setEchoTime(long val){
  echoTime = val;
  float d = ((float)echoTime * SOS * .5) - 5.0;
  distance += (d-distance) * FILTER_SPEED;
}

UltraSonicSensorManager::UltraSonicSensorManager(
  int trigPin,
  int echoPins[],
  int numSensor,
  void (*callback)()
  )
{

  _instance = this;

  this->trigPin = trigPin;
  this->echoPins = echoPins;
  this->numSensor = numSensor;

  _callback = callback;

  sensors = new UltraSonicSensor*[numSensor];

  for(int i=0; i<numSensor; i++){
    sensors[i] = new UltraSonicSensor(trigPin,echoPins[i]);
  }

  time = micros();
  echoCounter = 0;
  digitalWrite(trigPin, LOW );
  trigStatus = LOW;
  callBackDelayCount = 0;

  interruptFncs[0] = interruptCallbackA;
  interruptFncs[1] = interruptCallbackB;
  interruptFncs[2] = interruptCallbackC;
  interruptFncs[3] = interruptCallbackD;

}

void UltraSonicSensorManager::begin(){
  pinMode(trigPin, OUTPUT);

  for(int i=0;i<numSensor;i++){

    int inter = digitalPinToInterrupt(echoPins[i]);

    attachInterrupt(inter, interruptFncs[i], FALLING);

  }

}

void UltraSonicSensorManager::update(){
  deltaTime = micros() - time;

  if( deltaTime >= UPDATE_INTERVAL){
    time = micros();

    // Serial.print("now::");
    // Serial.print(micros());
    // Serial.print(" tt::");
    // Serial.print(trigTime);
    // Serial.print(" echo::");
    // Serial.println(sensors[0]->echoTime);

    if(callBackDelayCount > CALLBACK_EARY){
      callBackDelayCount = 0;
      _callback();
    }
    callBackDelayCount++;

  } else if( deltaTime >= TRIG_TIME ){

    if(trigStatus == HIGH){
      trigStatus = LOW;
      trigTime = micros();
      digitalWrite(trigPin, LOW );

      // Serial.println("LOW");

    }

  } else if( deltaTime >= 0 ){

    if(trigStatus == LOW){
       trigStatus = HIGH;

      echoCounter = 0;

      digitalWrite(trigPin, HIGH );

      // Serial.println("HI");

    }

  }

}

void UltraSonicSensorManager::echoCallBack(int inx){
  long echoTime = micros() - trigTime;

  sensors[inx]->setEchoTime(echoTime);

  echoCounter++;
}
