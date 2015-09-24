#include "UltraSonicSensor.h"

#define TRIG_PIN 9
#define NUM_SENSOR 4
int sensorPins[] = {2,  20,  19,  18}; //INTERRUPT PINS

UltraSonicSensorManager *sensors = new UltraSonicSensorManager(TRIG_PIN,sensorPins, NUM_SENSOR, sensorUpdate);

void setup() {
  Serial.begin(57600);
  sensors->begin();
}

void loop() {
  sensors->update();
}

void sensorUpdate(){
  Serial.print("s0 cm:: ");
  Serial.println( sensors->sensors[0]->distance );

}
