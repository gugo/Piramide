#include "UltraSonicSensor.h"

#define TRIG_PIN 9
#define NUM_SENSOR 4
int sensorPins[] = {18,  19,  20,  21}; //INTERRUPT PINS

UltraSonicSensorManager *sensors = new UltraSonicSensorManager(TRIG_PIN,sensorPins, NUM_SENSOR, sensorUpdate);

void setup() {
  Serial.begin(57600);
  sensors->begin();
  Serial.println("<< ULTRASONICSOUND SENSORS >>");
}

void loop() {
  sensors->update();
}

void sensorUpdate(){

  for( int i=0 ; i<NUM_SENSOR; i++ ){
    Serial.print("s");
    Serial.print(i);
    Serial.print(":: ");
    Serial.print( sensors->sensors[i]->distance );
    Serial.print("cm\t\t");
  }

  Serial.println("");

}
