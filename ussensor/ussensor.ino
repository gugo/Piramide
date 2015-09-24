#include "UltraSonicSensor.h"

#define TRIG_PIN 42
#define NUM_SENSOR 4
#define DIST_MIN 20
#define DIST_THRESHOLD 40


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

  int counter = 0;
  float average = 0.0;

  for( int i=0 ; i<NUM_SENSOR; i++ ){
    Serial.print("s");
    Serial.print(i);
    Serial.print(":: ");
    Serial.print( sensors->sensors[i]->distance );
    Serial.print("cm\t\t");

    if(sensors->sensors[i]->distance < DIST_THRESHOLD){
      counter++;
      average += sensors->sensors[i]->distance;

    }
  }

  if(counter > 0){
    average /= counter;
    average = ( average - DIST_MIN ) / ( DIST_THRESHOLD - DIST_MIN ); 
    average = 1.0 - average;
    average = min(average, 1);
    average = max(average, 0);
  }else{
    average = 0;
  }

  // if(counter > 0){
  //   average = ( average - DIST_MIN ) / ( DIST_THRESHOLD - DIST_MIN );
  // } else {
  //   average = 1;
  // }
  // average /= NUM_SENSOR;
   
  // average = 1.0 - average;
  // average = min(average, 1);
  // average = max(average, 0);

  Serial.print(":: ");
  Serial.print(counter);
  Serial.print("\tavrg\t");
  Serial.println(average);
  

}
