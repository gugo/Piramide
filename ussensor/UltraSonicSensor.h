#include "Arduino.h"

#define UPDATE_INTERVAL 30000 //in microseconds
#define TRIG_TIME 1000
#define CALLBACK_EARY 5
#define SOS 0.034029




class UltraSonicSensor
{
public:
  UltraSonicSensor(int trigPin, int echoPin);

  float distance;
  void setEchoTime(long val);
  long echoTime;
private:
  int trigPin;
  int echoPin;
};




class UltraSonicSensorManager
{
public:
  UltraSonicSensorManager(
    int trigPin,
    int echoPins[],
    int numSensor,
    void (*callback)()
  );

  static UltraSonicSensorManager* instance(){ return _instance; }

  UltraSonicSensor **sensors;
  void begin();
  void update();

  int trigPin;
  bool trigStatus;
  int *echoPins;
  int numSensor;

  int callBackDelayCount;

  void echoCallBack();

  static UltraSonicSensorManager* _instance;

private:
  void (*_callback)();

  long time;
  long deltaTime;

  long trigTime;
  int echoCounter;





};
