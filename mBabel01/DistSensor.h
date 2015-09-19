#include <NewPing.h>

#define TRIGGER_PIN  12
#define ECHO_PIN     11
#define MAX_DISTANCE 200

class DistSensor {
public: 
	DistSensor();
	bool inRange;
};