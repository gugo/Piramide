#define RANGE_DISTANCE 50

class DistSensor {
public: 
	DistSensor();
	bool inRange;

private:
	bool getState();

	void update();
};