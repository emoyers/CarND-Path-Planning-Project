#include <vector>

class Vehicle{

public:

	double car_x;
	double car_y;
	double car_s;
	double car_d;
	double car_yaw;
	double car_speed;
	double ref_velocity;
	int actual_lane;

	Vehicle();

	Vehicle(double ref_vel, int lane);

	virtual ~Vehicle();


	void set_parameters(double x, double y, double s, double d, double yaw, double speed);
};