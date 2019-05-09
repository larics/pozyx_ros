#ifndef POZYX_ESTIMATOR
#define POZYX_ESTIMATOR

#include <geometry_msgs/TransformStamped.h>

class PozyxEstimator
{
public:
	PozyxEstimator(int rate);
	void transformInputCbRos(const geometry_msgs::TransformStamped &msg);
	void run(void);
private:
	int rate_;
	bool new_measurement_;

	geometry_msgs::TransformStamped raw_transform_;
};

#endif