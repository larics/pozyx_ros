#ifndef POZYX_ESTIMATOR
#define POZYX_ESTIMATOR

#include <geometry_msgs/TransformStamped.h>
#include "ros/ros.h"
#include <pozyx_ros/KalmanFilter.h>
#include <sensor_msgs/Imu.h>

class PozyxEstimator
{
public:
	PozyxEstimator(int rate);
	void transformInputCbRos(const geometry_msgs::TransformStamped &msg);
	void imucb(const sensor_msgs::Imu &imu);
	void run(void);
private:
	ros::NodeHandle n;
	ros::Subscriber transform_input_, imu_input;
	ros::Publisher transform_estimated_pub_, odometry_estimated_pub_;
	int rate_, seq_;
	bool new_measurement_, start_flag_;

	KalmanFilter kf_x_, kf_y_, kf_z_;

	geometry_msgs::TransformStamped raw_transform_;
};

#endif