#include <pozyx_ros/PozyxEstimator.h>
#include "ros/ros.h"

PozyxEstimator::PozyxEstimator(int rate)
	: rate_(rate),
	  new_measurement_(false)
{

}

void PozyxEstimator::run()
{
	ros::Rate loop_rate(rate_);

	while (ros::ok())
	{
		ros::spinOnce();

		if (new_measurement_)
		{
			new_measurement_ = false;
		}

		loop_rate.sleep();
	}
}

void PozyxEstimator::transformInputCbRos(const geometry_msgs::TransformStamped &msg)
{
	new_measurement_ = true;
	raw_transform_ = msg;
	printf("image\n");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "PozyxEstimatorNode");
	ros::NodeHandle private_node_handle_("~");
	ros::NodeHandle n;
	ros::Subscriber transform_input;

	int rate;

	private_node_handle_.param("rate", rate, int(100));

	PozyxEstimator pozyx_estimator(rate);

	transform_input = n.subscribe("eaglered/position", 1, &PozyxEstimator::transformInputCbRos, &pozyx_estimator);

	pozyx_estimator.run();
}