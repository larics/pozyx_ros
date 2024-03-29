#include <pozyx_ros/PozyxEstimator.h>
#include <nav_msgs/Odometry.h>

PozyxEstimator::PozyxEstimator(int rate)
	: rate_(rate),
	  new_measurement_(false),
	  start_flag_(false),
	  seq_(0)
{
	transform_input_ = n.subscribe("pozyx/measured", 1, &PozyxEstimator::transformInputCbRos, this);
	//imu_input = n.subscribe("/euroc3/imu", 1, &PozyxEstimator::imucb, this);
	transform_estimated_pub_ = n.advertise<geometry_msgs::TransformStamped>("pozyx/estimated_transform", 1);
	odometry_estimated_pub_ = n.advertise<nav_msgs::Odometry>("pozyx/estimated_odometry", 1);
	//imu_pub_ = n.advertise<sensor_msgs::Imu>("imu", 1);

	kf_x_.setMeasureNoise(20.0);
	kf_y_.setMeasureNoise(20.0);
	kf_z_.setMeasureNoise(20.0);

	kf_x_.setPositionNoise(0.1);
	kf_y_.setPositionNoise(0.1);
	kf_z_.setPositionNoise(0.1);

	kf_x_.setVelocityNoise(1.0);
	kf_y_.setVelocityNoise(1.0);
	kf_z_.setVelocityNoise(1.0);
}

/*void PozyxEstimator::imucb(const sensor_msgs::Imu &imu)
{
	sensor_msgs::Imu imu1 = imu;
	imu1.header.stamp = ros::Time::now();
	imu_pub_.publish(imu1);
}*/

void PozyxEstimator::run()
{
	ros::Rate loop_rate(rate_);

	while (ros::ok())
	{
		ros::spinOnce();

		kf_x_.modelUpdate(1.0/rate_);
		kf_y_.modelUpdate(1.0/rate_);
		kf_z_.modelUpdate(1.0/rate_);

		if (new_measurement_)
		{
			new_measurement_ = false;
			kf_x_.measureUpdate(raw_transform_.transform.translation.x);
			kf_y_.measureUpdate(raw_transform_.transform.translation.y);
			kf_z_.measureUpdate(raw_transform_.transform.translation.z);
		}

		geometry_msgs::TransformStamped transform_estimated;

		transform_estimated.header.stamp = ros::Time::now();
		transform_estimated.header.frame_id = raw_transform_.header.frame_id;
		transform_estimated.header.seq = ++seq_; 
		transform_estimated.child_frame_id = raw_transform_.child_frame_id;

		transform_estimated.transform.translation.x = kf_x_.getPosition();
		transform_estimated.transform.translation.y = kf_y_.getPosition();
		transform_estimated.transform.translation.z = kf_z_.getPosition();

		transform_estimated.transform.rotation.x = raw_transform_.transform.rotation.x;
		transform_estimated.transform.rotation.y = raw_transform_.transform.rotation.y;
		transform_estimated.transform.rotation.z = raw_transform_.transform.rotation.z;
		transform_estimated.transform.rotation.w = raw_transform_.transform.rotation.w;

		nav_msgs::Odometry odometry_estimated;

		odometry_estimated.header = transform_estimated.header;
		odometry_estimated.child_frame_id = raw_transform_.child_frame_id;

		odometry_estimated.pose.pose.position.x = kf_x_.getPosition();
		odometry_estimated.pose.pose.position.y = kf_y_.getPosition();
		odometry_estimated.pose.pose.position.z = kf_z_.getPosition();
		odometry_estimated.pose.pose.orientation.x = raw_transform_.transform.rotation.x;
		odometry_estimated.pose.pose.orientation.y = raw_transform_.transform.rotation.y;
		odometry_estimated.pose.pose.orientation.z = raw_transform_.transform.rotation.z;
		odometry_estimated.pose.pose.orientation.w = raw_transform_.transform.rotation.w;

		odometry_estimated.twist.twist.linear.x = kf_x_.getVelocity();
		odometry_estimated.twist.twist.linear.y = kf_y_.getVelocity();
		odometry_estimated.twist.twist.linear.z = kf_z_.getVelocity();

		transform_estimated_pub_.publish(transform_estimated);
		odometry_estimated_pub_.publish(odometry_estimated);
		loop_rate.sleep();
	}
}

void PozyxEstimator::transformInputCbRos(const geometry_msgs::TransformStamped &msg)
{
	if (!start_flag_)
	{
		start_flag_ = true;
		kf_x_.initializePosition(msg.transform.translation.x);
		kf_y_.initializePosition(msg.transform.translation.y);
		kf_z_.initializePosition(msg.transform.translation.z);
	}
	new_measurement_ = true;
	raw_transform_ = msg;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "PozyxEstimatorNode");
	ros::NodeHandle private_node_handle_("~");
	ros::Subscriber transform_input;

	int rate;

	private_node_handle_.param("rate", rate, int(100));

	PozyxEstimator pozyx_estimator(rate);
	pozyx_estimator.run();
}
