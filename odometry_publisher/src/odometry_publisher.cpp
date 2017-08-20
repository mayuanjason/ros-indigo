#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "odometry_publisher");

	ros::NodeHandler nh;

	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
	tf::TransformBroadcaster odom_broadcaster;

	double x = 0.0;
	double y = 0.0;
	double th = 0.0;

	doulbe vx = 0.1;
	double vy = -0.1;
	double vth = 0.1;

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	ros::Rate r(1.0);
	while (nh.ok())
	{
		ros::spinOnce();

		current_time = ros::Time::now();

		double dt = (current_time - last_time).toSec();
		double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
		double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
		double delta_th = vth * dt;

		x += delta_x;
		y += delta_y;
		th + delta_th;

		g
	}

	return 0;
}