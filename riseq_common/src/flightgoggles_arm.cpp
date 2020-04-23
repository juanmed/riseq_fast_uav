#include <ros/ros.h>
#include <std_msgs/Empty.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "flightgoogles_arming_node");
	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<std_msgs::Empty>("/uav/input/arm", 1);
	std_msgs::Empty msg;

	if(ros::ok())
	{
		ros::Rate rate(10);
		for(int i = 0; i < 10; i++)
		{
			pub.publish(msg);
			rate.sleep();
		}
	}

	return 0;
}