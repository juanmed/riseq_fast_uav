#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <mav_msgs/Actuators.h>
#include <riseq_msgs/FlatTrajectory.h>

bool trajectory_received, control_received = false;

void trajectoryCallback(const riseq_msgs::FlatTrajectory& msg)
{
	trajectory_received = true;
}

void commandCallback(const mav_msgs::Actuators& msg)
{
	control_received = true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "flightgoogles_arming_node");
	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<std_msgs::Empty>("/uav/input/arm", 1);
	ros::Subscriber reference_sub = n.subscribe("/riseq/uav/trajectory", 1, &trajectoryCallback);
	ros::Subscriber controller_sub = n.subscribe("/uav/input/motorspeed", 1, &commandCallback);

	std_msgs::Empty msg;

	if(true)
	{
		if(ros::ok())
		{
			ros::Rate rate(10);
			for(int i = 0; i < 10; i++)
			{
				pub.publish(msg);
				rate.sleep();
			}
			//break;
		}
		else
		{
      //std::cout << "ARMING NODE: Trajectory not received" << std::endl;
		}
	}

	return 0;
}