#ifndef ARMINGCONTROLLER_V12020_H
#define ARMINGCONTROLLER_V12020_H

#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Empty.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/RateThrust.h>
#include <riseq_msgs/FlatTrajectory.h>

class ArmingController
{
public:
	ArmingController(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
		double rate);
	virtual ~ ArmingController();

private:

	ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;
	ros::Publisher pub_;
	ros::Subscriber reference_sub_;
	ros::Subscriber controller_sub_;
	ros::Timer arming_timer_;

	double rate_;
	bool trajectory_received_, control_received_ = false;

	void arm(const ros::TimerEvent& event);
	void trajectoryCallback(const riseq_msgs::FlatTrajectory& msg);
	void commandCallback(const mav_msgs::RateThrust& msg); 
};

ArmingController::ArmingController(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
	double rate):	nh_(nh), nh_private_(nh_private)
{
	rate_ = rate;
	pub_ =  nh_.advertise<std_msgs::Empty>("/uav/input/arm", 1);
	reference_sub_ = nh_.subscribe("/riseq/uav/trajectory", 1, &ArmingController::trajectoryCallback, this, ros::TransportHints().tcpNoDelay());
	controller_sub_ = nh_.subscribe("/riseq/uav/rateThrust", 1, &ArmingController::commandCallback, this, ros::TransportHints().tcpNoDelay());
	arming_timer_ = nh_.createTimer(ros::Duration(rate_), &ArmingController::arm, this);
}

void ArmingController::arm(const ros::TimerEvent& event)
{
	std_msgs::Empty msg;
	// arm only after trajectory and control signals are already being sent
	if(trajectory_received_)
	{
		ros::Rate rate(10);
		int i = 0;
		for(i; i < 10; i++)
		{
			pub_.publish(msg);
			rate.sleep();
		}
		arming_timer_.stop();
		std::cout << "ARMING SENT " << i << " TIMES" << std::endl;
	}
	else
	{
    std::cout << "ARMING NODE: Trajectory not received" << std::endl;
	}
}

void ArmingController::trajectoryCallback(const riseq_msgs::FlatTrajectory& msg)
{
	trajectory_received_ = true;
}

void ArmingController::commandCallback(const mav_msgs::RateThrust& msg)
{
	control_received_ = true;
} 

ArmingController::~ArmingController(){
	/* Destructor */
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "flightgoogles_arming_node");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");

	ArmingController *arm = new ArmingController(nh, nh_private, 0.1);
	ros::spin();
	return 0;
}

#endif /* ARMINGCONTROLLER */