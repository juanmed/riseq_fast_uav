#ifndef ERRORMETRICSNODE_V12020_H
#define ERRORMETRICSNODE_V12020_H

#include <ros/ros.h>
#include <iostream>
#include <Eigen/Dense>

#include <nav_msgs/Odometry.h>
#include <riseq_msgs/FlatTrajectory.h>
#include <geometry_msgs/Vector3Stamped.h>

#include "../../libs/error_metrics/error_metrics.h"

class ErrorMetricsNode{
public:
	ErrorMetricsNode(const ros::NodeHandle& nh);
	virtual ~ ErrorMetricsNode();
	double ate_;
private:

	ros::NodeHandle nh_;
	ros::Subscriber state_sub_;
	ros::Subscriber trajectory_sub_;
	ros::Publisher error_pub_;
	ros::Publisher error_pub_2_;
	ros::Publisher error_stats_;
	ros::Timer error_timer_;	

	Eigen::Vector3d d_;
	Eigen::Vector3d p_;
	Eigen::Vector3d p_ref_;
	Eigen::Vector3d v_;
	Eigen::Quaterniond q_;
	Eigen::Vector3d angular_velocity_;

	ErrorMetrics * error_calculator_;

	void computer_errors(const ros::TimerEvent& event);
	void odometryCallback(const nav_msgs::Odometry& msg);
	void trajectoryCallback(const riseq_msgs::FlatTrajectory& msg);	

};

#endif /* ERRORMETRICSNODE_V12020_H */ 