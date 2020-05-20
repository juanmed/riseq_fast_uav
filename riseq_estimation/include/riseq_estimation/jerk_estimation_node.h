#ifndef JERKESTIMATIONNODE_V12020_H
#define JERKESTIMATIONNODE_V12020_H

#include <ros/ros.h>
#include <iostream>
#include <Eigen/Dense>

#include <nav_msgs/Odometry.h>
#include <mav_msgs/RateThrust.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "../libs/multicopter_jerk_estimation/jerk_estimator.h"

class JerkEstimationNode
{
public:
	JerkEstimationNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
	virtual ~ JerkEstimationNode();
private:

	ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;	
	ros::Subscriber state_sub_;
	ros::Subscriber thrust_sub_;
	ros::Subscriber imu_sub_;
	ros::Publisher jerk_pub_;
	ros::Timer estimator_timer;

	Eigen::Quaterniond q_;
	Eigen::Vector3d a_imu_;
	Eigen::Vector3d angular_velocity_imu_;
	double thrust_dot_, thrust_;

	JerkEstimator * jerk_estimator_;

	void estimateJerk(const ros::TimerEvent& event);
	void odometryCallback(const nav_msgs::Odometry& msg);
	void imuCallback(const sensor_msgs::Imu& msg);
	void thrustCallback(const mav_msgs::RateThrust& msg);

};

#endif /* JERKESTIMATIONNODE_V12020_H */