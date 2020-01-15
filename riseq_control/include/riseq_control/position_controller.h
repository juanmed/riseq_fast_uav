#ifndef POSITIONCONTROLLER_V12020_H
#define POSITIONCONTROLLER_V12020_H

#include <ros/ros.h>
#include <Eigen/Dense>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <riseq_msgs/FlatTrajectory.h>
#include <mav_msgs/RateThrust.h>

#include "../libs/feedback_linearization_controller/feedback_linearization_controller.h"

class PositionController
{
	public:

		PositionController(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
		virtual ~ PositionController();

	private:

		ros::NodeHandle nh_;
		ros::NodeHandle nh_private_;

		ros::Subscriber reference_sub_;
		ros::Subscriber state_sub_;
		ros::Subscriber position_sub_;
		ros::Subscriber velocity_sub_;

		ros::Publisher input_publisher_;

		ros::Timer control_loop_timer_;

		// reference trajectory
		Eigen::Vector3d p_ref_;
		Eigen::Vector3d v_ref_;
		Eigen::Vector3d a_ref_;
		Eigen::Vector3d j_ref_;
		Eigen::Vector3d s_ref_;
		Eigen::Quaterniond q_ref_;
		Eigen::Vector3d angular_velocity_ref_;
		double yaw_ref_;
		double yaw_dot_ref_;
		double yaw_ddot_ref_;

		// vehicle state
		Eigen::Vector3d p_;
		Eigen::Vector3d v_;
		Eigen::Quaterniond q_;
		Eigen::Vector3d angular_velocity_;

		// control quantities
		Eigen::Vector3d thrust_vector_;
		Eigen::Vector3d desired_orientation_;
		Eigen::Vector3d desired_angular_velocity_;

		// controllers
	  FeedbackLinearizationController * fb_controller_;

		void computeControlInputs(const ros::TimerEvent& event);
		void publishControlInputs(void);

		void odometryCallback(const nav_msgs::Odometry& msg);
		void positionCallback(const geometry_msgs::PoseStamped& msg);
		void velocityCallback(const geometry_msgs::TwistStamped& msg);
		void trajectoryCallback(const riseq_msgs::FlatTrajectory& msg);

};

#endif