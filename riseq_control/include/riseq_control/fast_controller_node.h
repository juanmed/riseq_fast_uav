#ifndef FASTCONTROLLERNODE_V12020_H
#define FASTCONTROLLERNODE_V12020_H

#include <ros/ros.h>
#include <iostream>
#include <Eigen/Dense>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <riseq_msgs/FlatTrajectory.h>
#include <mav_msgs/RateThrust.h>
#include <mav_msgs/Actuators.h>
#include <sensor_msgs/Imu.h>
#include "../libs/fast_controller/fast_controller.h"
#include "../libs/feedback_linearization_controller/feedback_linearization_controller.h"

class FastControllerNode
{
	public:
		FastControllerNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
		virtual ~ FastControllerNode();

	private:

		ros::NodeHandle nh_;
		ros::NodeHandle nh_private_;
		ros::Subscriber reference_sub_;
		ros::Subscriber state_sub_;
		ros::Subscriber position_sub_;
		ros::Subscriber velocity_sub_;
		ros::Subscriber imu_sub_;
		ros::Publisher high_input_publisher_;
		ros::Publisher low_input_publisher_;
		ros::Publisher rdes_publisher_;
		ros::Timer high_control_loop_timer_;
		ros::Timer mid_control_loop_timer_;
		ros::Timer low_control_loop_timer_;

		// flow control 
		bool trajectory_received_, state_received_, imu_received_ = false;

		// reference trajectory
		Eigen::Vector3d p_ref_;
		Eigen::Vector3d v_ref_;
		Eigen::Vector3d a_ref_;
		Eigen::Vector3d j_ref_;
		Eigen::Vector3d s_ref_;
		Eigen::Quaterniond q_ref_;
		Eigen::Vector3d angular_velocity_ref_;
		Eigen::Vector3d angular_velocity_dot_ref_;
		Eigen::Vector3d euler_dot_ref_;
		Eigen::Vector3d torque_ref_;
		double yaw_ref_;
		double yaw_dot_ref_;
		double yaw_ddot_ref_;
		double thrust_ref_;

		// vehicle state
		Eigen::Vector3d p_;
		Eigen::Vector3d v_;
		Eigen::Quaterniond q_;
		Eigen::Vector3d angular_velocity_;
		Eigen::Vector3d a_, j_ = Eigen::Vector3d::Zero();

		// control quantities
		Eigen::Vector3d collective_thrust_vector_;
		Eigen::Vector3d collective_thrust_vector_dot_;
		Eigen::Vector3d collective_thrust_vector_ddot_;
		Eigen::Matrix3d desired_orientation_;
		Eigen::Matrix3d desired_orientation_dot_;
		Eigen::Matrix3d desired_orientation_ddot_;
		Eigen::Vector3d desired_angular_velocity_;
		Eigen::Vector3d desired_angular_velocity_dot_;
		Eigen::Vector3d torque_vector_;
		Eigen::Vector4d rotor_rpms_;
		double collective_thrust_;
		double command_thrust_;

		// sensors
		Eigen::Vector3d a_imu_;

		// control gains
		Eigen::Vector3d Kp_;
		Eigen::Vector3d Kd_;
		Eigen::Vector3d Ki_;
		double Kr_, Ko_;

		// Vehicle physical quantities
		Eigen::Matrix3d vehicleInertia_;
		Eigen::Matrix4d mixer_matrix_inv_;
		double gravity_, cd1_;
		int rotor_count_;

 		// basis vector generating R^3
 		Eigen::Vector3d e1_;
 		Eigen::Vector3d e2_;
 		Eigen::Vector3d e3_;

		// controllers
	  FastController * controller_;

		void computeHighControlInputs(const ros::TimerEvent& event);
		void computeLowControlInputs(const ros::TimerEvent& event);

		void publishHighControlInputs(void);
		void publishLowControlInputs(void);

		void odometryCallback(const nav_msgs::Odometry& msg);
		void positionCallback(const geometry_msgs::PoseStamped& msg);
		void velocityCallback(const geometry_msgs::TwistStamped& msg);
		void trajectoryCallback(const riseq_msgs::FlatTrajectory& msg);
		void imuCallback(const sensor_msgs::Imu& msg);

		void initializeParameters(void);

};

#endif /* FASTCONTROLLERNODE_V12020_H */