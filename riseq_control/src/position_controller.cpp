#include "riseq_control/position_controller.h"


using namespace std;

PositionController::PositionController(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private): 
	nh_(nh),
	nh_private_(nh_private){

		reference_sub_ = nh_.subscribe("/riseq/uav/trajectory", 1, &PositionController::trajectoryCallback, this, ros::TransportHints().tcpNoDelay());
		state_sub_ = nh_.subscribe("/riseq/uav/state", 1, &PositionController::odometryCallback, this, ros::TransportHints().tcpNoDelay());
		position_sub_ = nh_.subscribe("/riseq/uav/position", 1, &PositionController::positionCallback, this, ros::TransportHints().tcpNoDelay());
		velocity_sub_ = nh_.subscribe("/riseq/uav/velocity", 1, &PositionController::velocityCallback, this, ros::TransportHints().tcpNoDelay());

		input_publisher_ = nh_.advertise<mav_msgs::RateThrust>("/riseq/uav/rateThrust", 1);

		control_loop_timer_ = nh_.createTimer(ros::Duration(0.01), &PositionController::computeControlInputs, this); // Define timer for constant loop rate

		// initialize members
		p_ref_ << 0., 0., 0.;
		v_ref_ << 0., 0., 0.;
		a_ref_ << 0., 0., 0.;
		j_ref_ << 0., 0., 0.;
		s_ref_ << 0., 0., 0.;
		q_ref_ = Eigen::Quaterniond::Identity();
		yaw_ref_ = 0.;
		yaw_dot_ref_ = 0.;
		yaw_ddot_ref_ = 0.;

		p_ << 0., 0., 0.;
		v_ << 0., 0., 0.;
		q_ = Eigen::Quaterniond::Identity();
		angular_velocity_ << 0., 0., 0.;

		thrust_vector_ << 0., 0., 0.;
		desired_orientation_= Eigen::Vector3d::Zero();
		desired_angular_velocity_ = Eigen::Vector3d::Zero();

		double Kp_x, Kp_y, Kp_z, Kd_x, Kd_y, Kd_z = 0.;
		double Kr = 8.0;


		/*
	  if (!ros::param::get("riseq/Kp_x", Kp_x, 8.0)){
	      std::cout << "Did not get bool Kp_x from the params, defaulting to 8.0" << std::endl;
	  }		
	  if (!ros::param::get("riseq/Kp_x", Kp_x, 8.0)){
	      std::cout << "Did not get bool Kp_x from the params, defaulting to 8.0" << std::endl;
	  }	
	  if (!ros::param::get("riseq/Kp_x", Kp_x, 8.0)){
	      std::cout << "Did not get bool Kp_x from the params, defaulting to 8.0" << std::endl;
	  }	
	  if (!ros::param::get("riseq/Kp_x", Kp_x, 8.0)){
	      std::cout << "Did not get bool Kp_x from the params, defaulting to 8.0" << std::endl;
	  }	
	  if (!ros::param::get("riseq/Kp_x", Kp_x, 8.0)){
	      std::cout << "Did not get bool Kp_x from the params, defaulting to 8.0" << std::endl;
	  }	
		*/

		Eigen::Vector3d Kp(Kp_x, Kp_y, Kp_z);
		Eigen::Vector3d Kd(Kd_x, Kd_y, Kd_z);
		Eigen::Vector3d Ki = Eigen::Vector3d::Zero();

		fb_controller_ = new FeedbackLinearizationController(1.0, 1.0, 1.0, Eigen::Matrix3d::Identity(), Kp, Kd, Ki, Kr);



	}

	void PositionController::computeControlInputs(const ros::TimerEvent& event)
	{
		Eigen::Vector3d thrust_vector_;
		thrust_vector_ = fb_controller_->computeDesiredThrustVector(p_, p_ref_, v_, v_ref_, a_ref_);
		fb_controller_->computeDesiredOrientation();
		fb_controller_->computeDesiredAngularVelocity();

		publishControlInputs();
	}

	void PositionController::publishControlInputs(void)
	{
		mav_msgs::RateThrust command_msg;

		command_msg.header.stamp = ros::Time::now();
		command_msg.header.frame_id = "map";
		command_msg.thrust.z = thrust_vector_.norm();
		command_msg.angular_rates.x = desired_angular_velocity_(0);
		command_msg.angular_rates.y = desired_angular_velocity_(1);
		command_msg.angular_rates.z = desired_angular_velocity_(2);

		input_publisher_.publish(command_msg);

	}

	void PositionController::trajectoryCallback(const riseq_msgs::FlatTrajectory& msg)
	{
		p_ref_ << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
		v_ref_ << msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z;
		a_ref_ << msg.acc.x, msg.acc.y, msg.acc.z;
		j_ref_ << msg.jerk.x, msg.jerk.y, msg.jerk.z;
		s_ref_ << msg.snap.x, msg.snap.y, msg.snap.z;

		q_ref_.w() = msg.pose.orientation.w;
		q_ref_.vec() << msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z;
		yaw_ref_ = msg.yaw;
		yaw_dot_ref_ = msg.yawdot;
		yaw_ddot_ref_ = msg.yawddot;

		angular_velocity_ref_ << msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z;
	}

	void PositionController::odometryCallback(const nav_msgs::Odometry& msg)
	{
		p_ << msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z;
		v_ << msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z;
		q_.w() =  msg.pose.pose.orientation.w;
		q_.vec() << msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z;
		angular_velocity_ << msg.twist.twist.angular.x , msg.twist.twist.angular.y, msg.twist.twist.angular.z;
	}

	void PositionController::positionCallback(const geometry_msgs::PoseStamped& msg)
	{
		p_ << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
		q_.w() =  msg.pose.orientation.w;
		q_.vec() << msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z;
	}

	void PositionController::velocityCallback(const geometry_msgs::TwistStamped& msg)
	{
		v_ << msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z;
		angular_velocity_ << msg.twist.angular.x , msg.twist.angular.y, msg.twist.angular.z;
	}

	PositionController::~PositionController()
	{
  //Destructor
	}

