#include "riseq_control/position_controller.h"


PositionController::PositionController(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private): 
	nh_(nh),
	nh_private_(nh_private)
	{
		reference_sub_ = nh_.subscribe("/riseq/uav/trajectory", 1, &PositionController::trajectoryCallback, this, ros::TransportHints().tcpNoDelay());
		state_sub_ = nh_.subscribe("/riseq/uav/state", 1, &PositionController::odometryCallback, this, ros::TransportHints().tcpNoDelay());
		position_sub_ = nh_.subscribe("/riseq/uav/position", 1, &PositionController::positionCallback, this, ros::TransportHints().tcpNoDelay());
		velocity_sub_ = nh_.subscribe("/riseq/uav/velocity", 1, &PositionController::velocityCallback, this, ros::TransportHints().tcpNoDelay());
		input_publisher_ = nh_.advertise<mav_msgs::RateThrust>("/riseq/uav/rateThrust", 1);
		rdes_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/riseq/uav/desired_orientation", 1);
		control_loop_timer_ = nh_.createTimer(ros::Duration(0.02), &PositionController::computeControlInputs, this); // Define timer for constant loop rate
		mid_control_loop_timer_ = nh_.createTimer(ros::Duration(0.01), &PositionController::computeMidControlInputs, this); // Define timer for constant loop rate

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
		desired_orientation_= Eigen::Matrix3d::Zero();
		desired_angular_velocity_ = Eigen::Vector3d::Zero();

		enable_output_ = false;

		// get vehicle parameters
		double mass;
	  if (!nh_private_.param<double>("riseq/mass", mass, 1.0)){
	      std::cout << "Did not get mass from the params, defaulting to 1.0" << std::endl;
	  }			

  	Eigen::Matrix3d vehicleInertia;
	  if (!nh_private_.param<double>("riseq/Ixx", vehicleInertia(0, 0), 0.1)){
	      std::cout << "Did not get Kp_x from the params, defaulting to 8.0" << std::endl;
	  }		
	  if (!nh_private_.param<double>("riseq/Iyy", vehicleInertia(1, 1), 0.1)){
	      std::cout << "Did not get Kp_y from the params, defaulting to 8.0" << std::endl;
	  }	
	  if (!nh_private_.param<double>("riseq/Izz", vehicleInertia(2, 2), 0.1)){
	      std::cout << "Did not get Kp_z from the params, defaulting to 8.0" << std::endl;
	  }

		double Kp_x, Kp_y, Kp_z, Kd_x, Kd_y, Kd_z, Kr = 0.;

	  if (!nh_private_.param<double>("riseq/Kp_x", Kp_x, 4.0)){
	      std::cout << "Did not get Kp_x from the params, defaulting to 8.0" << std::endl;
	  }		
	  if (!nh_private_.param<double>("riseq/Kp_y", Kp_y, 4.0)){
	      std::cout << "Did not get Kp_y from the params, defaulting to 8.0" << std::endl;
	  }	
	  if (!nh_private_.param<double>("riseq/Kp_z", Kp_z, 4.5)){
	      std::cout << "Did not get Kp_z from the params, defaulting to 8.0" << std::endl;
	  }	
	  if (!nh_private_.param<double>("riseq/Kd_x", Kd_x, 1.0)){
	      std::cout << "Did not get Kd_x from the params, defaulting to 8.0" << std::endl;
	  }	
	  if (!nh_private_.param<double>("riseq/Kd_y", Kd_y, 1.0)){
	      std::cout << "Did not get Kd_y from the params, defaulting to 8.0" << std::endl;
	  }	
	  if (!nh_private_.param<double>("riseq/Kd_z", Kd_z, 1.0)){
	      std::cout << "Did not get Kd_z from the params, defaulting to 8.0" << std::endl;
	  }
	  if (!nh_private_.param<double>("riseq/Kr", Kr, 8.0)){
	      std::cout << "Did not get Kr from the params, defaulting to 8.0" << std::endl;
	  }

		Kp_ << Kp_x, Kp_y, Kp_z;
		Kd_ << Kd_x, Kd_y, Kd_z;
		Ki_ << 0., 0., 0.;
		Kr_  = Kr;

		fb_controller_ = new FeedbackLinearizationController(mass, 1.0, 1.0, vehicleInertia, Kp_, Kd_, Ki_, Kr_);
	}

	void PositionController::computeControlInputs(const ros::TimerEvent& event)
	{
		if(enable_output_)
		{
			Eigen::Vector3d a_des;
			a_des = fb_controller_->computeDesiredAcceleration(p_, p_ref_, v_, v_ref_, a_ref_);
			thrust_vector_ = fb_controller_->computeDesiredThrustVector(q_, a_des);
			desired_orientation_ = fb_controller_->computeDesiredOrientation(thrust_vector_, yaw_ref_);
			//desired_angular_velocity_ = fb_controller_->computeDesiredAngularVelocity(q_.normalized().toRotationMatrix(), desired_orientation_, euler_dot_ref_);
		}
		else
		{
			// do nothing if reference trajectory is not received
		}
	}

	void PositionController::computeMidControlInputs(const ros::TimerEvent& event)
	{	
		if(enable_output_)
		{
			desired_angular_velocity_ = fb_controller_->computeDesiredAngularVelocity(q_.toRotationMatrix(), desired_orientation_, euler_dot_ref_);
			publishControlInputs();
		}
		else
		{
			// do nothing if reference trajectory is not received
		}
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

		//debugging only

		Eigen::Quaterniond des_q(desired_orientation_);

		geometry_msgs::PoseStamped desired_orientation_msg;
		desired_orientation_msg.header.stamp = ros::Time::now();
		desired_orientation_msg.header.frame_id = "map";
		desired_orientation_msg.pose.orientation.x = des_q.x();
		desired_orientation_msg.pose.orientation.y = des_q.y();
		desired_orientation_msg.pose.orientation.z = des_q.z();
		desired_orientation_msg.pose.orientation.w = des_q.w();
		rdes_publisher_.publish(desired_orientation_msg);
	}

	void PositionController::trajectoryCallback(const riseq_msgs::FlatTrajectory& msg)
	{
		enable_output_ = true;
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
		euler_dot_ref_ << msg.uc.x, msg.uc.y, msg.uc.z;
	}

	void PositionController::odometryCallback(const nav_msgs::Odometry& msg)
	{
		p_ << msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z;
		v_ << msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z;
		q_.w() =  msg.pose.pose.orientation.w;
		q_.vec() << msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z;
		q_ = q_.conjugate(); //transform from world-to-body to body-to-world rotation
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

	void PositionController::initializeParameters(void)
	{
		double Kp_x, Kp_y, Kp_z, Kd_x, Kd_y, Kd_z, Kr = 0.;

	  if (!nh_private_.param<double>("riseq/Kp_x", Kp_x, 8.0)){
	      std::cout << "Did not get bool Kp_x from the params, defaulting to 8.0" << std::endl;
	  }		
	  if (!nh_private_.param<double>("riseq/Kp_y", Kp_y, 8.0)){
	      std::cout << "Did not get bool Kp_y from the params, defaulting to 8.0" << std::endl;
	  }	
	  if (!nh_private_.param<double>("riseq/Kp_z", Kp_z, 8.0)){
	      std::cout << "Did not get bool Kp_z from the params, defaulting to 8.0" << std::endl;
	  }	
	  if (!nh_private_.param<double>("riseq/Kd_x", Kd_x, 8.0)){
	      std::cout << "Did not get bool Kd_x from the params, defaulting to 8.0" << std::endl;
	  }	
	  if (!nh_private_.param<double>("riseq/Kd_y", Kd_y, 8.0)){
	      std::cout << "Did not get bool Kd_y from the params, defaulting to 8.0" << std::endl;
	  }	
	  if (!nh_private_.param<double>("riseq/Kd_z", Kd_z, 8.0)){
	      std::cout << "Did not get bool Kd_z from the params, defaulting to 8.0" << std::endl;
	  }
	  if (!nh_private_.param<double>("riseq/Kr", Kr, 8.0)){
	      std::cout << "Did not get bool Kr from the params, defaulting to 8.0" << std::endl;
	  }

		Kp_ << Kp_x, Kp_y, Kp_z;
		Kd_ << Kd_x, Kd_y, Kd_z;
		Ki_ << 0., 0., 0.;
		Kr_  = Kr;
	}

	PositionController::~PositionController()
	{
  //Destructor
	}

