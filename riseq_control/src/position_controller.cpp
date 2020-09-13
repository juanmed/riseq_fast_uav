#include "riseq_control/position_controller.h"

PositionController::PositionController(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private): 
	nh_(nh),
	nh_private_(nh_private)
	{
		reference_sub_ = nh_.subscribe("/riseq/uav/trajectory", 1, &PositionController::trajectoryCallback, this, ros::TransportHints().tcpNoDelay());
		state_sub_ = nh_.subscribe("/riseq/uav/state", 1, &PositionController::odometryCallback, this, ros::TransportHints().tcpNoDelay());
		position_sub_ = nh_.subscribe("/riseq/uav/position", 1, &PositionController::positionCallback, this, ros::TransportHints().tcpNoDelay());
		velocity_sub_ = nh_.subscribe("/riseq/uav/velocity", 1, &PositionController::velocityCallback, this, ros::TransportHints().tcpNoDelay());
		imu_sub_ = nh_.subscribe("/riseq/uav/sensors/imu", 1, &PositionController::imuCallback, this, ros::TransportHints().tcpNoDelay());
		high_input_publisher_ = nh_.advertise<mav_msgs::RateThrust>("/riseq/uav/rateThrust", 1);
		low_input_publisher_ = nh_.advertise<mav_msgs::Actuators>("/riseq/uav/motorspeed", 1);
		rdes_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/riseq/uav/desired_orientation", 1);
		high_control_loop_timer_ = nh_.createTimer(ros::Duration(0.01), &PositionController::computeHighControlInputs, this); // Define timer for constant loop rate
		//mid_control_loop_timer_ = nh_.createTimer(ros::Duration(0.01), &PositionController::computeMidControlInputs, this); // Define timer for constant loop rate
	  low_control_loop_timer_ = nh_.createTimer(ros::Duration(0.005), &PositionController::computeLowControlInputs, this);

		// initialize members
		p_ref_ << 0., 0., 0.;
		v_ref_ << 0., 0., 0.;
		a_ref_ << 0., 0., 0.;
		j_ref_ << 0., 0., 0.;
		s_ref_ << 0., 0., 0.;
		q_ref_ = Eigen::Quaterniond::Identity();
		angular_velocity_ref_ = Eigen::Vector3d::Zero();
		angular_velocity_dot_ref_ = Eigen::Vector3d::Zero();
		yaw_ref_ = 0.;
		yaw_dot_ref_ = 0.;
		yaw_ddot_ref_ = 0.;

		p_ << 0., 0., 0.;
		v_ << 0., 0., 0.;
		q_ = Eigen::Quaterniond::Identity();
		angular_velocity_ << 0., 0., 0.;
		a_ = Eigen::Vector3d::Zero();

		thrust_vector_ << 0., 0., 0.;
		desired_orientation_= Eigen::Matrix3d::Zero();
		desired_angular_velocity_ = Eigen::Vector3d::Zero();
		torque_vector_ = Eigen::Vector3d::Zero();
		rotor_rpms_ = Eigen::Vector4d::Zero();
		collective_thrust_2 = 0.0;

		// sensors
		a_imu_ = Eigen::Vector3d::Zero();

		enable_output_ = false;

		e1_ << 1., 0., 0.;
		e2_ << 0., 1., 0.;
		e3_ << 0., 0., 1.;

		// get vehicle parameters
		double mass, gravity;
	  if (!nh_private_.param<double>("riseq/mass", mass, 1.0)){
	      std::cout << "Did not get mass from the params, defaulting to 1.0" << std::endl;
	  }			
	  if (!nh_private_.param<double>("riseq/gravity", gravity, 9.81)){
	      std::cout << "Did not get gravity from the params, defaulting to 9.81" << std::endl;
	  }
	  gravity_ = gravity;

  	vehicleInertia_ = Eigen::Matrix3d::Zero();
	  if (!nh_private_.param<double>("riseq/Ixx", vehicleInertia_(0, 0), 0.0049)){
	      std::cout << "Did not get Ixx from the params, defaulting to 0.0049" << std::endl;
	  }		
	  if (!nh_private_.param<double>("riseq/Iyy", vehicleInertia_(1, 1), 0.0049)){
	      std::cout << "Did not get Iyy from the params, defaulting to 0.0049" << std::endl;
	  }	
	  if (!nh_private_.param<double>("riseq/Izz", vehicleInertia_(2, 2), 0.0069)){
	      std::cout << "Did not get Izz from the params, defaulting to 0.0069" << std::endl;
	  }

		double Kp_x, Kp_y, Kp_z, Kd_x, Kd_y, Kd_z, Kr, Ct, Cq, arm_length  = 0.;

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
	  if (!nh_private_.param<double>("riseq/thrust_coeff", Ct, 1.91e-6)){
	      std::cout << "Did not get thrust coefficient from the params, defaulting to 1.91e-6" << std::endl;
	  }
	  if (!nh_private_.param<double>("riseq/torque_coeff", Cq, 2.6e-7)){
	      std::cout << "Did not get torque coefficient from the params, defaulting to 2.6e-7" << std::endl;
	  }
	  if (!nh_private_.param<double>("riseq/arm_length", arm_length, 0.08)){
	      std::cout << "Did not get arm length from the params, defaulting to 0.08" << std::endl;
	  }
		Kp_ << Kp_x, Kp_y, Kp_z;
		Kd_ << Kd_x, Kd_y, Kd_z;
		Ki_ << 0., 0., 0.;
		Kr_  = Kr;

		// if true, + configurate, else X configuration
		Eigen::Matrix4d mixer_matrix = Eigen::Matrix4d::Zero();
		if(false)
		{
			mixer_matrix << Ct, Ct, Ct, Ct,
											0., arm_length * Ct, 0., -arm_length * Ct,
											-arm_length * Ct, 0., arm_length * Ct, 0.,
											-Cq, Cq, -Cq, Cq;
		}
		else
		{
			double m = std::sqrt(2.0)/2.0;
			mixer_matrix << Ct, Ct, Ct, Ct,
											m * arm_length * Ct, m * arm_length * Ct, -m * arm_length * Ct, -m * arm_length * Ct,
											-m * arm_length * Ct, m * arm_length * Ct, m * arm_length * Ct, -m * arm_length * Ct,
											-Cq, +Cq, -Cq, +Cq;
		}		
		mixer_matrix_inv_ = mixer_matrix.inverse();

		fb_controller_ = new FeedbackLinearizationController(mass, 1.0, 1.0, vehicleInertia_, Kp_, Kd_, Ki_, Kr_, gravity);
	}

	void PositionController::computeHighControlInputs(const ros::TimerEvent& event)
	{
		if(enable_output_ & imu_received_)
		{
			Eigen::Vector3d a_des;
			Eigen::Matrix3d Rbw;
			a_des = fb_controller_->computeDesiredAcceleration(p_, p_ref_, v_, v_ref_, a_ref_);
			thrust_vector_ = fb_controller_->computeDesiredThrustVector(q_, a_des);
			desired_orientation_ = fb_controller_->computeDesiredOrientation(thrust_vector_, yaw_ref_);
			q_.normalize();
			Rbw = q_.toRotationMatrix();
			a_ = Rbw * a_imu_ - gravity_ * e3_;
			desired_angular_velocity_ = fb_controller_->computeDesiredAngularVelocity(q_.toRotationMatrix(), desired_orientation_, euler_dot_ref_);
			publishHighControlInputs();
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
			q_.normalize();
			desired_angular_velocity_ = fb_controller_->computeDesiredAngularVelocity(q_.toRotationMatrix(), desired_orientation_, euler_dot_ref_);
			publishHighControlInputs();
		}
		else
		{
			// do nothing if output is disabled
		}
	}

	void PositionController::computeLowControlInputs(const ros::TimerEvent& event)
	{
		if(enable_output_ & imu_received_)
		{
			//torque_vector_ = fb_controller_->computeDesiredTorque(angular_velocity_, desired_angular_velocity_, torque_ref_);
			torque_vector_ = fb_controller_->computeDesiredTorque2(angular_velocity_, desired_angular_velocity_, angular_velocity_dot_ref_);
			rotor_rpms_ = fb_controller_-> computeRotorRPM(thrust_vector_.norm(), torque_vector_, mixer_matrix_inv_);
			publishLowControlInputs();
		}
		else
		{
			// do nothing if output is disabled
		}
	}

	void PositionController::publishHighControlInputs(void)
	{

		//Eigen::Quaterniond des_q(desired_orientation_);
		geometry_msgs::PoseStamped desired_orientation_msg;
		desired_orientation_msg.header.stamp = ros::Time::now();
		desired_orientation_msg.header.frame_id = "map";
		desired_orientation_msg.pose.position.x = a_(0);
		desired_orientation_msg.pose.position.y = a_(1);
		desired_orientation_msg.pose.position.z = a_(2);
		//desired_orientation_msg.pose.orientation.x = yaw_ref_; //des_q.x();
		//desired_orientation_msg.pose.orientation.y = v_(1); //des_q.y();
		//desired_orientation_msg.pose.orientation.z = v_ref_(0); //des_q.z();
		//desired_orientation_msg.pose.orientation.w = v_ref_(1); //des_q.w();
		rdes_publisher_.publish(desired_orientation_msg);

		mav_msgs::RateThrust command_msg;
		double thrust  = thrust_vector_.norm();

		command_msg.header.stamp = ros::Time::now();
		command_msg.header.frame_id = "map";
		command_msg.thrust.y = thrust;
		command_msg.thrust.x = (thrust - collective_thrust_2)/0.01; // approx derivative of collective thrust
		collective_thrust_2 = thrust;
		command_msg.thrust.z = thrust;
		command_msg.angular_rates.x = desired_angular_velocity_(0);
		command_msg.angular_rates.y = desired_angular_velocity_(1);
		command_msg.angular_rates.z = desired_angular_velocity_(2);

		high_input_publisher_.publish(command_msg);
	}

	void PositionController::publishLowControlInputs(void)
	{

		//debugging only
		mav_msgs::Actuators command_msg;
		command_msg.header.stamp = ros::Time::now();
		command_msg.header.frame_id = "map";
		for (uint i = 0; i < 4; i++)
		{
			command_msg.angular_velocities.push_back(rotor_rpms_(i));
		}
		low_input_publisher_.publish(command_msg);
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
		angular_velocity_dot_ref_ << msg.ub.x, msg.ub.y, msg.ub.z;
		euler_dot_ref_ << msg.uc.x, msg.uc.y, msg.uc.z;
		torque_ref_ << msg.ux.x, msg.ux.y, msg.ux.z;
		enable_output_ = true;
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

	void PositionController::imuCallback(const sensor_msgs::Imu& msg)
	{
		a_imu_ << msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z;
		imu_received_ = true;
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

