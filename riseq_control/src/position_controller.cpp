#include "riseq_control/position_controller.h"


using namespace std;

PositionController::PositionController(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private): 
	nh_(nh),
	nh_private_(nh_private){

		reference_sub_ = nh_.subscribe("/riseq/uav/trajectory", 1, &PositionController::trajectoryCallback, this, ros::TransportHints().tcpNoDelay());
		state_sub_ = nh_.subscribe("/riseq/uav/state", 1, &PositionController::odometryCallback, this, ros::TransportHints().tcpNoDelay());
		position_sub_ = nh_.subscribe("/riseq/uav/position", 1, &PositionController::positionCallback, this, ros::TransportHints().tcpNoDelay());
		velocity_sub_ = nh_.subscribe("/riseq/uav/velocity", 1, &PositionController::velocityCallback, this, ros::TransportHints().tcpNoDelay());

		input_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/riseq/uav/rateThrust", 1);

		p_ref_ << 0., 0., 0.;
		v_ref_ << 0., 0., 0.;
		a_ref_ << 0., 0., 0.;
		j_ref_ << 0., 0., 0.;
		s_ref_ << 0., 0., 0.;

		p_ << 0., 0., 0.;
		v_ << 0., 0., 0.;
		q_ = Eigen::Quaterniond::Identity();
		angular_velocity_ << 0., 0., 0.;


	}

	void PositionController::computeControlInputs()
	{

	}

	void PositionController::publishControlInputs()
	{

	}

	void PositionController::trajectoryCallback(const geometry_msgs::PoseStamped& msg)
	{

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

