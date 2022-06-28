#include "riseq_estimation/jerk_estimation_node.h"

JerkEstimationNode::JerkEstimationNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
nh_(nh),
nh_private_(nh_private)
{
	state_sub_ = nh_.subscribe("/riseq/uav/state", 1, &JerkEstimationNode::odometryCallback, this, ros::TransportHints().tcpNoDelay());
	thrust_sub_ = nh_.subscribe("riseq/uav/rateThrust", 1, &JerkEstimationNode::thrustCallback, this, ros::TransportHints().tcpNoDelay());
	imu_sub_ = nh_.subscribe("/riseq/uav/sensors/imu", 1, &JerkEstimationNode::imuCallback, this, ros::TransportHints().tcpNoDelay());
	imu_bias_sub_ = nh_.subscribe("/riseq/uav/sensors/imu_bias", 1, &JerkEstimationNode::imuBiasCallback, this, ros::TransportHints().tcpNoDelay());
	jerk_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/riseq/uav/jerk", 10);
	imu_corrected_pub_ = nh_.advertise<sensor_msgs::Imu>("/riseq/uav/sensors/imu_corrected", 10);
	estimator_timer = nh_.createTimer(ros::Duration(0.01), &JerkEstimationNode::estimateJerk, this);

	Eigen::Quaterniond q_ = Eigen::Quaterniond::Identity();
	Eigen::Vector3d a_imu_, ba_, bg_, v_ = Eigen::Vector3d::Zero();
	Eigen::Vector3d angular_velocity_imu_ = Eigen::Vector3d::Zero();
	double thrust_dot_, thrust_, thrust_prev_ = 0.0;

	// get vehicle parameters
	double mass, gravity, drag_coeff;
  if (!ros::param::get("riseq/mass", mass)){
      std::cout << "Did not get mass from the params, defaulting to 1.0" << std::endl;
      mass = 1.0; // kg
  }			
  if (!ros::param::get("riseq/gravity", gravity)){
      std::cout << "Did not get gravity from the params, defaulting to 9.81" << std::endl;
      gravity = 9.81; // m/s^2
  }
  if (!ros::param::get("riseq/rotor_drag_coefficient", drag_coeff)){
      std::cout << "Did not get rotor drag coefficient from the params, defaulting to 0.01" << std::endl;
      drag_coeff = 0.1; // m/s^2
  }

	jerk_estimator_ = new JerkEstimator(mass, gravity, drag_coeff);
}

void JerkEstimationNode::estimateJerk(const ros::TimerEvent& event)
{

	if( jerk_estimator_-> initialized())
	{

		Eigen::Vector3d jerk = jerk_estimator_ -> computeJerkEstimation(q_, a_imu_, 
			angular_velocity_imu_, v_, thrust_, thrust_dot_);

		geometry_msgs::Vector3Stamped jerk_msg;
		jerk_msg.header.frame_id = "map";
		jerk_msg.header.stamp = ros::Time::now();
		jerk_msg.vector.x = jerk(0);
		jerk_msg.vector.y = jerk(1);
		jerk_msg.vector.z = jerk(2);
		jerk_pub_.publish(jerk_msg);

		sensor_msgs::Imu imu_msg;
		imu_msg.header.frame_id = "map";
		imu_msg.header.stamp = ros::Time::now();
		Eigen::Vector3d linear_acceleration_corrected(a_imu_ - ba_);	
		Eigen::Vector3d angular_velocity_corrected(angular_velocity_imu_ - bg_);
		imu_msg.linear_acceleration.x = linear_acceleration_corrected.x();
		imu_msg.linear_acceleration.y = linear_acceleration_corrected.y();
		imu_msg.linear_acceleration.z = linear_acceleration_corrected.z();
		imu_msg.angular_velocity.x = angular_velocity_corrected.x();
		imu_msg.angular_velocity.y = angular_velocity_corrected.y();
		imu_msg.angular_velocity.z = angular_velocity_corrected.z();
		imu_corrected_pub_.publish(imu_msg);
	}	

}

void JerkEstimationNode::odometryCallback(const nav_msgs::Odometry& msg)
{
	q_.w() =  msg.pose.pose.orientation.w;
	q_.vec() << msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z;
	v_ << msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z;
}

void JerkEstimationNode::imuCallback(const sensor_msgs::Imu& msg)
{
	a_imu_ << -msg.linear_acceleration.x, -msg.linear_acceleration.y, -msg.linear_acceleration.z;
	angular_velocity_imu_ << -msg.angular_velocity.x , -msg.angular_velocity.y, -msg.angular_velocity.z;
}

void JerkEstimationNode::imuBiasCallback(const sensor_msgs::Imu& msg)
{
	ba_ << msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z;
	bg_ << msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z;
	jerk_estimator_ -> setBiases(ba_, bg_);
}

void JerkEstimationNode::thrustCallback(const blackbird::MotorRPM& msg)
{
	double sum_sqr_rpm = std::pow(msg.rpm[0], 2) + std::pow(msg.rpm[1], 2) + std::pow(msg.rpm[2], 2) + std::pow(msg.rpm[3], 2);
	thrust_ = 2.27e-8 * sum_sqr_rpm;
	thrust_dot_ = 0.5 * (thrust_ - thrust_prev_);
	thrust_prev_ = thrust_;
}

JerkEstimationNode::~JerkEstimationNode()
{
  //Destructor
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "jerk_estimation_node");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");

	//PositionController *controller = new PositionController(nh, nh_private);
	JerkEstimationNode *jerk_estimator_node = new JerkEstimationNode(nh, nh_private);
	ros::spin();
	return 0;
}