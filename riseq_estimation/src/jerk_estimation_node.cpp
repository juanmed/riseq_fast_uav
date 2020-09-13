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
	estimator_timer = nh_.createTimer(ros::Duration(0.01), &JerkEstimationNode::estimateJerk, this);

	Eigen::Quaterniond q_ = Eigen::Quaterniond::Identity();
	Eigen::Vector3d a_imu_ = Eigen::Vector3d::Zero();
	Eigen::Vector3d angular_velocity_imu_ = Eigen::Vector3d::Zero();
	double thrust_dot_, thrust_ = 0.0;

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
      drag_coeff = 0.01; // m/s^2
  }

	jerk_estimator_ = new JerkEstimator(mass, gravity, drag_coeff);
}

void JerkEstimationNode::estimateJerk(const ros::TimerEvent& event)
{
	Eigen::Vector3d jerk = jerk_estimator_ -> computeJerkEstimation(q_, a_imu_, angular_velocity_imu_, thrust_, thrust_dot_);

	if(jerk.norm() < 100.)
	{
		geometry_msgs::Vector3Stamped jerk_msg;
		jerk_msg.header.frame_id = "map";
		jerk_msg.header.stamp = ros::Time::now();
		jerk_msg.vector.x = jerk(0);
		jerk_msg.vector.y = jerk(1);
		jerk_msg.vector.z = jerk(2);
		jerk_pub_.publish(jerk_msg);
	}

}

void JerkEstimationNode::odometryCallback(const nav_msgs::Odometry& msg)
{
	q_.w() =  msg.pose.pose.orientation.w;
	q_.vec() << msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z;
}

void JerkEstimationNode::imuCallback(const sensor_msgs::Imu& msg)
{
	a_imu_ << msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z;
	angular_velocity_imu_ << msg.angular_velocity.x , msg.angular_velocity.y, msg.angular_velocity.z;
}

void JerkEstimationNode::imuBiasCallback(const sensor_msgs::Imu& msg)
{
	Eigen::Vector3d ba(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
	Eigen::Vector3d bg(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
	jerk_estimator_ -> setBiases(ba, bg);
}

void JerkEstimationNode::thrustCallback(const mav_msgs::RateThrust& msg)
{
	thrust_ = msg.thrust.y;
	thrust_dot_ = msg.thrust.x;
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