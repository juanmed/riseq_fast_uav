#include "error_metrics_node.h"

ErrorMetricsNode::ErrorMetricsNode(const ros::NodeHandle& nh):nh_(nh)
{
	state_sub_ = nh_.subscribe("/riseq/uav/state", 1, &ErrorMetricsNode::odometryCallback, this, ros::TransportHints().tcpNoDelay());
	trajectory_sub_ = nh_.subscribe("/riseq/uav/trajectory", 1, &ErrorMetricsNode::trajectoryCallback, this, ros::TransportHints().tcpNoDelay());
	error_timer_ = nh_.createTimer(ros::Duration(0.01), &ErrorMetricsNode::computer_errors, this);
	error_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/riseq/uav/error_absolute", 20);
	error_pub_2_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/riseq/uav/error_alternative", 20);
	error_stats_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/riseq/uav/error_stats", 20);

	p_ = Eigen::Vector3d::Zero();
	p_ref_ = Eigen::Vector3d::Zero();
	v_ = Eigen::Vector3d::Zero();

	ate_ = 0.0;

	double arm_length = 0.0;
  if (!nh_.param<double>("riseq/arm_length", arm_length, 0.08)){
      std::cout << "Did not get arm length from the params, defaulting to 0.08" << std::endl;
  }

  d_ << arm_length, arm_length, 0.002;
	error_calculator_ = new ErrorMetrics(d_);	
}

void ErrorMetricsNode::computer_errors(const ros::TimerEvent& event)
{
	error_calculator_ -> increaseSamples();
	Eigen::Vector3d error = error_calculator_ -> vector_error(p_, p_ref_);
	ate_ = error_calculator_ -> absoluteTrajectoryError(p_, p_ref_);

	ros::Time time = ros::Time::now();
	geometry_msgs::Vector3Stamped error_msg;
	error_msg.header.frame_id = "map";
	error_msg.header.stamp = time;
	error_msg.vector.x = error(0);
	error_msg.vector.y = error(1);
	error_msg.vector.z = error(2);
	error_pub_.publish(error_msg);

	Eigen::Vector3d alt_error = error_calculator_ -> alternativeError(p_, p_ref_, v_, angular_velocity_, q_);

	geometry_msgs::Vector3Stamped alt_error_msg;
	alt_error_msg.header.frame_id = "map";
	alt_error_msg.header.stamp = time;
	alt_error_msg.vector.x = alt_error(0);
	alt_error_msg.vector.y = alt_error(1);
	alt_error_msg.vector.z = alt_error(2);
	error_pub_2_.publish(alt_error_msg);
	
	Eigen::Vector3d error_stats = v_ + q_.toRotationMatrix()*angular_velocity_.cross(d_);

	geometry_msgs::Vector3Stamped error_stats_msg;
	error_stats_msg.header.frame_id = "map";
	error_stats_msg.header.stamp = time;
	error_stats_msg.vector.x = error_stats(0);
	error_stats_msg.vector.y = error_stats(1);
	error_stats_msg.vector.z = error_stats(2);
	error_stats_.publish(error_stats_msg);

}

void ErrorMetricsNode::odometryCallback(const nav_msgs::Odometry& msg)
{
	p_ << msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z;
	v_ << msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z;
	q_.w() =  msg.pose.pose.orientation.w;
	q_.vec() << msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z;
	angular_velocity_ << msg.twist.twist.angular.x , msg.twist.twist.angular.y, msg.twist.twist.angular.z;
	//state_received_ = true;
}

void ErrorMetricsNode::trajectoryCallback(const riseq_msgs::FlatTrajectory& msg)
{
	p_ref_ << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
	/*
	v_ref_ << msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z;
	a_ref_ << msg.acc.x, msg.acc.y, msg.acc.z;
	j_ref_ << msg.jerk.x, msg.jerk.y, msg.jerk.z;
	j_ref_ << 0.,0.,0.;
	s_ref_ << msg.snap.x, msg.snap.y, msg.snap.z;

	q_ref_.w() = msg.pose.orientation.w;
	q_ref_.vec() << msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z;
	yaw_ref_ = msg.yaw;
	yaw_dot_ref_ = msg.yawdot;
	yaw_ddot_ref_ = msg.yawddot;
	thrust_ref_ = msg.thrust;

	angular_velocity_ref_ << msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z;
	angular_velocity_dot_ref_ << msg.ub.x, msg.ub.y, msg.ub.z;
	euler_dot_ref_ << msg.uc.x, msg.uc.y, msg.uc.z;
	torque_ref_ << msg.ux.x, msg.ux.y, msg.ux.z;
	trajectory_received_ = true;
	*/
}

ErrorMetricsNode::~ErrorMetricsNode()
{
	std::cout << "Absolute Trajectory Error: " << ate_ << std::endl;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "error_metrics_node");
	ros::NodeHandle nh("");

	ErrorMetricsNode * error_metrics_node = new ErrorMetricsNode(nh);
	ros::spin();
	std::cout << "Absolute Trajectory Error: " << error_metrics_node -> ate_ << std::endl;
	return 0;
}