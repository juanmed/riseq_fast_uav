#include "error_metrics.h"

ErrorMetrics::ErrorMetrics(const Eigen::Vector3d d){
	error_norm_acc_ = 0.0;
	mass_ = 0.0;
	d_ = d;
	samples_ = 0.0;
	ones_ = Eigen::Vector3d::Ones();
}

Eigen::Vector3d ErrorMetrics::vector_error(Eigen::Vector3d a, Eigen::Vector3d b){
	return a - b;
}

double ErrorMetrics::vectorErrorMagnitude(Eigen::Vector3d a, Eigen::Vector3d b){
	return (a - b).norm();
}

double ErrorMetrics::absoluteTrajectoryError(Eigen::Vector3d a, Eigen::Vector3d b){
	error_norm_acc_ = error_norm_acc_ + this -> vectorErrorMagnitude(a,b);
	return std::sqrt(error_norm_acc_ / samples_);
}

Eigen::Vector3d ErrorMetrics::alternativeError(const Eigen::Vector3d a, const Eigen::Vector3d b,
	const Eigen::Vector3d v, const Eigen::Vector3d angular_velocity, const Eigen::Quaterniond q)
{
	Eigen::Vector3d speed_factor = v + q.toRotationMatrix() * angular_velocity.cross(d_);
	Eigen::Vector3d alt_error = (a - b); //.cwiseQuotient(ones_ + speed_factor.cwiseAbs());
	for (int i = 0; i <3; i ++){
		alt_error(i) = alt_error(i) / std::sqrt(1. + std::fabs(speed_factor(i)));
	}
	return alt_error;
}

void ErrorMetrics::increaseSamples()
{
	samples_ += 1;
}

ErrorMetrics::~ErrorMetrics()
{
	
}