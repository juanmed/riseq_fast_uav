#ifndef ERRORMETRICS_V12020_H
#define ERRORMETRICS_V12020_H

#include <iostream>
#include <Eigen/Geometry>

class ErrorMetrics{
public:
	ErrorMetrics(const Eigen::Vector3d d);
	virtual ~ ErrorMetrics();
	Eigen::Vector3d vector_error(const Eigen::Vector3d a, const Eigen::Vector3d b);
	double vectorErrorMagnitude(const Eigen::Vector3d a, const Eigen::Vector3d b);
	double absoluteTrajectoryError(const Eigen::Vector3d a, const Eigen::Vector3d b);
	Eigen::Vector3d alternativeError(const Eigen::Vector3d a, const Eigen::Vector3d b,
		const Eigen::Vector3d v, const Eigen::Vector3d angular_velocity, const Eigen::Quaterniond q);
	void increaseSamples();
private:
	double error_norm_acc_;
	double mass_;
	Eigen::Vector3d d_;
	long samples_;
	Eigen::Vector3d ones_;
};

#endif /* ERRORMETRICS_V12020_H */
