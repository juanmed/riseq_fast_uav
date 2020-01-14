
#include <Eigen/Dense>


class FeedbackLinearizationController
{
	public:

		Eigen::Vector3d computeDesiredThrust(Eigen::Vector3d p, Eigen::Vector3d p_ref);
		Eigen::Matrix3d computeDesiredOrientation();
		Eigen::Vector3d computeDesiredBodyRate();


	private:

		double mass_;
		double max_thrust_;
		double min_thrust_;		
}

