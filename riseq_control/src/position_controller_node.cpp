#include "riseq_control/position_controller.h"

int main(int argc, char** argv){

	ros::init(argc, argv, "position_controller");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");

	PositionController *controller = new PositionController(nh, nh_private);
	ros::spin();
	return 0;
}