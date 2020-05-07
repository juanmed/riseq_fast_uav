#include "riseq_control/position_controller.h"
#include "riseq_control/fast_controller_node.h"


int main(int argc, char** argv){

	ros::init(argc, argv, "position_controller");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");

	FastControllerNode *controller = new FastControllerNode(nh, nh_private);
	ros::spin();
	return 0;
}