#include "riseq_control/position_controller.h"
#include "riseq_control/fast_controller_node.h"


int main(int argc, char** argv){

	ros::init(argc, argv, "position_controller");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");

	std::string controller;
  if (!ros::param::get("riseq/controller_type", controller)){
      std::cout << "Did not get controller_type from the params, defaulting to FLC" << std::endl;
      controller = "flc";
  }

  if (controller == "flc")
  {
		PositionController *controller = new PositionController(nh, nh_private);
  }
  else if (controller == "fc")
  {
		FastControllerNode *controller = new FastControllerNode(nh, nh_private);
  }
  else
  {
   	std::cout << "**********************************************" << std::endl;
  	std::cout << "Controller " << controller << " not recognized!" << std::endl;
    std::cout << "**********************************************" << std::endl;
    return 0;
  }
	
	ros::spin();
	return 0;
}