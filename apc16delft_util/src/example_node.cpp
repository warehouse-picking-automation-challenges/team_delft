#include "managed_node.hpp"

#include <ros/init.h>

namespace apc16delft {

class ExampleNode : public ManagedNode {};

}


int main(int argc, char ** argv) {
	ros::init(argc, argv, "example_node");
	apc16delft::ExampleNode example_node;
	ros::spin();
}

