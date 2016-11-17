#include <motoman_msgs/ReadSingleIO.h>
#include <motoman_msgs/WriteSingleIO.h>

#include <ros/ros.h>

namespace apc16delft {

class FakeRobotIo {
private:
	ros::NodeHandle node;

	struct {
		ros::ServiceServer read;
		ros::ServiceServer write;
	} servers;

public:
	FakeRobotIo() {
		servers.read  = node.advertiseService("read_single_io", &FakeRobotIo::onReadSingleIo, this);
		servers.write = node.advertiseService("write_single_io", &FakeRobotIo::onWriteSingleIo, this);
	}

private:
	bool onReadSingleIo(motoman_msgs::ReadSingleIO::Request & request, motoman_msgs::ReadSingleIO::Response & response) {
		response.value = true;
		ROS_INFO_STREAM("Read fake IO: " << request.address << ", returning true.");
		return true;
	}
	bool onWriteSingleIo(motoman_msgs::WriteSingleIO::Request & request, motoman_msgs::WriteSingleIO::Response &) {
		ROS_INFO_STREAM("Setting fake IO: " << request.address << " to " << (request.value ? "true" : "false") << ".");
		return true;
	}
};


}

int main(int argc, char * * argv) {
	ros::init(argc, argv, ROS_PACKAGE_NAME "_io");
	apc16delft::FakeRobotIo node;
	ros::spin();
}
