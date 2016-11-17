#include <ros/ros.h>
#include <apc16delft_util/managed_node.hpp>
#include <apc16delft_msgs/GripperStateStamped.h>
#include <apc16delft_msgs/SetGripperState.h>
#include <sensor_msgs/JointState.h>

namespace apc16delft {

class GripperNode : public ManagedNode {
	ros::Rate publish_rate{10};
	std::string prefix = "gripper_";

	ros::Timer publish_timer;
	ros::Publisher gripper_publisher;
	ros::Publisher joint_publisher;
	ros::ServiceServer set_state_server;

	apc16delft_msgs::GripperStateStamped gripper_state;
	sensor_msgs::JointState joint_state;

	static constexpr double vacuum_angle   = 0.5 * M_PI;
	static constexpr double thumb_distance = 0.06;
	static constexpr double thumb_angle    = 0.5 * M_PI;

	int onConfigure() override {
		int publish_rate = 10;
		node_handle.getParam("publish_rate", publish_rate);
		node_handle.getParam("prefix",       prefix);

		joint_state.name = {
			prefix + "vacuum",
			prefix + "finger_slider",
			prefix + "finger",
		};

		joint_state.effort.assign(3, 0);
		joint_state.position.assign(3, 0);
		joint_state.velocity.assign(3, 0);

		this->publish_rate = ros::Rate(publish_rate);
		return 0;
	}

	int onActivate() override {
		gripper_publisher = node_handle.advertise<apc16delft_msgs::GripperStateStamped>("state", 1, false);
		joint_publisher   = node_handle.advertise<sensor_msgs::JointState>("joint_states", 1, false);
		set_state_server  = node_handle.advertiseService("set_state", &GripperNode::onSetState, this);
		publish_timer     = node_handle.createTimer(publish_rate, &GripperNode::onPublishTimeout, this);
		return 0;
	}

	int onDeactivate() override {
		publish_timer.stop();
		gripper_publisher.shutdown();
		set_state_server.shutdown();
		return 0;
	}

	void onPublishTimeout(ros::TimerEvent const &) {
		ros::Time now = ros::Time::now();
		gripper_state.header.stamp = now;
		joint_state.header.stamp   = now;

		gripper_publisher.publish(gripper_state);
		joint_publisher.publish(joint_state);
	}

	bool onSetState(apc16delft_msgs::SetGripperState::Request & request, apc16delft_msgs::SetGripperState::Response & response) {
		gripper_state.state = request.state;
		joint_state.position[0] = gripper_state.state.vacuum_rotated ? vacuum_angle   : 0;
		joint_state.position[1] = gripper_state.state.thumb_extended ? thumb_distance : 0;
		joint_state.position[2] = gripper_state.state.thumb_rotated  ? thumb_angle    : 0;
		response.error = 0;
		return true;
	}

};

}

int main(int argc, char * * argv) {
	ros::init(argc, argv, ROS_PACKAGE_NAME "_gripper");
	apc16delft::GripperNode node;
	node.configure();
	node.activate();

	ros::spin();
}

