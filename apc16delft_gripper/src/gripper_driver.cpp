#include <ros/ros.h>
#include <apc16delft_util/managed_node.hpp>
#include <apc16delft_msgs/GripperStateStamped.h>
#include <apc16delft_msgs/SetGripperState.h>
#include <sensor_msgs/JointState.h>

#include "motoman_msgs/ReadSingleIO.h"
#include "motoman_msgs/WriteSingleIO.h"

namespace apc16delft {

class GripperNode : public ManagedNode {

protected:
	ros::Rate publish_rate{10};
	std::string prefix = "gripper_";

	ros::Timer publish_timer;
	ros::Publisher gripper_publisher;
	ros::Publisher joint_publisher;
	ros::ServiceServer set_state_server;

	ros::ServiceClient read_single_io;   // handle for read_single_io service
	ros::ServiceClient write_single_io;   // handle for write_single_io service

	apc16delft_msgs::GripperStateStamped gripper_state;
	sensor_msgs::JointState joint_state;

	static constexpr double vacuum_angle   = 0.5 * M_PI;
	static constexpr double thumb_distance = 0.06;
	static constexpr double thumb_angle    = 0.5 * M_PI;
	static const int base_io_address       = 10010;
	static const int num_io                = 6;

	int io_state[num_io] = {0};

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
		read_single_io	  = node_handle.serviceClient<motoman_msgs::ReadSingleIO>("/read_single_io");
		write_single_io	  = node_handle.serviceClient<motoman_msgs::WriteSingleIO>("/write_single_io");

		gripper_publisher = node_handle.advertise<apc16delft_msgs::GripperStateStamped>("state", 1, false);
		joint_publisher   = node_handle.advertise<sensor_msgs::JointState>("joint_states", 1, false);
		set_state_server  = node_handle.advertiseService("set_state", &GripperNode::onSetState, this);

		// Read IO states from motoman driver.
		for (int io_index = 0; io_index < num_io; io_index++) {
			motoman_msgs::ReadSingleIO srv;
			srv.request.address = base_io_address + io_index;
			if (read_single_io.call(srv)) {
				io_state[io_index] = srv.response.value;
			} else {
				ROS_ERROR_STREAM("Could not read IO states from motoman driver.");
			}
		}

		// Update our internal gripper state.
		gripper_state.state.led_enabled    = io_state[0];
		gripper_state.state.vacuum_enabled = io_state[1];
		gripper_state.state.thumb_extended = io_state[3];
		gripper_state.state.vacuum_rotated = io_state[5];

		publish_timer = node_handle.createTimer(publish_rate, &GripperNode::onPublishTimeout, this);
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

		
		motoman_msgs::WriteSingleIO srv;
		gripper_state.state = request.state;
		
		// Vacuum rotation.
		joint_state.position[0] = gripper_state.state.vacuum_rotated ? vacuum_angle   : 0;
		srv.request.address = base_io_address + 5;
		srv.request.value   = gripper_state.state.vacuum_rotated ? 1 : 0;
		write_single_io.call(srv);

		joint_state.position[1] = gripper_state.state.thumb_extended ? thumb_distance : 0;
		srv.request.address = base_io_address + 3;
		srv.request.value   = gripper_state.state.thumb_extended ? 1 : 0;
		write_single_io.call(srv);
		
		joint_state.position[2] = gripper_state.state.thumb_rotated  ? thumb_angle    : 0;

		// Vacuum enabled.
		srv.request.address = base_io_address + 1;
		srv.request.value   = gripper_state.state.vacuum_enabled ? 1 : 0;
		write_single_io.call(srv);

		// Led enabled.
		srv.request.address = base_io_address + 0;
		srv.request.value   = gripper_state.state.led_enabled ? 1 : 0;
		write_single_io.call(srv);

		response.error = 0;

		return true;
	}


};
} // namespace

int main(int argc, char * * argv) {
	ros::init(argc, argv, "gripper_driver");
	apc16delft::GripperNode node;

	ros::spin();
}

