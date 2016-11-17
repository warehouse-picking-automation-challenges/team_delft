#include "managed_node.hpp"

#include <apc16delft_msgs/LifecycleState.h>
#include <apc16delft_msgs/RequestTransition.h>

#include <boost/function.hpp>
#include <functional>
#include <stdexcept>

namespace apc16delft {

int toMsg(LifecycleState state) {
	switch (state) {
		case LifecycleState::unconfigured:     return apc16delft_msgs::LifecycleState::UNCONFIGURED;
		case LifecycleState::inactive:         return apc16delft_msgs::LifecycleState::INACTIVE;
		case LifecycleState::active:           return apc16delft_msgs::LifecycleState::ACTIVE;
		case LifecycleState::finalized:        return apc16delft_msgs::LifecycleState::FINALIZED;
		case LifecycleState::configuring:      return apc16delft_msgs::LifecycleState::CONFIGURING;
		case LifecycleState::cleaning_up:      return apc16delft_msgs::LifecycleState::CLEANING_UP;
		case LifecycleState::shutting_down:    return apc16delft_msgs::LifecycleState::SHUTTING_DOWN;
		case LifecycleState::activating:       return apc16delft_msgs::LifecycleState::ACTIVATING;
		case LifecycleState::deactivating:     return apc16delft_msgs::LifecycleState::DEACTIVATING;
		case LifecycleState::error_processing: return apc16delft_msgs::LifecycleState::ERROR_PROCESSING;
	}
	throw std::logic_error("Invalid lifecycle state.");
}

using ServiceHandler    = boost::function<bool (apc16delft_msgs::RequestTransition::Request &, apc16delft_msgs::RequestTransition::Response &)>;
using TransitionHandler = int (ManagedNode::*)();

ServiceHandler makeServiceHandler(LifecycleStateMachine & state_machine, ManagedNode & node, TransitionHandler handler) {
	return [&state_machine, &node, handler] (apc16delft_msgs::RequestTransition::Request &, apc16delft_msgs::RequestTransition::Response & response) {
		try {
			response.result_code = (node.*handler)();
			return true;
		} catch(...) {
			std::terminate();
		}
	};
}

ManagedNode::ManagedNode() : node_handle("~") {
	state_machine_.on_transition = std::bind(&ManagedNode::onTransition, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	state_machine_.on_error      = std::bind(&ManagedNode::onError,      this);
	state_machine_.on_configure  = std::bind(&ManagedNode::onConfigure,  this);
	state_machine_.on_cleanup    = std::bind(&ManagedNode::onCleanup,    this);
	state_machine_.on_activate   = std::bind(&ManagedNode::onActivate,   this);
	state_machine_.on_deactivate = std::bind(&ManagedNode::onDeactivate, this);
	state_machine_.on_shutdown   = std::bind(&ManagedNode::onShutdown,   this);

	state_publisher_    = node_handle.advertise<apc16delft_msgs::LifecycleState>("lifecycle/status", 10, true);
	servers_.configure  = node_handle.advertiseService("lifecycle/configure",  makeServiceHandler(state_machine_, *this, &ManagedNode::configure));
	servers_.cleanup    = node_handle.advertiseService("lifecycle/cleanup",    makeServiceHandler(state_machine_, *this, &ManagedNode::cleanup));
	servers_.activate   = node_handle.advertiseService("lifecycle/activate",   makeServiceHandler(state_machine_, *this, &ManagedNode::activate));
	servers_.deactivate = node_handle.advertiseService("lifecycle/deactivate", makeServiceHandler(state_machine_, *this, &ManagedNode::deactivate));
	servers_.shutdown   = node_handle.advertiseService("lifecycle/shutdown",   makeServiceHandler(state_machine_, *this, &ManagedNode::shutdown));
	servers_.destroy    = node_handle.advertiseService("lifecycle/destroy",    makeServiceHandler(state_machine_, *this, &ManagedNode::destroy));

	apc16delft_msgs::LifecycleState state;
	state.state       = toMsg(state_machine_.currentState());
	state.old_state   = toMsg(state_machine_.previousState());
	state.result_code = state_machine_.transitionResult();
	state_publisher_.publish(state);
}

ManagedNode::~ManagedNode() {}

int ManagedNode::destroy() {
	if (state_machine_.currentState() != LifecycleState::finalized) return -1;
	ROS_INFO_STREAM("Destroying node.");
	ros::shutdown();
	return 0;
}

int ManagedNode::onConfigure() {
	ROS_INFO_STREAM("Default onConfigure called.");
	return 0;
}

int ManagedNode::onCleanup() {
	ROS_INFO_STREAM("Default onCleanup called.");
	return 0;
}

int ManagedNode::onActivate() {
	ROS_INFO_STREAM("Default onActivated called.");
	return 0;
}

int ManagedNode::onDeactivate() {
	ROS_INFO_STREAM("Default onDeactivated called.");
	return 0;
}

int ManagedNode::onError() {
	ROS_WARN_STREAM("Default onError called.");
	return -2; // TODO: Use constants for error codes.
}

int ManagedNode::onShutdown() {
	ROS_INFO_STREAM("Default onShutdown called.");
	return 0;
}

int ManagedNode::transition(LifecycleAction action) {
	return state_machine_.transition(action);
}

void ManagedNode::onTransition(LifecycleState new_state, LifecycleState old_state, int result_code) {
	ROS_INFO_STREAM("Node transitioned from " << toString(old_state) << " to " << toString(new_state) << " with code " << result_code);
}


}
