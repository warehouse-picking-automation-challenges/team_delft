#pragma once
#include "lifecycle_state_machine.hpp"

#include <ros/ros.h>

namespace apc16delft {

/// A ROS node virtual class with lifecycle functions.
class ManagedNode {
protected:
	/// The ROS node handle;
	ros::NodeHandle node_handle;

private:
	/// The state machine for the internal node state.
	LifecycleStateMachine state_machine_;

	/// Publisher to publisher the managed node state.
	ros::Publisher state_publisher_;

	struct {
		ros::ServiceServer configure;
		ros::ServiceServer cleanup;
		ros::ServiceServer activate;
		ros::ServiceServer deactivate;
		ros::ServiceServer shutdown;
		ros::ServiceServer destroy;
	} servers_;

public:
	/// Construct a managed node.
	ManagedNode();

	/// Virtual destructor.
	virtual ~ManagedNode();

	LifecycleStateMachine const & state_machine() const { return state_machine_; }

	int configure()  { return transition(LifecycleAction::configure);  }
	int cleanup()    { return transition(LifecycleAction::cleanup);    }
	int activate()   { return transition(LifecycleAction::activate);   }
	int deactivate() { return transition(LifecycleAction::deactivate); }
	int shutdown()   { return transition(LifecycleAction::shutdown);   }
	int destroy();

protected:
	/// Called when configure is requested.
	virtual int onConfigure();

	/// Called when cleanup is requested.
	virtual int onCleanup();

	/// Called upon node activation.
	virtual int onActivate();

	/// Called upon node deactivation.
	virtual int onDeactivate();

	/// Called when node shutdown is requested.
	virtual int onShutdown();

	/// Called when an error is raised.
	virtual int onError();

private:
	/// Attempt to transition to a new state.
	int transition(LifecycleAction action);

	/// Called when the state machine performs a transition.
	void onTransition(LifecycleState new_state, LifecycleState old_state, int result_code);
};

}
