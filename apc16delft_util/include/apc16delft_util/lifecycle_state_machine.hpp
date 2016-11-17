#pragma once
#include <string>
#include <functional>

namespace apc16delft {

/// The possible states for a managed node.
enum class LifecycleState {
	unconfigured,
	inactive,
	active,
	finalized,
	configuring,
	cleaning_up,
	shutting_down,
	activating,
	deactivating,
	error_processing
};

/// The possible actions for a managed node.
enum class LifecycleAction {
	configure,
	cleanup,
	activate,
	deactivate,
	shutdown,
};

/// Convert state to string for printing the state
std::string toString(LifecycleState const & state);

/// Convert action to string for printing the state
std::string toString(LifecycleAction const & action);

/// A ROS node virtual class with lifecycle functions.
class LifecycleStateMachine {
public:
	using State  = LifecycleState;
	using Action = LifecycleAction;

	/// Callback to invoke when the state machine performs a state transition.
	std::function<void (State new_state, State old_state, int result)> on_transition;

	/// Callback to invoke when the configure action is executed.
	std::function<int ()> on_configure;

	/// Callback to invoke when the cleanup action is executed.
	std::function<int ()> on_cleanup;

	/// Callback to invoke when the activate action is executed.
	std::function<int ()> on_activate;

	/// Callback to invoke when the deactivate action is executed.
	std::function<int ()> on_deactivate;

	/// Callback to invoke when the shutdown action is executed.
	std::function<int ()> on_shutdown;

	/// Callback to invoke when a state transition fails.
	std::function<int ()> on_error;

private:
	/// The current state of the managed node.
	State current_state_;

	/// The previous state of the managed node.
	State previous_state_;

	/// The result code of the last transition.
	int transition_result_;

	/// Check if an action is allowed in a given state.
	static bool actionAllowed(State state, Action action);

	/// Get the transition state for a given action.
	static State transitionState(Action action);

	/// Get the target state for a given action.
	static State targetState(Action action);

public:
	/// Default constructor.
	LifecycleStateMachine();

	/// Try to perform an action.
	int transition(Action action);

	/// Get the current state.
	State currentState() const {
		return current_state_;
	}

	/// Get the previous state.
	State previousState() const {
		return previous_state_;
	}

	/// Get the result code.
	int transitionResult() const {
		return transition_result_;
	}

private:
	/// Execute a transition.
	void transition(State new_state, State old_state, int result_code);

	/// Get the callback associated with an action.
	std::function<int()> actionCallback(Action action);
};

}
