#include "lifecycle_state_machine.hpp"
#include <stdexcept>

namespace apc16delft {
using State              = LifecycleStateMachine::State;
using Action             = LifecycleStateMachine::Action;

std::string toString(State const & state) {
	switch (state) {
		case State::unconfigured:     return "unconfigured";
		case State::inactive:         return "inactive";
		case State::active:           return "active";
		case State::finalized:        return "finalized";
		case State::configuring:      return "configuring";
		case State::cleaning_up:      return "cleaning_up";
		case State::activating:       return "activating";
		case State::deactivating:     return "deactivating";
		case State::shutting_down:    return "shutting_down";
		case State::error_processing: return "error_processing";
	}
	throw std::logic_error("Invalid state.");
}

std::string toString(Action const & action) {
	switch (action) {
		case Action::configure:  return "unconfigured";
		case Action::cleanup:    return "inactive";
		case Action::activate:   return "active";
		case Action::deactivate: return "finalized";
		case Action::shutdown:   return "configuring";
	}
	throw std::logic_error("Invalid action.");
}

bool LifecycleStateMachine::actionAllowed(State state, Action action) {
	switch (action) {
		case Action::configure:  return state == State::unconfigured;
		case Action::cleanup:    return state == State::configuring;
		case Action::activate:   return state == State::inactive;
		case Action::deactivate: return state == State::active;
		case Action::shutdown:   return state == State::unconfigured || state == State::inactive || state == State::active;
	}

	throw std::logic_error("Invalid action.");
}

State LifecycleStateMachine::transitionState(Action action) {
	switch (action) {
		case Action::configure:  return State::configuring;
		case Action::cleanup:    return State::cleaning_up;
		case Action::activate:   return State::activating;
		case Action::deactivate: return State::deactivating;
		case Action::shutdown:   return State::shutting_down;
	}

	throw std::logic_error("Invalid action.");
}

State LifecycleStateMachine::targetState(Action action) {
	switch (action) {
		case Action::configure:  return State::inactive;
		case Action::cleanup:    return State::unconfigured;
		case Action::activate:   return State::active;
		case Action::deactivate: return State::inactive;
		case Action::shutdown:   return State::finalized;
	}

	throw std::logic_error("Invalid action.");
}

std::function<int()> LifecycleStateMachine::actionCallback(Action action) {
	switch (action) {
		case Action::configure:  return on_configure;
		case Action::cleanup:    return on_cleanup;
		case Action::activate:   return on_activate;
		case Action::deactivate: return on_deactivate;
		case Action::shutdown:   return on_shutdown;
	}

	throw std::logic_error("Invalid action.");
}

LifecycleStateMachine::LifecycleStateMachine() {
	current_state_     = State::unconfigured;
	previous_state_    = State::unconfigured;
	transition_result_ = 0;
}

void LifecycleStateMachine::transition(State new_state, State old_state, int result_code) {
	// Update the internal state.
	current_state_     = new_state;
	previous_state_    = old_state;
	transition_result_ = result_code;

	// Invoke the transition callback if it is set.
	if (on_transition) on_transition(new_state, old_state, result_code);
}

int LifecycleStateMachine::transition(Action action) {
	// Make sure the transition is allowed.
	if (!actionAllowed(current_state_, action)) return -1; // TODO: use constants for error code.

	// Switch to the transition state.
	transition(transitionState(action), current_state_, 0);

	// Invoke the action callback.
	std::function<int()> callback = actionCallback(action);
	if (!callback) throw std::runtime_error("No action callback registered for action `" + toString(action) + "'");
	transition_result_ = callback();

	// If the transition failed, execute the error handler.
	if (transition_result_ != 0) {
		transition(State::error_processing, current_state_, transition_result_);
		State next_state = on_error && on_error() == 0 ? State::unconfigured : State::finalized;
		transition(next_state, current_state_, transition_result_);
		return transition_result_;
	}

	// Transition to the target state. Keep the last primary state as previous_state.
	transition(targetState(action), current_state_, 0);
	return 0;
}

}
