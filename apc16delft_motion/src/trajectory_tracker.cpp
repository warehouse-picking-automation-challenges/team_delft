#include "trajectory_tracker.hpp"

#include <std_msgs/Int32.h>

#include <ros/console.h>

#include <algorithm>
#include <cmath>

namespace apc16delft {

namespace {
	/// Calculate the euclidean distance between two vectors of doubles.
	double euclidianDistance(std::vector<double> const & a, std::vector<double> const & b) {
		if (a.size() != b.size()) throw std::range_error("a.size() != b.size()");
		double sum_of_squares = 0;
		for (std::size_t i = 0; i < a.size(); ++i) {
			double difference = a[i] - b[i];
			sum_of_squares += difference * difference;
		}
		return std::sqrt(sum_of_squares);
	}
}

TrajectoryTracker::TrajectoryTracker(ros::NodeHandle & node, std::string const & joint_topic, std::string const & progress_topic, double target_threshold) {
	ROS_INFO_STREAM("TrajectoryTracker: Listening to `" << joint_topic << "' and publishing to `" << progress_topic << "'");
	progres_publisher_ = node.advertise<std_msgs::Int32>(progress_topic, 1);

	ros::SubscribeOptions subscribe;
	subscribe.init<sensor_msgs::JointState>(joint_topic, 1, boost::bind(&TrajectoryTracker::onJointUpdate_, this, _1));
	subscribe.callback_queue = spinner_.queue();
	joint_subscriber_        = node.subscribe(subscribe);
	target_threshold_        = target_threshold;
	spinner_.start();
}

void TrajectoryTracker::startTracking(trajectory_msgs::JointTrajectory const & new_trajectory) {
	std::lock_guard<std::mutex> guard(mutex_);
	current_waypoint_ = 0;
	trajectory_       = new_trajectory;
	running_          = true;
	known_state_.assign(new_trajectory.joint_names.size(), std::numeric_limits<double>::quiet_NaN());
}

void TrajectoryTracker::stopTracking() {
	std::lock_guard<std::mutex> guard(mutex_);
	running_ = false;
	std_msgs::Int32 msg;
	msg.data = trajectory_.points.size() - 1;
	progres_publisher_.publish(msg);
}

void TrajectoryTracker::onJointUpdate_(const sensor_msgs::JointState::ConstPtr& joint_state) {
	std::lock_guard<std::mutex> guard(mutex_);

	if (!running_) return;

	// Reorder joint state to match the trajectory.
	for (std::size_t i = 0; i < joint_state->name.size(); ++i) {
		std::string const & name = joint_state->name[i];
		auto target = std::find(trajectory_.joint_names.begin(), trajectory_.joint_names.end(), name);
		if (target == trajectory_.joint_names.end()) {
			ROS_WARN_STREAM("TrajectoryTracker: Failed to find joint `" << name << "' from the joint state in the trajectory being tracked.");
			return;
		}
		known_state_[target - trajectory_.joint_names.begin()] = joint_state->position.at(i);
	}

	// Check that all joints are known.
	for (double joint : known_state_) {
		if (std::isnan(joint)) {
			ROS_WARN_STREAM("TrajectoryTracker: There are still unknown joints.");
			return;
		}
	}

	// Follow trajectory up to the current waypoint.
	while (true) {
		double distance = euclidianDistance(known_state_, trajectory_.points[current_waypoint_].positions);
		ROS_DEBUG_STREAM("TrajectoryTracker: Distance to next waypoint: " << distance << "/" << target_threshold_);
		if (distance > target_threshold_) return;

		std_msgs::Int32 msg;
		msg.data = current_waypoint_++;
		progres_publisher_.publish(msg);

		if (current_waypoint_ >= trajectory_.points.size()) {
			ROS_INFO_STREAM("TrajectoryTracker: Trajectory finished at waypoint: " << (current_waypoint_ - 1));
			running_ = false;
			return;
		}
	}
}

}
