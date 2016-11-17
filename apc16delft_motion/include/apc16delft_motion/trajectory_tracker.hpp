#pragma once
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>

#include <ros/callback_queue.h>
#include <ros/node_handle.h>

#include <thread>
#include <atomic>
#include <mutex>

namespace apc16delft {

class NonRetardedAsyncSpinner {
private:
	std::thread thread_;
	ros::CallbackQueue queue_;
	std::atomic_flag running_ = ATOMIC_FLAG_INIT;
	std::atomic_bool stop_;

public:
	~NonRetardedAsyncSpinner() {
		stop();
		join();
	}

	ros::CallbackQueueInterface * queue() {
		return &queue_;
	}

	void start() {
		if (running_.test_and_set()) return;
		stop_ = false;
		thread_ = std::thread([this] () {
			work_();
		});
	}

	void stop() {
		stop_ = true;
	}

	void join() {
		if (!thread_.joinable()) return;
		thread_.join();
	}

private:
	void work_() {
		while (!stop_) {
			queue_.callAvailable(ros::WallDuration(0.1));
		}
		running_.clear();
	}
};

class TrajectoryTracker {
private:
	/// Private spinner with dedicated queue for handling incoming messages.
	NonRetardedAsyncSpinner spinner_;

	/// The joint subscription.
	ros::Subscriber joint_subscriber_;

	/// The progress publisher.
	ros::Publisher progres_publisher_;

	/// Mutex to lock for accesing internal state from multiple threads.
	std::mutex mutex_;

	/// The last known state of the joints.
	std::vector<double> known_state_;

	/// Flag to indicate if the tracker is active.
	bool running_;

	/// Threshold to consider joint states equal (based on euclidian distance in joint space).
	double target_threshold_;

	/// The last seen waypoint.
	std::size_t current_waypoint_;

	/// The trajectory to track.
	trajectory_msgs::JointTrajectory trajectory_;

public:
	/// Construct a waypoint tracker with given node handle and topic names.
	TrajectoryTracker(ros::NodeHandle & node, std::string const & joint_topic, std::string const & progress_topic, double threshold = 0.01);

	/// Start tracking a trajectory.
	void startTracking(trajectory_msgs::JointTrajectory const & new_trajectory);

	/// Stop tracking trajectories.
	void stopTracking();

private:
	/// Called when a new joint state is received.
	void onJointUpdate_(sensor_msgs::JointState::ConstPtr const & jointstate);
};

}
