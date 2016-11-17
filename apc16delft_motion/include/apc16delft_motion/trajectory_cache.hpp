#include <utility>
#include <map>
#include <string>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <apc16delft_msgs/MasterPoseDescriptor.h>
#include "master_pose.hpp"
#include <dirent.h>

namespace apc16delft {

class TrajectoryCache {
	using MasterPoseDescriptor = apc16delft_msgs::MasterPoseDescriptor;
	using Key = std::tuple<MasterPoseDescriptor, MasterPoseDescriptor>;
protected:
	std::map<Key, trajectory_msgs::JointTrajectory> trajectories_;

public:
	/// Construct a trajectory cache data container.
	TrajectoryCache();

	/// Construct a trajectory cache data container and fill the cache with trajectories from a yaml database.
	TrajectoryCache(const std::string & database_directory);

	/// Lookup a JointTrajectory in the cache from a source MasterPose to dst MasterPose.
	trajectory_msgs::JointTrajectory lookupTrajectory(apc16delft_msgs::MasterPoseDescriptor const & from, apc16delft_msgs::MasterPoseDescriptor const & to);

protected:
	void loadTrajectoriesFromDirectory(std::string const & directory);
	bool loadTrajectory(std::string const & directory, MasterPoseDescriptor const & from, MasterPoseDescriptor const & to);
};

} // namespace

