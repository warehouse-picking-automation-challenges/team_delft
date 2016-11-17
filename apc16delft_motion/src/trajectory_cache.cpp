#include "yaml.hpp"
#include "trajectory_cache.hpp"
#include "master_pose.hpp"
#include <apc16delft_msgs/MasterPoseDescriptor.h>

namespace apc16delft {

TrajectoryCache::TrajectoryCache() {}

TrajectoryCache::TrajectoryCache(const std::string & directory) {
	loadTrajectoriesFromDirectory(directory);
	ROS_INFO_STREAM("Created a cache with contents from " << directory << ".");
}

trajectory_msgs::JointTrajectory TrajectoryCache::lookupTrajectory(apc16delft_msgs::MasterPoseDescriptor const & from, apc16delft_msgs::MasterPoseDescriptor const & to) {
	ROS_INFO_STREAM("Looking up trajectory from '" << from << "' to '" << to << "'.");
	auto i = trajectories_.find(Key{from, to});
	if (i == trajectories_.end()) {
		ROS_INFO_STREAM("Plan not found in cache, returning empty trajectory.");
		trajectory_msgs::JointTrajectory result;
		return result;
	}

	ROS_INFO_STREAM("Requested trajectory found in cache.");
	return i->second;
}

void TrajectoryCache::loadTrajectoriesFromDirectory(const std::string & directory) {
	MasterPoseDescriptor home;
	MasterPoseDescriptor tote;
	MasterPoseDescriptor tote_drop;
	MasterPoseDescriptor bin;
	MasterPoseDescriptor cam;
	MasterPoseDescriptor cam_src;
	MasterPoseDescriptor cam_dst;

	home.type = MasterPoseDescriptor::TYPE_HOME;
	home.group_name = MasterPoseDescriptor::GROUP_TOOL0;


	tote_drop.type = MasterPoseDescriptor::TYPE_TOTE;

	tote.type = MasterPoseDescriptor::TYPE_TOTE;
	// bin_index is used for the inner tote poses
	tote.bin_index = 0; 
	// tote.group_name = MasterPoseDescriptor::GROUP_TOOL0;
	
	bin.type  = MasterPoseDescriptor::TYPE_BIN;
	cam.type  = MasterPoseDescriptor::TYPE_CAM;

	loadTrajectory(directory, home, tote);
	loadTrajectory(directory, tote, home);

	for (int i = 0; i < 12; ++i) {
		bin.bin_index = i;
		cam.bin_index = i;
		loadTrajectory(directory, home, bin);
		loadTrajectory(directory, bin,  home);
		loadTrajectory(directory, tote, bin);
		loadTrajectory(directory, bin,  tote);

		loadTrajectory(directory, home, cam);
		loadTrajectory(directory, cam,  home);
		loadTrajectory(directory, tote, cam);
		loadTrajectory(directory, cam,  tote);
		
		loadTrajectory(directory, bin, cam);
		loadTrajectory(directory, cam, bin);
	}
	//Load cam to cam trajectories
	cam_src.type  = MasterPoseDescriptor::TYPE_CAM;
	cam_dst.type  = MasterPoseDescriptor::TYPE_CAM;
	for (size_t src_idx = 0; src_idx < 12 ; src_idx++) {
		for (size_t dst_idx = 0; dst_idx < 12; dst_idx++) {
			if(src_idx == dst_idx) {
				continue;
			} else {
				cam_src.bin_index = src_idx;
				cam_dst.bin_index = dst_idx;
				loadTrajectory(directory, cam_src, cam_dst);
			}
		}
	}

	// load vertical totes (also used for pinched items)
	for (int i = 0; i<6; i++){
		tote_drop.bin_index = i+1;
		tote_drop.group_name = apc16delft_msgs::MasterPoseDescriptor::GROUP_TOOL1;
		loadTrajectory(directory, tote, tote_drop);
		loadTrajectory(directory, tote_drop, tote);
	}

	// load horizontal totes
	for (int i = 0; i<6; i++){
		tote_drop.bin_index = i+1;
		tote_drop.group_name = apc16delft_msgs::MasterPoseDescriptor::GROUP_TOOL0;
		loadTrajectory(directory, tote, tote_drop);
		loadTrajectory(directory, tote_drop, tote);
	}

	ROS_INFO_STREAM("Successfully loaded " << trajectories_.size() << " trajectories");
}

bool TrajectoryCache::loadTrajectory(std::string const & directory, MasterPoseDescriptor const & from, MasterPoseDescriptor const & to) {
	std::string filename = masterPoseToString(from) + "_to_" + masterPoseToString(to) + ".yaml";
	YAML::Node node;
	try {
		node =  YAML::LoadFile(directory + "/" + filename);
	} catch (YAML::BadFile) {
		ROS_WARN_STREAM("File `" << filename << "' not found.");
		return false;
	}
	trajectories_[Key{from, to}] = node["trajectory"].as<trajectory_msgs::JointTrajectory>();
	return true;
}

}
