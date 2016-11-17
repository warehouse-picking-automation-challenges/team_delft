#include "yaml.hpp"

namespace YAML {

bool convert<ros::Duration>::decode(const Node & node, ros::Duration & result) {
	std::int32_t seconds;
	std::int32_t nanoseconds;
	if (!convert<std::int32_t>::decode(node["secs"],  seconds    )) return false;
	if (!convert<std::int32_t>::decode(node["nsecs"], nanoseconds)) return false;
	result = ros::Duration(seconds, nanoseconds);
	return true;
}

bool convert<ros::Time>::decode(const Node & node, ros::Time & result) {
	std::int32_t seconds;
	std::int32_t nanoseconds;
	if (!convert<std::int32_t>::decode(node["secs"],  seconds)) return false;
	if (!convert<std::int32_t>::decode(node["nsecs"], nanoseconds)) return false;
	result = ros::Time(seconds, nanoseconds);
	return true;
}

bool convert<ros::Header>::decode(const Node & node, std_msgs::Header & result) {
		if (!convert<unsigned int>::decode(node["seq"], result.seq)) return false;
		if (!convert<std::string >::decode(node["frame_id"], result.frame_id)) return false;
		if (!convert<ros::Time>::decode(node["stamp"], result.stamp)) return false;
		return true;
	}

bool convert<trajectory_msgs::JointTrajectoryPoint>::decode(const Node & node, trajectory_msgs::JointTrajectoryPoint & result) {
	if (!convert<std::vector<double>>::decode(node["positions"], result.positions)) return false;
	if (!convert<std::vector<double>>::decode(node["velocities"], result.velocities)) return false;
	if (!convert<std::vector<double>>::decode(node["accelerations"], result.accelerations)) return false;
	if (!convert<std::vector<double>>::decode(node["effort"], result.effort)) return false;
	if (!convert<ros::Duration>::decode(node["time_from_start"], result.time_from_start)) return false;
	return true;
}

bool convert<trajectory_msgs::JointTrajectory>::decode(const Node& node, trajectory_msgs::JointTrajectory & result) {
	using TrajectoryPoints = std::vector<trajectory_msgs::JointTrajectoryPoint>;
	using JointNames       = std::vector<std::string>;

	if (!convert<ros::Header>::decode(node["header"], result.header)) return false;
	if (!convert<JointNames>::decode(node["joint_names"], result.joint_names)) return false;
	if (!convert<TrajectoryPoints>::decode(node["points"], result.points)) return false;
	return true;
}

}
