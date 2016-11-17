#pragma once
#include <yaml-cpp/yaml.h>

#include <ros/duration.h>
#include <ros/time.h>
#include <ros/header.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <cstdint>
namespace YAML {

template<>
struct convert<ros::Duration> {
	static bool decode(const Node & node, ros::Duration & result);
};

template<>
struct convert<ros::Time> {
	static bool decode(const Node & node, ros::Time & result);
};

template<>
struct convert<ros::Header> {
	static bool decode(const Node & node, std_msgs::Header & result);
};

template<>
struct convert<trajectory_msgs::JointTrajectoryPoint> {
	static bool decode(const Node & node, trajectory_msgs::JointTrajectoryPoint & result);
};

template<>
struct convert<trajectory_msgs::JointTrajectory> {
	static bool decode(const Node& node, trajectory_msgs::JointTrajectory & result);
};

}
