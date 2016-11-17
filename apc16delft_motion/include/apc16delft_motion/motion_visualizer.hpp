#pragma once
#include<ros/ros.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/move_group_interface/move_group.h>

namespace apc16delft {

class MotionVisualizer {
private:
	ros::Publisher trajectory_publisher_;
	ros::Timer publish_timer_;
	ros::Rate publish_rate_{10};
	moveit_msgs::DisplayTrajectory display_trajectory_;
public:
	MotionVisualizer (ros::NodeHandle &nh)
	{
		trajectory_publisher_ = nh.advertise<moveit_msgs::DisplayTrajectory>("display_trajectory", 1, true);
		publish_timer_ = nh.createTimer(publish_rate_, &MotionVisualizer::publishTrajectory,this);
	};

	void publishTrajectory(ros::TimerEvent const &) {
		//display_trajectory_.trajectory_start.joint_state.header.stamp = ros::Time::now();
		//trajectory_publisher_.publish(display_trajectory_);
	}


	void displayTrajectory(move_group_interface::MoveGroup::Plan &executable_plan) {
		display_trajectory_.trajectory_start.joint_state.header.stamp = ros::Time::now();
		display_trajectory_.trajectory_start = executable_plan.start_state_;
		clearTrajectoryPublisher();
		display_trajectory_.trajectory.push_back(executable_plan.trajectory_);
		trajectory_publisher_.publish(display_trajectory_);
	}

	void clearTrajectoryPublisher() {
		display_trajectory_.trajectory.clear();
	}
};

} //namespace

