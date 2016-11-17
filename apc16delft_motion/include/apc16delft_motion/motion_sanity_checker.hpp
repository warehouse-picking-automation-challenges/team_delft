#pragma once
#include<ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_state/robot_state.h>
#include <trajectory_msgs/JointTrajectory.h>
namespace apc16delft {

class MotionSanityChecker {
private:
	boost::shared_ptr<sensor_msgs::JointState const> rail_state_, sia20_state_, all_states_;
	double topic_time_out_;
public:
	MotionSanityChecker () :
		topic_time_out_(0.5)
		{};

	bool checkTrajectorySanity(trajectory_msgs::JointTrajectory & motion_trajectory, moveit::planning_interface::MoveGroup* current_group) {
		//Check if we are running on real robot
		all_states_ = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", ros::Duration(topic_time_out_));
		if (!all_states_) {
			ROS_ERROR_STREAM("Joint states have not been published");
			return false;
		}
		if (all_states_->position.size() != 8) {
			ROS_INFO_STREAM("Working with the real robot..");
			//Wait for robot to stop.
			waitForRobotToStop ();
			//Get current joint values.
			double joint_diff;
			double distance = 0.0;
			sensor_msgs::JointState actual_state;
			//Combine the joint names from both groups.
			std::vector<std::string> joint_names;
			joint_names = (*rail_state_).name;
			joint_names.insert(joint_names.end(), (*sia20_state_).name.begin(), (*sia20_state_).name.end());

			//Combine the joint values from both groups.
			std::vector<double> joint_values;
			joint_values = (*rail_state_).position;
			joint_values.insert(joint_values.end(), (*sia20_state_).position.begin(), (*sia20_state_).position.end());
			//Compute distance between current state and the trajectory start state
			std::vector<std::string>::iterator it;
			for (size_t idx=0;idx < motion_trajectory.points[0].positions.size();idx++) {
				it = std::find(motion_trajectory.joint_names.begin(), motion_trajectory.joint_names.end(),joint_names[idx]);
				int pos_idx = std::distance(motion_trajectory.joint_names.begin(), it);

				joint_diff = joint_values[idx] - motion_trajectory.points[0].positions[pos_idx];
				//Adjust starting point of cached trajectory to current position for practical reasons.
				motion_trajectory.points[0].positions[pos_idx] = joint_values[idx];
				ROS_INFO_STREAM(joint_names[idx] <<": " << joint_values[idx]);
				distance += (joint_diff*joint_diff);
			}
			distance = sqrt(distance);

			if(distance < 1e-2) {
				//Trajectory safe to execute
				ROS_INFO_STREAM("Trajectory sanity check OK! Offset (" << distance <<") is within the specified tolerance of 0.01.");
				//robot_state::RobotStatePtr kinematic_state(current_group->getCurrentState());
				//kinematic_state->setVariablePositions(joint_names, joint_values);
				//current_group->setStartState(*kinematic_state);
				return true;
			} else {
				ROS_ERROR_STREAM("Motion safety check violation.");
				ROS_ERROR_STREAM("Starting configuration offset " << distance << "> 0.01.");
				return false;
			}
		} else {
			//TODO: This is basically repeated code. Refactoring required.
			ROS_INFO_STREAM("Working with simulated robot...");	
			std::vector<double> current_joint_values;
			double *joint_value_ptr;

			robot_state::RobotStatePtr kinematic_state(current_group->getCurrentState());
			//Get current joint values.
			double joint_diff;
			double distance = 0.0;
			joint_value_ptr = kinematic_state->getVariablePositions();
			const std::vector<std::string> joint_names= kinematic_state->getVariableNames();
			//Compute distance between current state and the trajectory start state
			std::vector<std::string>::iterator it;
			for (size_t idx=0;idx < motion_trajectory.points[0].positions.size();idx++) {
				it = std::find(motion_trajectory.joint_names.begin(), motion_trajectory.joint_names.end(),joint_names[idx]);
				int pos_idx = std::distance(motion_trajectory.joint_names.begin(), it);
				ROS_INFO_STREAM("src: " << joint_names[idx] << ": " << joint_value_ptr[idx] << ", dst: " << motion_trajectory.joint_names[pos_idx] << ": " << motion_trajectory.points[0].positions[pos_idx] << ".");

				joint_diff = joint_value_ptr[idx] - motion_trajectory.points[0].positions[pos_idx];
				//Adjust starting point of cached trajectory to current position for practical reasons.
				motion_trajectory.points[0].positions[pos_idx] = joint_value_ptr[idx];
				ROS_DEBUG_STREAM(joint_names[idx] <<": " << joint_value_ptr[idx]);
				distance += (joint_diff*joint_diff);
			}
			distance = sqrt(distance);

			if(distance < 1e-2) {
				//Trajectory safe to execute
				ROS_INFO_STREAM("Trajectory sanity check OK! Offset (" << distance <<") is within the specified tolerance of 0.01.");
				return true;
			} else {
				ROS_ERROR_STREAM("Motion safety check violation.");
				ROS_ERROR_STREAM("Starting configuration offset " << distance << "> 0.01.");
				return false;
			}
		}
	}
	bool waitForRobotToStop() {
		all_states_ = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", ros::Duration(topic_time_out_));
		if (!all_states_) {
			ROS_ERROR_STREAM("Joint states have not been published");
			return false;
		}
		if( (*all_states_).position.size() == 8) {
			// We are using simulated robot. We are Ok!
			return true;
		}
		else {
			// Get actual joint states of the robot.
			while (true) {
				rail_state_ = ros::topic::waitForMessage<sensor_msgs::JointState>("/sia20f/sia20f_b1_controller/joint_states", ros::Duration(topic_time_out_));
				sia20_state_ = ros::topic::waitForMessage<sensor_msgs::JointState>("/sia20f/sia20f_r1_controller/joint_states", ros::Duration(topic_time_out_));
				if (!rail_state_) {
					ROS_ERROR_STREAM("Rail states have not been published");
					continue;
				}
				if (!sia20_state_) {
					ROS_ERROR_STREAM("Robot states have not been published");
					continue;
				}
				break;
			}
			bool rail_has_stopped = false;
			std::vector<double> current_positions;
			while(!rail_has_stopped) {
				current_positions  = rail_state_->position;
				rail_state_        = ros::topic::waitForMessage<sensor_msgs::JointState>("/sia20f/sia20f_b1_controller/joint_states", ros::Duration(topic_time_out_));
				if (!rail_state_) {
					ROS_ERROR_STREAM("Rail states have not been published");
					continue;
				}
				if(getDistance(current_positions, rail_state_->position) < 1e-4) {
					rail_has_stopped = true;
					ROS_INFO_STREAM("Rail has stopped.");
				}
			}
			bool robot_has_stopped = false;
			current_positions.clear();
			while(!robot_has_stopped) {
				current_positions = sia20_state_->position;
				sia20_state_ = ros::topic::waitForMessage<sensor_msgs::JointState>("/sia20f/sia20f_r1_controller/joint_states", ros::Duration(topic_time_out_));
				if (!sia20_state_) {
					ROS_ERROR_STREAM("Rail states have not been published");
					continue;
				}
				if(getDistance(current_positions, sia20_state_->position) < 1e-4) {
					robot_has_stopped = true;
					ROS_INFO_STREAM("Robot has stopped.");
				}
			}
			return (rail_has_stopped && robot_has_stopped);
		}
	}
	double getDistance (std::vector<double> const & joints_p1, std::vector<double> const & joints_p2) {
		double distance = 0.0;
		double diff;
		if (joints_p1.size() != joints_p2.size()) {
			ROS_ERROR_STREAM ("Comparing points of different sizes.");
			distance = 1e10;
			return distance;
		}
		for (size_t pos_idx = 0; pos_idx < joints_p1.size(); pos_idx++) {
			diff = joints_p1[pos_idx] - joints_p2[pos_idx];
			distance += diff*diff;
	}
		return sqrt(distance);
	}


	bool checkTrajectorySanity(trajectory_msgs::JointTrajectory & motion_trajectory1, trajectory_msgs::JointTrajectory & motion_trajectory2) {
		double distance = 0.0;
		double joint_diff;

		for (int i = 0; i < motion_trajectory1.points.back().positions.size(); i++) {
			joint_diff = motion_trajectory1.points.back().positions[i] - motion_trajectory2.points[0].positions[i];
			distance += (joint_diff*joint_diff);
		}
		distance = sqrt(distance);
		
		if(distance < 1e-1) {
			//Trajectory safe to execute
			ROS_INFO_STREAM("Trajectory sanity check OK!");
			return true;
		} else {
			ROS_ERROR_STREAM("Motion safety check violation.");
			ROS_ERROR_STREAM("Starting configuration offset " << distance << "> 0.01.");
			return false;
		}
	}
};
} //namespace
