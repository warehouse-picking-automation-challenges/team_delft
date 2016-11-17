/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Sachin Chitta */

#include <moveit/workspace_analysis/workspace_analysis.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <fstream>
#include <iostream>

int main(int argc, char **argv)
{
  ros::init (argc, argv, "workspace_analysis");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  // wait some time for everything to be loaded correctly...
  ROS_INFO_STREAM("Waiting a few seconds to load the robot description correctly...");
  sleep(3);
  ROS_INFO_STREAM("Hope this was enough!");

  /*Get some ROS params */
  ros::NodeHandle node_handle("~");
  double res_x, res_y, res_z;
  double min_x, min_y, min_z;
  double max_x, max_y, max_z;
  int max_attempts;
  double joint_limits_penalty_multiplier;
  std::string group_name;

  if (!node_handle.getParam("min_x", min_x))
    min_x = 0.0;
  if (!node_handle.getParam("max_x", max_x))
    max_x = 0.0;
  if (!node_handle.getParam("res_x", res_x))
    res_x = 0.1;

  if (!node_handle.getParam("min_y", min_y))
    min_y = 0.0;
  if (!node_handle.getParam("max_y", max_y))
    max_y = 0.0;
  if (!node_handle.getParam("res_y", res_y))
    res_y = 0.1;

  if (!node_handle.getParam("min_z", min_z))
    min_z = 0.0;
  if (!node_handle.getParam("max_z", max_z))
    max_z = 0.0;
  if (!node_handle.getParam("res_z", res_z))
    res_z = 0.1;
  if (!node_handle.getParam("max_attempts", max_attempts))
    max_attempts = 10000;

  if (!node_handle.getParam("joint_limits_penalty_multiplier", joint_limits_penalty_multiplier))
    joint_limits_penalty_multiplier = 0.0;

  std::string filename;
  if (!node_handle.getParam("filename", filename))
    ROS_ERROR("Will not write to file");

  std::string quat_filename;
  if (!node_handle.getParam("quat_filename", quat_filename))
    ROS_ERROR("Will not write to file");

  if (!node_handle.getParam("group_name", group_name))
    ROS_FATAL("Must have valid group name");

  /* Load the robot model */
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

  /* Get a shared pointer to the model */
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

  /* Create a robot state*/
  robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));

  if(!robot_model->hasJointModelGroup(group_name))
    ROS_FATAL("Invalid group name: %s", group_name.c_str());

  const robot_model::JointModelGroup* joint_model_group = robot_model->getJointModelGroup(group_name);

  /* Construct a planning scene - NOTE: this is for illustration purposes only.
     The recommended way to construct a planning scene is to use the planning_scene_monitor
     to construct it for you.*/
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

  moveit_msgs::WorkspaceParameters workspace;
  workspace.min_corner.x = min_x;
  workspace.min_corner.y = min_y;
  workspace.min_corner.z = min_z;

  workspace.max_corner.x = max_x;
  workspace.max_corner.y = max_y;
  workspace.max_corner.z = max_z;

  /* Load the workspace analysis */
  moveit_workspace_analysis::WorkspaceAnalysis workspace_analysis(planning_scene, true, joint_limits_penalty_multiplier);

  /* Compute the metrics */

  // load the set of quaternions
  std::vector<geometry_msgs::Quaternion> orientations;
  std::ifstream quat_file;
  quat_file.open(quat_filename.c_str());
  while(!quat_file.eof())
  {
    geometry_msgs::Quaternion temp_quat;
	quat_file >> temp_quat.x >> temp_quat.y >> temp_quat.z >> temp_quat.w;
    orientations.push_back(temp_quat);
  }
  
  ros::Time init_time;
  init_time = ros::Time::now();

  moveit_workspace_analysis::WorkspaceMetrics metrics = workspace_analysis.computeMetrics(workspace, orientations, robot_state.get(), joint_model_group, res_x, res_y, res_z);

  if(metrics.points_.empty())
    ROS_WARN_STREAM("No point to be written to file: consider changing the workspace, or recompiling moveit_workspace_analysis with a longer sleeping time at the beginning (if this could be the cause)");
  
  //ros::WallDuration duration(100.0);
  //moveit_workspace_analysis::WorkspaceMetrics metrics = workspace_analysis.computeMetricsFK(&(*robot_state), joint_model_group, max_attempts, duration);

  if(!filename.empty())
    if(!metrics.writeToFile(filename,",",false))
      ROS_INFO("Could not write to file");

  ros::Duration total_duration = ros::Time::now() - init_time;
  ROS_INFO_STREAM("Total duration: " << total_duration.toSec() << "s");

  ros::shutdown();
  return 0;
}
