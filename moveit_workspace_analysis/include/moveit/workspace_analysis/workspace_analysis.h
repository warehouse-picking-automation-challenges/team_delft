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

#ifndef MOVEIT_WORKSPACE_ANALYSIS_H_
#define MOVEIT_WORKSPACE_ANALYSIS_H_

#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <moveit_msgs/WorkspaceParameters.h>
#include <moveit/planning_scene/planning_scene.h>

namespace moveit_workspace_analysis
{
static const double DEFAULT_RESOLUTION = 2.0;

struct point3D
{
  double x,y,z;
  point3D(double x, double y, double z):x(x),y(y),z(z){};
  point3D operator=(point3D p){x = p.x; y = p.y; z=p.z; return *this;};
  bool operator==(const point3D& p) const {return (x == p.x) && (y == p.y) && (z==p.z);};
  bool operator<(const point3D& p) const
  {
    if (x < p.x) 
      return true; 
    else if ((x == p.x)&&(y < p.y))
      return true;
    else if ((x == p.x)&&(y == p.y)&&(z < p.z))
      return true;
    return false;
  };
};

struct WorkspaceMetrics
{
  std::string robot_name_;
  std::string group_name_;
  std::string frame_id_;
  std::vector<geometry_msgs::Pose> points_;
  std::vector<double> manipulability_;
  std::vector<double> min_distance_joint_limits_;
  std::vector<unsigned int> min_distance_joint_limit_index_;
  std::vector<std::vector<double> > joint_values_;
  bool writeToFile(const std::string &filename, const std::string &delimiter = ",", bool exclude_strings = true);
  bool readFromFile(const std::string &filename, unsigned int num_joints);
  visualization_msgs::Marker getMarker(double marker_scale, unsigned int id, const std::string &ns) const;
  visualization_msgs::Marker getDensityMarker(double marker_scale, unsigned int id, const std::string &ns, bool smooth_colors = true) const;
};

class WorkspaceAnalysis
{
public:

  WorkspaceAnalysis(const planning_scene::PlanningSceneConstPtr &planning_scene,
                    bool position_only=false,
                    double joint_limits_penalty_multiplier=0.0);

  virtual ~WorkspaceAnalysis()
  {
  };

  WorkspaceMetrics computeMetrics(const moveit_msgs::WorkspaceParameters &workspace,
                                  const std::vector<geometry_msgs::Quaternion> &orientations,
                                  robot_state::RobotState *joint_state,
                                  const robot_model::JointModelGroup *joint_model_group,
                                  double x_resolution,
                                  double y_resolution,
                                  double z_resolution) const;

  WorkspaceMetrics computeMetricsFK(robot_state::RobotState *joint_state,
                                    const robot_model::JointModelGroup *joint_model_group,
                                    unsigned int max_attempts,
                                    const ros::WallDuration &max_duration,
                                    const std::map<std::string, std::vector<double> > &fixed_joint_values = std::map<std::string, std::vector<double> >()) const;

  void setJointLimitsPenaltyMultiplier(double multiplier)
  {
    kinematics_metrics_->setPenaltyMultiplier(multiplier);
  };

  void activate()
  {
    canceled_ = false;
  }

  void cancel()
  {
    canceled_ = true;
  };

private:

  void updateMetrics(robot_state::RobotState *joint_state,
                     const robot_model::JointModelGroup *joint_model_group,
                     moveit_workspace_analysis::WorkspaceMetrics &metrics) const;

  bool isIKSolutionCollisionFree(robot_state::RobotState *joint_state,
                                 const robot_model::JointModelGroup *joint_model_group,
                                 const double *ik_solution);

  std::vector<geometry_msgs::Pose> sampleUniform(const moveit_msgs::WorkspaceParameters &workspace,
                                                 const std::vector<geometry_msgs::Quaternion> &orientations,
                                                 double x_resolution,
                                                 double y_resolution,
                                                 double z_resolution) const;
  bool position_only_ik_, canceled_;
  kinematics_metrics::KinematicsMetricsPtr kinematics_metrics_;
  robot_state::GroupStateValidityCallbackFn state_validity_callback_fn_;
  const planning_scene::PlanningSceneConstPtr planning_scene_;
};

}
#endif
