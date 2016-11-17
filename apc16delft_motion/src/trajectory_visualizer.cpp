#include <ros/ros.h>
#include <ros/package.h>

#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <eigen_conversions/eigen_msg.h>

#include <dr_param/param.hpp>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/attached_body.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometric_shapes/shapes.h>

#include <apc16delft_msgs/MasterPoseDescriptor.h>

#include "trajectory_cache.hpp"


using namespace std;

int DispTrajectory(ros::Publisher pub, 
  apc16delft::TrajectoryCache trajectory_cache, 
  apc16delft_msgs::MasterPoseDescriptor from, 
  apc16delft_msgs::MasterPoseDescriptor to){

  trajectory_msgs::JointTrajectory t;
  t = trajectory_cache.lookupTrajectory(from, to);
  double duration = 1.0/t.points.size();

  int sel = 49;
  while(sel == 49){
    sel = 10;
    for (int i = 0; i < t.points.size(); i++){
      moveit_msgs::DisplayRobotState rs;
      rs.state.joint_state.name = t.joint_names;
      rs.state.joint_state.position = t.points[i].positions;
      pub.publish(rs);
      ros::Duration(duration).sleep();
    }
    ROS_INFO_STREAM("enter for next, 1 for again, close terminal to exit");
    sel = cin.get();
    cin.clear();
    cin.ignore(10000, '\n');
    ROS_INFO_STREAM(sel);
    if (sel == 50){
      ros::shutdown();
    }
  }

}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "visual_tools_demo");
  ros::NodeHandle nh_;

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(1);
  spinner.start();

  string path = ros::package::getPath("apc16delft_data");
  stringstream ss;
  ss << path << "/trajectories/";
  string yaml_path = ss.str();

  ros::Publisher rspub_ = nh_.advertise<moveit_msgs::DisplayRobotState>("/viz_robot_state", 100);

  // visual_tools_->loadRobotStatePub("display_robot_state");
  // visual_tools_->setManualSceneUpdating(true);

  // Allow time to publish messages
  ros::spinOnce();
  ros::Duration(0.1).sleep();

  // Clear collision objects and markers
  ROS_INFO_STREAM("hello?");

  // Show message
  // Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  // text_pose.translation().z() = 4;
  // visual_tools_->publishText(text_pose, "MoveIt! Visual Tools", rvt::WHITE, rvt::XLARGE, /*static_id*/ false);

  apc16delft::TrajectoryCache trajectory_cache = apc16delft::TrajectoryCache(yaml_path);

  apc16delft_msgs::MasterPoseDescriptor home;
  apc16delft_msgs::MasterPoseDescriptor tote;
  apc16delft_msgs::MasterPoseDescriptor tote_drop;
  apc16delft_msgs::MasterPoseDescriptor bin;
  apc16delft_msgs::MasterPoseDescriptor cam;
  apc16delft_msgs::MasterPoseDescriptor cam_src;
  apc16delft_msgs::MasterPoseDescriptor cam_dst;


  home.type = apc16delft_msgs::MasterPoseDescriptor::TYPE_HOME;
  tote_drop.type = apc16delft_msgs::MasterPoseDescriptor::TYPE_TOTE;

  tote.type = apc16delft_msgs::MasterPoseDescriptor::TYPE_TOTE;
  // bin_index is used for the inner tote poses
  tote.bin_index = 0; 
  bin.type  = apc16delft_msgs::MasterPoseDescriptor::TYPE_BIN;
  cam.type  = apc16delft_msgs::MasterPoseDescriptor::TYPE_CAM;

  DispTrajectory(rspub_, trajectory_cache, home, tote);
  DispTrajectory(rspub_, trajectory_cache, tote, home);

  for (int i = 0; i < 12; ++i) {
    bin.bin_index = i;
    cam.bin_index = i;
    ROS_INFO_STREAM("visualization of index " << i);
    DispTrajectory(rspub_, trajectory_cache, home,  bin);
    // DispTrajectory(rspub_, trajectory_cache, bin,  home);

    DispTrajectory(rspub_, trajectory_cache, home, cam);
    // DispTrajectory(rspub_, trajectory_cache, cam,  home);

    DispTrajectory(rspub_, trajectory_cache, tote, bin);
    // DispTrajectory(rspub_, trajectory_cache, bin,  tote);
    
    DispTrajectory(rspub_, trajectory_cache, tote, cam);
    // DispTrajectory(rspub_, trajectory_cache, cam,  tote);

    DispTrajectory(rspub_, trajectory_cache, bin, cam);
    // DispTrajectory(rspub_, trajectory_cache, cam, bin);
  }
  //Load cam to cam trajectories
  cam_src.type  = apc16delft_msgs::MasterPoseDescriptor::TYPE_CAM;
  cam_dst.type  = apc16delft_msgs::MasterPoseDescriptor::TYPE_CAM;
  for (size_t src_idx = 0; src_idx < 12 ; src_idx++) {
    for (size_t dst_idx = src_idx+1; dst_idx < 12; dst_idx++) {
      if(src_idx == dst_idx) {
        continue;
      } else {
        cam_src.bin_index = src_idx;
        cam_dst.bin_index = dst_idx;
        DispTrajectory(rspub_, trajectory_cache, cam_src, cam_dst);
        // DispTrajectory(rspub_, trajectory_cache, cam_dst, cam_src);
      }
    }
  }  

  //Load tote to tote trajectories

  tote_drop.group_name = tote_drop.GROUP_TOOL1;
  for (int i = 0; i<6; i++){
    tote_drop.bin_index = i+1;
    DispTrajectory(rspub_, trajectory_cache, tote, tote_drop);
    // DispTrajectory(rspub_, trajectory_cache, tote_drop, tote);
  }

  tote_drop.group_name = tote_drop.GROUP_TOOL0;
  for (int i = 0; i<6; i++){
    tote_drop.bin_index = i+1;
    DispTrajectory(rspub_, trajectory_cache, tote, tote_drop);
    // DispTrajectory(rspub_, trajectory_cache, tote_drop, tote);
  }


  ROS_INFO_STREAM("Shutting down.");
  return 0;
}
