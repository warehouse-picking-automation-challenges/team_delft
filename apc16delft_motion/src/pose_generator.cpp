#include <ros/ros.h>
#include <ros/package.h>

#include <string>
#include <fstream>
#include <iostream>
#include <sstream>

#include <math.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>


using namespace std;

void Save(vector<vector<double> > values1, vector<vector<double> > values2){
  ofstream os, os1;
  stringstream ss, ss1;

  string path = ros::package::getPath("apc16delft_data");

  ss << path << "/masterposes/master_poses.yaml";
  ss1 << path << "/masterposes/moveit_commander.vars";

  os.open(ss.str());
  os1.open(ss1.str());

  os << "apc16delft:\n  master_poses:\n";
  os1 << "use manipulator_tool0\n";
  for (int index1 = 0; index1 < values1.size(); index1++){
    os << "    bin" << index1 << ":\n";
    os1 << "bin" << index1 << " = [";
    for (int index2 = 0; index2 < values1[index1].size(); index2++){
      os << "    - " << values1[index1][index2] << '\n';
      os1 << values1[index1][index2] << " ";
    }
    os1 << "]\n";

    os << "    cam" << index1 << ":\n";
    os1 << "cam" << index1 << " = [";
    for (int index2 = 0; index2 < values2[index1].size(); index2++){
      os << "    - " << values2[index1][index2] << '\n';
      os1 << values2[index1][index2] << " ";
    }
    os1 << "]\n";
  }
  os.close();
  os1.close();
}


int main(int argc, char **argv)
{
  std::string ns = "apc16delft_pose_generator";
  
  ros::init(argc, argv, ns);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Setting Distance type for TRAC_IK
  ros::NodeHandle nh;
  stringstream param_ns1;
  param_ns1 << "/" << ns << "/manipulator_tool0/solve_type";
  ros::param::set(param_ns1.str(), "Distance");


  ROS_INFO("Starting master and camerapose generation");

  // Measurements starting here!
  double shelf_offset = 1.1;
  double shelf_width = 1.1;
  double shelf_face_distance = 0.44;

  vector<double> shelf_heights;
  shelf_heights.push_back(1.81);
  shelf_heights.push_back(1.54);
  shelf_heights.push_back(1.32);
  shelf_heights.push_back(1.09);
  shelf_heights.push_back(0.82);

  double camera_angle = (20/180.0)*M_PI;
  double camera_distance = 0.65;

  double ee_angle = 0/(2*M_PI);
  double ee_distance = 0.20;
  // Measurements ending here!


  Eigen::Matrix3d cam_rot;
  cam_rot = Eigen::AngleAxisd(-((M_PI/2.0)+camera_angle), Eigen::Vector3d::UnitX());

  Eigen::Matrix3d grip_rot;
  grip_rot = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());

  vector<Eigen::Affine3d> cameraposes_cartesian;
  vector<Eigen::Affine3d> gripperposes_cartesian;


  for (int index = 0; index < 12; index++){
    Eigen::Affine3d pose;
    Eigen::Affine3d pose_grip;

    pose.linear() = cam_rot;
    pose_grip.linear() = grip_rot;

  	int shelf_row = floor(index/3);
  	int shelf_col = index%3;

  	pose.translation().x() = shelf_width*(shelf_col-1)/4;
  	pose.translation().y() = shelf_offset-shelf_face_distance+0.10-camera_distance*cos(camera_angle);
  	pose.translation().z() = shelf_heights[shelf_row+1]+camera_distance*sin(camera_angle);

    pose_grip.translation().x() = shelf_width*(shelf_col-1)/4;
    pose_grip.translation().y() = shelf_offset-shelf_face_distance-0.20;
    pose_grip.translation().z() = shelf_heights[shelf_row+1]+0.10;

    Eigen::Quaterniond rot(pose.rotation());
    Eigen::Quaterniond rot_g(pose_grip.rotation());

    gripperposes_cartesian.push_back(pose_grip);
    cameraposes_cartesian.push_back(pose);
  }

  ROS_INFO("Done generation cartesian poses");

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

  kinematic_state->setToDefaultValues();
  const robot_state::JointModelGroup* jmg_gripper = kinematic_model->getJointModelGroup("manipulator_tool0");

  vector<double> jv;
  for (int i = 0; i<12;i++) jv.push_back(0.0);

  vector<vector<double> > cameraposes_jointspace;
  vector<vector<double> > gripperposes_jointspace;

  for (int index = 0; index < cameraposes_cartesian.size(); index++){

    vector<double> cam_config;
    vector<double> grip_config;
    
    // find IK for camera pose

    if (index > 2){
      kinematic_state->setJointGroupPositions(jmg_gripper, cameraposes_jointspace[index-3]);
    } else {
      kinematic_state->setJointGroupPositions(jmg_gripper, jv);
    }

    // tranformation between gripper_camera_color and gripper_tool0
    // got the transform from: rosrun tf tf_echo gripper_camera_color gripper_tool0
    Eigen::Affine3d cam_grip_tf;
    Eigen::Quaterniond rot(0.574, -0.819, 0.000, 0.000);

    cam_grip_tf.translation().x() = 0.0;
    cam_grip_tf.translation().y() = 0.373;
    cam_grip_tf.translation().z() = 0.31;
    cam_grip_tf.linear() = rot.toRotationMatrix();

    bool found_ik = kinematic_state->setFromIK(jmg_gripper, cameraposes_cartesian[index]*cam_grip_tf, 2, 1);
    if (found_ik) {
      ROS_INFO("found IK solution! (cam)");
      kinematic_state->copyJointGroupPositions(jmg_gripper, cam_config);
      cameraposes_jointspace.push_back(cam_config);
    } else {
      ROS_INFO("Did not find IK solution (camera)");
    }

    found_ik = kinematic_state->setFromIK(jmg_gripper, gripperposes_cartesian[index], 2, 1);
    if (found_ik) {
      ROS_INFO("found IK solution! (grip)");
      kinematic_state->copyJointGroupPositions(jmg_gripper, grip_config);
      gripperposes_jointspace.push_back(grip_config);
    } else {
      ROS_INFO("Did not find IK solution (gripper)");
    }
  }

  Save(gripperposes_jointspace, cameraposes_jointspace);
  return 0;
}
