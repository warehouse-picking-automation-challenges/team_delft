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

using namespace std;

void storeYaml(string from, string to, moveit::planning_interface::MoveGroup::Plan plan){
  string path = ros::package::getPath("apc16delft_data");

  trajectory_msgs::JointTrajectory trajectory = plan.trajectory_.joint_trajectory;

  ofstream os;
  stringstream ss;
  ss << path << "/trajectories/" << from << "_to_" << to << ".yaml";
  ROS_DEBUG("Storing YAML: %s", ss.str().c_str());


  os.open(ss.str());
  os << "trajectory: \n  header: \n    seq: 0\n    stamp: \n      secs: 0\n      nsecs: 0\n    frame_id: /world\n";
  os << "  joint_names: ['";

  vector<string> jns = trajectory.joint_names;
  for (int ii = 0; ii < jns.size()-1; ii++) os << jns[ii] << "', '";
  os << jns.back() << "']\n";
 
  os << "  points: \n";

  for (int i = 0; i < trajectory.points.size(); i++){
    trajectory_msgs::JointTrajectoryPoint jtp = trajectory.points[i];

    os << "    - \n";
    os << "      positions: [";
    for (int ii = 0; ii < jtp.positions.size()-1; ii++) os << jtp.positions[ii] << ", ";
    os << jtp.positions.back() << "]\n";

    os << "      velocities: [";
    for (int ii = 0; ii < jtp.velocities.size()-1; ii++) os << jtp.velocities[ii] << ", ";
    os << jtp.velocities.back() << "]\n";

    os << "      accelerations: [";
    for (int ii = 0; ii < jtp.accelerations.size()-1; ii++) os << jtp.accelerations[ii] << ", ";
    os << jtp.accelerations.back() << "]\n";

    os << "      effort: []\n";
    os << "      time_from_start: \n";
    os << "        secs: " << jtp.time_from_start.sec << "\n";
    os << "        nsecs: " << jtp.time_from_start.nsec << "\n";
  }
  os.close();
}


bool planPaths(moveit::planning_interface::MoveGroup &group, 
  moveit::planning_interface::MoveGroup::Plan &plan, 
  moveit::planning_interface::MoveGroup::Plan &reverse_plan,
  robot_state::RobotState start_state,
  int loops){

  group.setNumPlanningAttempts(1);
  double trajectory_velocity_scaling_ = 0.2;

  moveit::planning_interface::MoveGroup::Plan temp_plan;
  bool success = false;

  ros::Duration best_time(100000.0);
  ros::Duration current_time(0.0);

  for (int i = 0; i < loops; i++){
    bool suc = group.plan(temp_plan);
    if (suc){
      success = true;
      current_time = temp_plan.trajectory_.joint_trajectory.points.back().time_from_start;
      if (current_time < best_time){
        plan = temp_plan;
        best_time = current_time;
      }
    }
  }
  reverse_plan = plan; 

  std::reverse(reverse_plan.trajectory_.joint_trajectory.points.begin(), 
    reverse_plan.trajectory_.joint_trajectory.points.end());

  robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), group.getName());
  rt.setRobotTrajectoryMsg(start_state, reverse_plan.trajectory_);

  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  iptp.computeTimeStamps(rt, trajectory_velocity_scaling_);
  rt.getRobotTrajectoryMsg(reverse_plan.trajectory_);
  return success;
}

void stitchPlans(moveit::planning_interface::MoveGroup &group, 
  moveit::planning_interface::MoveGroup::Plan &plan1, 
  moveit::planning_interface::MoveGroup::Plan &plan2,
  robot_state::RobotState start_state){

  double trajectory_velocity_scaling_ = 0.2;

  plan1.trajectory_.joint_trajectory.points.insert(
      plan1.trajectory_.joint_trajectory.points.end(),
      plan2.trajectory_.joint_trajectory.points.begin()+1,
      plan2.trajectory_.joint_trajectory.points.end());

  robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), group.getName());
  rt.setRobotTrajectoryMsg(start_state, plan1.trajectory_);

  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  iptp.computeTimeStamps(rt, trajectory_velocity_scaling_);
  rt.getRobotTrajectoryMsg(plan1.trajectory_);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "apc16delft_trajectory_generator");
  ros::NodeHandle nh;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  double trajectory_velocity_scaling_ = dr::getParam<double>(nh, "trajectory_velocity_scaling_", 0.2);
  double box_dim = dr::getParam<double>(nh, "attach_box_sixe", 0.2);

  moveit::planning_interface::MoveGroup group("manipulator_tool0");
  group.setGoalTolerance(0.001);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroup::Plan plan, reverse_plan, temp_plan;
  robot_state::RobotState start_state(*group.getCurrentState());
  const robot_state::JointModelGroup *jmg = start_state.getJointModelGroup(group.getName());
  group.setMaxVelocityScalingFactor(0.2);
  std::vector<shapes::ShapeConstPtr> shapes_ptr;


  ros::Publisher pub = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
  // moveit::planning_interface::PlanningSceneInterface *psi = new moveit::planning_interface::PlanningSceneInterface();
  ros::Duration(1.0).sleep();
  
  // Attached Collision Object

  moveit_msgs::AttachedCollisionObject aco;
  aco.link_name = "gripper_body";
  aco.object.id = "box";
  geometry_msgs::Pose aco_pose;

  aco_pose.position.x    = 0.0;
  aco_pose.position.y    = 0.008;
  aco_pose.position.z    = 0.623;
  aco_pose.orientation.x = 0.0;
  aco_pose.orientation.y = -0.7071;
  aco_pose.orientation.z = -0.7071;
  aco_pose.orientation.w = 0.0;

  shapes::Box collision_box(box_dim,box_dim,box_dim);

  aco.object.header.frame_id = "gripper_body";
  aco.object.operation = aco.object.ADD;

  vector<vector<double> > cam_bins;
  vector<vector<double> > grip_bins;
  vector<vector<double> > tote_v;
  vector<vector<double> > tote_h;

  int loops_low = 4;
  int loops_high = 8;

  //geometry_msgs::Pose tote_pose;
  
  vector<double> home;  
  group.setNamedTarget("home");
  const robot_state::RobotState home_rs = group.getJointValueTarget();
  home_rs.copyJointGroupPositions(jmg, home);

  // ROS_INFO("home: %f, %f, %f, %f, %f, %f, %f, %f", home[0], home[1], home[2], home[3], home[4], home[5], home[6], home[7]);

  vector<double> tote;
  group.setNamedTarget("tote");
  const robot_state::RobotState tote_rs = group.getJointValueTarget();
  tote_rs.copyJointGroupPositions(jmg, tote);


  vector<double> midway;
  nh.getParam("/apc16delft/master_poses/midway", midway);



  ROS_INFO_STREAM("Generating bin trajectories...");
  for (int i = 0; i<12; i++){
    vector<double> grip_state, cam_state;

    stringstream ss1, ss2;

    ss1 << "/apc16delft/master_poses/bin" << i;
    ss2 << "/apc16delft/master_poses/cam" << i;
    // ss2 << "/apc16delft/cameraposes_jointspace/bin" << i;

    nh.getParam(ss1.str(),grip_state);
    ROS_INFO_STREAM("getParam(): " << ss1.str() << ": " << grip_state.size());
    grip_bins.push_back(grip_state);

    nh.getParam(ss2.str(),cam_state);
    ROS_INFO_STREAM("getParam(): " << ss2.str() << ": " << cam_state.size());
    cam_bins.push_back(cam_state);
  }

  // Fetching tote states
  for (int i = 0; i<2; i++){
    for (int ii = 0; ii<3; ii++){
      vector<double> h_state, v_state;

      stringstream ss1, ss2;

      ss1 << "/apc16delft/master_poses/tote" << i << ii << "h";
      ss2 << "/apc16delft/master_poses/tote" << i << ii << "v";

      nh.getParam(ss1.str(),h_state);
      ROS_INFO_STREAM("getParam(): " << ss1.str() << ": " << h_state.size());
      tote_h.push_back(h_state);

      nh.getParam(ss2.str(),v_state);
      ROS_INFO_STREAM("getParam(): " << ss2.str() << ": " << v_state.size());
      tote_v.push_back(v_state);
    }
  }

  vector<double> start;
  vector<double> goal;
  bool success;

  int loops = loops_low;

  moveit::planning_interface::MoveGroup::Plan tote_to_mid;
  moveit::planning_interface::MoveGroup::Plan mid_to_tote;

  //////////////////////////////////////////////////////////////////////
  // tote to home

  // start = tote;
  // goal = home;

  // start_state.setJointGroupPositions(jmg, start);
  // group.setStartState(start_state);
  // group.setJointValueTarget(goal);

  // success = planPaths(group, plan, reverse_plan, start_state, loops);

  // if (success){
  //   // storeYaml("tote","home", plan);
  //   // storeYaml("home","tote", reverse_plan);
  // } else {
  //   ROS_WARN_STREAM("Cannot find trajectory for tote - home");
  // }

  //////////////////////////////////////////////////////////////////////
  // tote to midway

  start = tote;
  goal = midway;

  start_state.setJointGroupPositions(jmg, start);
  group.setStartState(start_state);
  group.setJointValueTarget(goal);

  success = planPaths(group, plan, reverse_plan, start_state, loops);
  if (success){
    tote_to_mid = plan;
    mid_to_tote = reverse_plan;
  } else {
    ROS_WARN_STREAM("Cannot find trajectory for tote - home");
  }

  ROS_INFO_STREAM("Starting bin!");

  std::vector<int> selected_bins;

  selected_bins.push_back(2);
  selected_bins.push_back(5);
  selected_bins.push_back(8);
  selected_bins.push_back(11);

  // only right bins changing
  for(int i=2; i<grip_bins.size(); i=i+3) {
    
    if (i==9||i==11){
      loops = loops_high;
    }

    ROS_INFO_STREAM("Starting with bin " << i);

    stringstream ss1, ss2;
    ss1 << "bin" << i;
    ss2 << "cam" << i;

    string bin = ss1.str();
    string cam = ss2.str();

    //////////////////////////////////////////////////////////////////////
    // bin to home and reverse

    // start = grip_bins[i];
    // goal = home;

    // start_state.setJointGroupPositions(jmg, start);
    // group.setStartState(start_state);
    // group.setJointValueTarget(goal);

    // success = planPaths(group, plan, reverse_plan, start_state, loops);

    // if (success){
    //   storeYaml(bin,"home", plan);
    //   storeYaml("home",bin, reverse_plan);
    // } else {
    //   ROS_WARN_STREAM("Cannot find trajectory for bin - home");
    // }


    // //////////////////////////////////////////////////////////////////////
    // // bin to tote and reverse

    // start = grip_bins[i];
    // goal = midway;

    // start_state.setJointGroupPositions(jmg, start);
    // group.setStartState(start_state);
    // group.setJointValueTarget(goal);
    
    // success = planPaths(group, plan, reverse_plan, start_state, loops);

    moveit::planning_interface::MoveGroup::Plan rev_traj = tote_to_mid;

    // stitchPlans(group, plan, mid_to_tote, start_state);
    // stitchPlans(group, rev_traj, reverse_plan, start_state);

    // if (success){
    //   storeYaml(bin,"tote", plan);
    //   storeYaml("tote",bin, rev_traj);
    // } else {
    //   ROS_WARN_STREAM("Cannot find trajectory for bin - tote");
    // }


    //////////////////////////////////////////////////////////////////////
    // cam to home and reverse

    start = cam_bins[i];
    goal = home;

    start_state.setJointGroupPositions(jmg, start);
    group.setStartState(start_state);
    group.setJointValueTarget(goal);
    
    success = planPaths(group, plan, reverse_plan, start_state, loops);

    if (success){
      storeYaml(cam, "home", plan);
      storeYaml("home", cam, reverse_plan);
    } else {
      ROS_WARN_STREAM("Cannot find trajectory for cam - home");
    }

    //////////////////////////////////////////////////////////////////////
    // tote to cam and reverse

    // start = tote;
    // goal = cam_bins[i];

    start = cam_bins[i];
    goal = midway;

    start_state.setJointGroupPositions(jmg, start);
    group.setStartState(start_state);
    group.setJointValueTarget(goal);

    success = planPaths(group, plan, reverse_plan, start_state, loops);

    rev_traj = tote_to_mid;

    stitchPlans(group, plan, mid_to_tote, start_state);
    stitchPlans(group, rev_traj, reverse_plan, start_state);

    if (success){
      storeYaml(cam, "tote", plan);
      storeYaml("tote", cam, rev_traj);
    } else {
      ROS_WARN_STREAM("Cannot find trajectory for cam - tote");
    }


    //////////////////////////////////////////////////////////////////////
    // bin to cam and reverse

    // start = grip_bins[i];
    // goal = cam_bins[i];

    start = cam_bins[i];
    goal = grip_bins[i];

    start_state.setJointGroupPositions(jmg, start);
    group.setStartState(start_state);
    group.setJointValueTarget(goal);

    success = planPaths(group, plan, reverse_plan, start_state, loops);

    if (success){
      storeYaml(cam, bin, plan);
      storeYaml(bin, cam, reverse_plan);
    } else {
      ROS_WARN_STREAM("Cannot find trajectory for bin - cam");
    }
  }

  loops = loops_low;

  //////////////////////////////////////////////////////////////////////
  // cam to cam
  stringstream ss1, ss2;
  string src, dst;
  for(size_t src_idx = 0; src_idx < cam_bins.size(); src_idx=src_idx+1) {
      ss1 << "cam" << src_idx;
      src = ss1.str();
    for(size_t dst_idx = 0; dst_idx < cam_bins.size(); dst_idx=dst_idx+1) {
      ss2 << "cam" << dst_idx;
      dst = ss2.str();
      if(src_idx == dst_idx) {
        //Clear string stream to avoid appending names.
        ss2.str(std::string());
        continue;
      } else {
        ROS_INFO_STREAM("Generating trajectory from " << src << " to " << dst << ".");

        if (src_idx == 9 || src_idx == 11 || src_idx == 9 || src_idx == 11){
          loops = loops_high;
        } else {
          loops = loops_low;
        }

        start = cam_bins[src_idx];
        goal = cam_bins[dst_idx];

        start_state.setJointGroupPositions(jmg, start);
        group.setStartState(start_state);
        group.setJointValueTarget(goal);

        success = planPaths(group, plan, reverse_plan, start_state, loops);

        if (success){
          storeYaml(src, dst, plan);
          storeYaml(dst, src, reverse_plan);
        } else {
          ROS_WARN_STREAM("Cannot find trajectory for cam - cam");
        }

        ss2.str(std::string());
      }
    }
    //Clear string stream to avoid appending names.
    ss1.str(std::string());
  }
  
  loops = loops_low;


/*
  for(int i=0; i<2; i++) {
    for(int ii=0; ii<3; ii++) {
      stringstream ss;

      // vertical tote poses
      start = tote;
      goal = tote_v[i*3+ii];

      start_state.setJointGroupPositions(jmg, start);
      group.setStartState(start_state);
      group.setJointValueTarget(goal);

      success = planPaths(group, plan, reverse_plan, start_state, loops);

      ss << "tote" << i << ii << "v";
      if (success){
        storeYaml("tote",ss.str(), plan);
        storeYaml(ss.str(), "tote", reverse_plan);
      } else {
        ROS_WARN_STREAM("Cannot find trajectory for tote - tote");
      }

      ss.str(std::string());

      // Horizontal totes

      goal = tote_h[i*3+ii];
      start_state.setJointGroupPositions(jmg, start);
      group.setStartState(start_state);
      group.setJointValueTarget(goal);

      success = planPaths(group, plan, reverse_plan, start_state, loops);

      ss << "tote" << i << ii << "h";
      if (success){
        storeYaml("tote",ss.str(), plan);
        storeYaml(ss.str(), "tote", reverse_plan);
      } else {
        ROS_WARN_STREAM("Cannot find trajectory for tote - tote");
      }
      ss.str(std::string());
    }
  }
*/

  ROS_INFO_STREAM("Finished trajectory generation.");
  ros::shutdown();  
  return 0;
}
