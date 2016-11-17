#! /usr/bin/env python
import rospy
import copy
import actionlib


import moveit_msgs.msg
import trajectory_msgs.msg
import geometry_msgs.msg

def simple_pick_client():
    client = actionlib.SimpleActionClient('pickup', moveit_msgs.msg.PickupAction)
    client.wait_for_server()
    
    grasp = moveit_msgs.msg.Grasp()
    grasp.id = "simple"
    grasp.grasp_quality = 1.0
    grasp.allowed_touch_objects.append("grasp_object")
    
    grasp.pre_grasp_posture = trajectory_msgs.msg.JointTrajectory() # make sure gripper is open
    grasp.pre_grasp_posture.joint_names.append("vacuum_tool_joint")
    point = trajectory_msgs.msg.JointTrajectoryPoint()
    point.positions.append(0.0)
    point.velocities.append(0.0)
    point.accelerations.append(0.0)
    point.effort.append(0.0)
    point.time_from_start = rospy.Duration(3.0)
    grasp.pre_grasp_posture.points.append(point)
    
    grasp.grasp_posture = trajectory_msgs.msg.JointTrajectory()     # gripper is closed
    grasp.grasp_posture.joint_names.append("vacuum_tool_joint")
    point2 = copy.deepcopy(point)
    point.positions = [-0.01]
    grasp.grasp_posture.points.append(point)
    
    grasp.grasp_pose = geometry_msgs.msg.PoseStamped()
    grasp.grasp_pose.header.frame_id = "object_10"
    grasp.grasp_pose.pose.position.x = 0.0
    grasp.grasp_pose.pose.position.y = 0.0
    grasp.grasp_pose.pose.position.z = 0.05
    grasp.grasp_pose.pose.orientation.w = 1.0
    
    grasp.pre_grasp_approach = moveit_msgs.msg.GripperTranslation()
    grasp.pre_grasp_approach.direction.header.frame_id = "object_10"
    grasp.pre_grasp_approach.direction.vector.x = 1.0
    grasp.pre_grasp_approach.direction.vector.y = 0.0
    grasp.pre_grasp_approach.direction.vector.z = 0.0
    grasp.pre_grasp_approach.desired_distance = 0.10
    grasp.pre_grasp_approach.min_distance = 0.05
    grasp.post_grasp_retreat = moveit_msgs.msg.GripperTranslation()
    grasp.post_grasp_retreat.direction.header.frame_id = "object_10"
    grasp.post_grasp_retreat.direction.vector.x = -1.0
    grasp.post_grasp_retreat.direction.vector.y = 0.0
    grasp.post_grasp_retreat.direction.vector.z = 0.0
    grasp.post_grasp_retreat.desired_distance = 0.10
    grasp.post_grasp_retreat.min_distance = 0.05
    grasp.post_place_retreat = moveit_msgs.msg.GripperTranslation()
    grasp.post_place_retreat.direction.header.frame_id = "object_10"
    grasp.post_place_retreat.direction.vector.x = -1.0
    grasp.post_place_retreat.direction.vector.y = 0.0
    grasp.post_place_retreat.direction.vector.z = 0.0
    grasp.post_place_retreat.desired_distance = 0.10
    grasp.post_place_retreat.min_distance = 0.05
    
    
    goal = moveit_msgs.msg.PickupGoal()
    goal.target_name = "grasp_object"
    goal.group_name = "manipulator_and_tool"
    goal.end_effector = "suction_tool"
    goal.possible_grasps.append(grasp)
    #goal.support_surface_name = ""
    goal.allow_gripper_support_collision = False
    goal.attached_object_touch_links.append("gripper")
    #rest is optional
    goal.planner_id = "RRTConnectkConfigDefault"
    goal.allowed_planning_time = 3.0
    
    
    
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('simple_pick_client')
        result = simple_pick_client()
        print result
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
