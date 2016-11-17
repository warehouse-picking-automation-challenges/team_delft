#! /usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

def simple_motion_node():
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group = moveit_commander.MoveGroupCommander("manipulator")

    display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory)

    print "================= Reference frame: %s" % group.get_planning_frame()
    print "================= Robot Groups:"
    print robot.get_group_names()

    # group.set_end_effector_link("gripper")
    # group.set_pose_reference_frame("bin_frame")
    group.set_planner_id("RRTConnectkConfigDefault")
    # print group.get_current_pose()
    # print robot.get_current_state()

    print "================= Generating Plan 1"
    pose_target = geometry_msgs.msg.PoseStamped()
    pose_target.header.frame_id = "bin_frame"
    pose_target.pose.orientation.w = 1.0
    pose_target.pose.orientation.x = 0.0
    pose_target.pose.orientation.y = 0.0
    pose_target.pose.orientation.z = 0.0
    pose_target.pose.position.x = 0.0
    pose_target.pose.position.y = -0.60
    pose_target.pose.position.z = 0.3
    # group.set_named_target("home")
    group.set_pose_target(pose_target)
    # print group.get_current_pose()
    # plan1 = group.plan()
    # rospy.sleep(5)

    # print "============ Visualizing plan1"
    # display_trajectory = moveit_msgs.msg.DisplayTrajectory()

    # display_trajectory.trajectory_start = robot.get_current_state()
    # display_trajectory.trajectory.append(plan1)
    # display_trajectory_publisher.publish(display_trajectory);

    # print "============ Waiting while plan1 is visualized (again)..."
    # rospy.sleep(5)

    group.go(wait=True)

    rospy.sleep(5.0)

    group.set_named_target("drop")
    group.go(wait=True)
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('simple_motion_node')
        result = simple_motion_node()
        print result
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
