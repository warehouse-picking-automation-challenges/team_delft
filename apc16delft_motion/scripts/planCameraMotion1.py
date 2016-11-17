#!/usr/bin/env python

import sys
import rospy

from apc16delft_msgs.msg import *
from apc16delft_msgs.srv import *


class TestClass:
	def move_to_home(self):
		rospy.wait_for_service('/motion_executor/move_to_home')
		try:
			move_home = rospy.ServiceProxy('/motion_executor/move_to_home', MoveToHome)
			res = move_home();
			print res.message;
			return True;
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def move_to_cam(self):
		rospy.wait_for_service('/motion_executor/execute_coarse_motion')
		try:
			service = rospy.ServiceProxy('/motion_executor/execute_coarse_motion', ExecuteCoarseMotion)

			req = ExecuteCoarseMotionRequest()
			req.start.group_name = 'manipulator_tool0'
			req.start.type = 'home'
			req.start.bin_index = 0
			req.target.group_name = 'manipulator_tool0'
			req.target.type = 'cam'
			req.target.bin_index = 1
			
			res = service(req);
			print res.message;
			return True;
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def move_back_and_forth(self):
		print "starting back and forth"
		rospy.wait_for_service('/manipulation_planner/plan_camera_motion')
		try:
			service = rospy.ServiceProxy('/manipulation_planner/plan_camera_motion', PlanCameraMotion)

			req = PlanCameraMotionRequest()
			req.bin_index = 9
			req.relative_transform.header.frame_id = 'gripper_tool0'

			req.relative_transform.pose.position.x = 0.0
			req.relative_transform.pose.position.y = 0.0
			req.relative_transform.pose.position.z = 0.0
			req.relative_transform.pose.orientation.x = 0.0
			req.relative_transform.pose.orientation.y = 0.0
			req.relative_transform.pose.orientation.z = 0.0
			req.relative_transform.pose.orientation.w = 1.0
			
			print "planning camera motion req!"
			res = service(req)
			print "planned camera motion!"

			rospy.wait_for_service('/motion_executor/execute_fine_motion')
			try:
				service = rospy.ServiceProxy('/motion_executor/execute_fine_motion', ExecuteFineMotion)

				req = ExecuteFineMotionRequest()
				req.trajectory = res.camera_trajectory
				
				res = service(req)
				print res.message
				return True
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

if __name__ == "__main__":
	rospy.init_node("manipulator_test");
	tc = TestClass();

	print "Starting test"
	tc.move_to_home();
	tc.move_to_cam();
	print "moving b and f?";
	tc.move_back_and_forth();

