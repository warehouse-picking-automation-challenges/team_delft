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
			return True;
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	# def configure_planner(self):
	# 	rospy.wait_for_service('/manipulation_planner/lifecycle/configure')
	# 	try:
	# 		srv = rospy.ServiceProxy('/manipulation_planner/lifecycle/configure', LifecycleState)
	# 		req = LifecycleStateRequest
	# 		res = srv(req);
	# 		return True;
	# 		rospy.wait_for_service('/manipulation_planner/lifecycle/activate')
	# 		try:
	# 			srv = rospy.ServiceProxy('/manipulation_planner/lifecycle/activate', LifecycleState)
	# 			req = LifecycleStateRequest
	# 			res = srv(req);
	# 		except rospy.ServiceException, e:
	# 			print "Service call failed: %s"%e
	# 	except rospy.ServiceException, e:
	# 		print "Service call failed: %s"%e

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
			return True;
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def coarse_motion(self, str1, num1, str2, num2):
		rospy.wait_for_service('/motion_executor/execute_coarse_motion')
		try:
			service = rospy.ServiceProxy('/motion_executor/execute_coarse_motion', ExecuteCoarseMotion)

			req = ExecuteCoarseMotionRequest()
			req.start.group_name = 'manipulator_tool0'
			req.start.type = str1
			req.start.bin_index = num1
			req.target.group_name = 'manipulator_tool0'
			req.target.type = str2
			req.target.bin_index = num2
			
			res = service(req);
			return True;
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def move_back_and_forth(self):
		print "starting back and forth"
		rospy.wait_for_service('/manipulation_planner/plan_camera_motion')
		print "waited for fn"
		try:
			service = rospy.ServiceProxy('/manipulation_planner/plan_camera_motion', PlanCameraMotion)

			req = PlanCameraMotionRequest()
			req.bin_index = 3
			req.relative_transform.header.frame_id = 'gripper_tool0'

			req.relative_transform.pose.position.x = 0.1
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
				reverse_traj = res.reverse_trajectory
				res = service(req)
				try:
					service = rospy.ServiceProxy('/motion_executor/execute_fine_motion', ExecuteFineMotion)
					req = ExecuteFineMotionRequest()
					req.trajectory = reverse_traj
					res = service(req)
					return True
				except rospy.ServiceException, e:
					print "Service call failed: %s"%e
				return True
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

if __name__ == "__main__":
	rospy.init_node("manipulator_test");
	tc = TestClass();

	print "Starting test"
	# tc.configure_planner();
	tc.move_to_home();
	tc.coarse_motion("home",0,"cam",3)
	print "moving b and f?";
	tc.move_back_and_forth();
	tc.coarse_motion("cam",3,"home",0)

