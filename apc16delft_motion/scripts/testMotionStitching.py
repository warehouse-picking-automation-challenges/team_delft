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
			print "Service call failed: %s"%ez

	def gettraj(self, str1, num1, str2, num2 ):
		rospy.wait_for_service('/motion_executor/get_coarse_motion')
		try:
			service = rospy.ServiceProxy('/motion_executor/get_coarse_motion', GetCoarseMotion)

			req = GetCoarseMotionRequest()
			req.start.group_name = 'manipulator_tool0'
			req.start.type = str1
			req.start.bin_index = num1
			req.target.group_name = 'manipulator_tool0'
			req.target.type = str2
			req.target.bin_index = num2
			
			res = service(req);
			return res
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			return False

	def stitch(self, scaling, stopwp, stoptime):
		res1 = self.gettraj("home",0,"bin",1)
		res2 = self.gettraj("bin",1,"cam",1)
		res3 = self.gettraj("cam",1,"bin",1)
		res4 = self.gettraj("bin",1,"home",0)

		rospy.wait_for_service('/motion_executor/execute_stitched_motion')
		try:
			service = rospy.ServiceProxy('/motion_executor/execute_stitched_motion', ExecuteStitchedMotion)
			req = ExecuteStitchedMotionRequest()

			req.trajectories.append(res1.trajectory)
			req.trajectories.append(res2.trajectory)
			req.trajectories.append(res3.trajectory)
			req.trajectories.append(res4.trajectory)

			req.delay_indices.append(stopwp)
			req.delay_times.append(stoptime)
			
			req.velocity_scaling = scaling
			# req.trajectories.append(res5.trajectory)
			
			res = service(req);
			return res
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			return False

if __name__ == "__main__":
	rospy.init_node("manipulator_test")
	tc = TestClass()

	print "Starting test"
	tc.move_to_home()
	print "making some moves"
	tc.stitch(0.5,5,1.0)
	tc.stitch(1,5,2.0)

