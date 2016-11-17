import rospy
import apc16delft_msgs.msg as msg
import apc16delft_msgs.srv as srv
import motoman_msgs.srv
import std_msgs

from services import ServiceProxy as DumpingServiceProxy
from lib import master_pose

HIGH_VELOCITY = 0.3

class SystemInterface():
	def __init__(self):
		self.execute_stitched_motion = DumpingServiceProxy('/motion_executor/execute_stitched_motion', srv.ExecuteStitchedMotion)
		self.get_coarse_motion       = DumpingServiceProxy('/motion_executor/get_coarse_motion',       srv.GetCoarseMotion)
		self.execute_coarse_motion   = DumpingServiceProxy('/motion_executor/execute_coarse_motion',   srv.ExecuteCoarseMotion)
		self.plan_camera_motion      = DumpingServiceProxy('/manipulation_planner/plan_camera_motion', srv.PlanCameraMotion)
		self.execute_fine_motion     = DumpingServiceProxy('/motion_executor/execute_fine_motion',     srv.ExecuteFineMotion)
		self.read_single_io          = DumpingServiceProxy('/read_single_io',                          motoman_msgs.srv.ReadSingleIO)
		self.write_single_io         = DumpingServiceProxy('/write_single_io',                         motoman_msgs.srv.WriteSingleIO)
		self.set_gripper_state       = DumpingServiceProxy('/gripper_driver/set_state',                srv.SetGripperState)

	def getCoarseMotion(self, start, target):
		result = self.get_coarse_motion(start=start, target=target)
		if result.error.code:
			rospy.logerr('Failed to get coarse motion with error {}: {}'.format(result.error.code, result.error.message))
			return None
		return result.trajectory

	def getCoarseTrajectory(self, start, target):
		if start == target:
			return []

		try:
			trajectory = self.getCoarseMotion(start, target)
			if trajectory is not None:
				return [trajectory]
		except rospy.ServiceException:
			pass

		start_to_home  = self.getCoarseMotion(start, master_pose.home())
		if start_to_home is None:
			return None

		home_to_target = self.getCoarseMotion(master_pose.home(), target)
		if home_to_target is None:
			return None

		return [start_to_home, home_to_target]

	def executeCoarseMotion(self, start, target):
		rospy.loginfo('Starting motion (coarse).')
		result = self.execute_coarse_motion(start=start, target=target)
		if result.error.code:
			rospy.logerr('Motion failed (coarse) with error {}: {}'.format(result.error.code, result.error.message))
			return False
		rospy.loginfo('Motion finished (coarse) according to moveit, which is probably not true.')
		return True

	def planCameraMotion(self, bin_index, offset):
		rospy.loginfo('Requesting camera motion plan for bin {} with offset:\n {}'.format(bin_index, offset))
		result = self.plan_camera_motion(bin_index=bin_index, relative_transform=offset)
		if result.error.code:
			rospy.logerr('Failed to plan camera motion with error {}: {}'.format(result.error.code, result.error.message))
			return None
		return result;

	def executeFineMotion(self, trajectory):
		rospy.loginfo('Starting motion (fine)')
		result = self.execute_fine_motion(trajectory = trajectory)
		if result.error.code:
			rospy.logerr('Motion failed (fine) with error {}: {}'.format(result.error.code, result.error.message))
			return False
		rospy.loginfo('Motion finished (fine) according to moveit, which is probably not true.')
		return True

	def executeStitchedMotion(self, trajectories, velocity_scaling = 0, delays = []):
		rospy.loginfo('Starting motion (stitched)')

		delay_indices = []
		delay_times   = []
		for index, delay in delays:
			delay_indices.append(index)
			delay_times.append(delay)

		result = self.execute_stitched_motion(
			trajectories = trajectories,
			velocity_scaling = velocity_scaling,
			delay_indices = delay_indices,
			delay_times = delay_times
		)

		if result.error.code:
			rospy.logerr('Motion failed (stitched) with error {}: {}'.format(result.error.code, result.error.message))
			return False
		rospy.loginfo('Motion finished (stitched) according to moveit, which is probably not true.')
		return True

	def readSingleIo(self, address):
		try:
			return self.read_single_io(address = address).value
		except rospy.ServiceException as e:
			rospy.logerr('Failed to read IO {}: {}'.format(address, str(e)))
			return None

	def writeSingleIo(self, address, value):
		try:
			self.write_single_io(address = address, value = value)
			return True
		except rospy.ServiceException as e:
			rospy.logerr('Failed to set IO {} to {}: {}'.format(address, value, str(e)))
			return False

	def readVacuum(self):
		return self.readSingleIo(20030)

	def enableVacuumPower(self, enable):
		return self.writeSingleIo(10016, enable)

	def setGripperState(self, state):
		try:
			result = self.set_gripper_state(state)
			return True
		except rospy.ServiceException as e:
			rospy.logerr('Failed to set gripper state with error {}: {}'.format(result.error.code, result.error.message))
			return False

	def enableHeadActuation(self, value):
		return self.writeSingleIo(10014, not value)

	def enableLed(self, enabled):
		"""
		Enable or disable LEDs.
		"""
		request = srv.SetGripperStateRequest()
		request.state.led_enabled = enabled
		return self.setGripperState(request)

class ExecutionTracker():
	def __init__(self, topic, handlers):
		self.__handlers   = handlers;
		self.__subscriber = rospy.Subscriber(topic, std_msgs.msg.Int32, self.__callback)
		self.__waypoint   = -1;

	def __enter__(self):
		return self

	def __exit__(self, type, value, traceback):
		self.__subscriber.unregister()
		return False

	def __callback(self, msg):
		for waypoint, callback in self.__handlers:
			if waypoint > self.__waypoint and waypoint <= msg.data:
				callback(waypoint, msg.data)
		self.__waypoint = msg.data
