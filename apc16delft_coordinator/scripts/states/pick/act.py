import rospy
import roslib
import smach
import tf
import time

from lib.services import ServiceState, ServiceProxy as DumpingServiceProxy
from lib.system_interface import ExecutionTracker
from lib import master_pose
from lib.objects import bypassVacuumCheck
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point

import apc16delft_msgs.msg as msg
import apc16delft_msgs.srv as srv

def tfPositionToGeometry(position):
	result = Point()
	result.x = position[0]
	result.y = position[1]
	result.z = position[2]
	return result

def tfOrientationToGeometry(orientation):
	result = Quaternion()
	result.x = orientation[0]
	result.y = orientation[1]
	result.z = orientation[2]
	result.w = orientation[3]
	return result

def tfPoseToGeometry(pose):
	result = Pose()
	result.position    = tfPositionToGeometry(pose[0])
	result.orientation = tfOrientationToGeometry(pose[1])
	return result

def tfLookup(tf, target_frame, source_frame, time, timeout = rospy.Duration(1)):
	tf.waitForTransform(target_frame, source_frame, time, timeout)
	return tfPoseToGeometry(tf.lookupTransform(target_frame, source_frame, time))

SLOW_ITEMS = [
	msg.Object.HANES_TUBE_SOCKS,
	msg.Object.KLEENEX_PAPER_TOWELS,
	msg.Object.FITNESS_GEAR_3LB_DUMBBELL,
	msg.Object.DASANI_WATER_BOTTLE
]

PINCH_STRATEGIES = [
	msg.GraspCandidate.DUMBBELL_SIDEWARDS,
	msg.GraspCandidate.DUMBBELL_SIDEWARDS,
	msg.GraspCandidate.PENCIL_CUP_INWARDS,
	msg.GraspCandidate.PENCIL_CUP_SIDEWAYS,
	msg.GraspCandidate.PENCIL_CUP_STANDING,
]

def objectVelocityScaling(object_type):
	if object_type in SLOW_ITEMS:
		return 0.1
	return 0

def gripperStateFromGrasp(grasp, vacuum, leds = False):
	return msg.GripperState(
		thumb_extended= (
			grasp.strategy == msg.GraspCandidate.PENCIL_CUP_INWARDS or
			grasp.strategy == msg.GraspCandidate.PENCIL_CUP_SIDEWAYS or
			grasp.strategy == msg.GraspCandidate.PENCIL_CUP_STANDING or
			grasp.strategy == msg.GraspCandidate.DUMBBELL_SIDEWARDS),
		thumb_rotated=False,
		vacuum_rotated=(
			grasp.strategy == msg.GraspCandidate.STRATEGY_VACUUM_V or
			grasp.strategy == msg.GraspCandidate.VACUUM_V_BIG or
			grasp.strategy == msg.GraspCandidate.DUMBBELL_INWARDS),
		vacuum_enabled=vacuum,
		led_enabled=leds
	)

class PickFromBin(smach.State):
	def __init__(self, system):
		smach.State.__init__(self,
			outcomes=['succeeded', 'no_grasp', 'error'],
			input_keys=['current_master_pose', 'job', 'grasp', 'pick_plan'],
			output_keys=['current_master_pose']
		)

		self.system = system

	def execute(self, userdata):
		# Move gripper to pre-pickup pose.
		start  = userdata.current_master_pose
		target = master_pose.bin(userdata.job.source_bin)
		cam_to_bin = self.system.getCoarseMotion(start, target)
		if cam_to_bin is None:
			return 'error'

		gripper_state = gripperStateFromGrasp(userdata.grasp, vacuum=False)
		# Set the gripper state.
		if not self.system.setGripperState(gripper_state):
			return 'error'

		def enableVacuum(trigger, last_waypoint):
			rospy.loginfo('Approach finished, enabling vacuum.')
			gripper_state.vacuum_enabled = True
			self.system.setGripperState(gripper_state)
			if userdata.job.object_type == msg.apc16delft_msgs.msg.Object.HANES_TUBE_SOCKS:
				time.sleep(0.2)
				self.system.enableHeadActuation(False)

		def rotateHead(trigger, last_waypoint):
			rospy.loginfo('Approach finished, rotating head.')
			gripper_state.vacuum_rotated = True
			self.system.setGripperState(gripper_state)

		def extendThumb(trigger, last_waypoint):
			rospy.loginfo('Extending thumb.')
			gripper_state.thumb_extended = True
			self.system.setGripperState(gripper_state)

		vacuum0 = None
		vacuum1 = None

		def contactEnd(x, y):
			rospy.loginfo('Contact trajectory finished.')
			vacuum0 = self.system.readVacuum() # Apparantly this is safe in python regardles of multithreading

		def liftEnd(x, y):
			rospy.loginfo('Lift trajectory finished.')
			vacuum1 = self.system.readVacuum()

		def milestoneAction(event):
			if event == msg.Milestone.VACUUM_POWER_ON:
				return lambda x, y: None
			elif event == msg.Milestone.SUCTION_ON:
				return enableVacuum
			elif event == msg.Milestone.CONTACT:
				return contactEnd
			elif event == msg.Milestone.LIFT:
				return liftEnd
			elif event == msg.Milestone.THUMB_EXTENDED:
				return extendThumb
			elif event == msg.Milestone.SUCTION_ROTATED:
				return rotateHead
			return None

		milestone_offset = len(cam_to_bin.points) - 1

		delays   = []
		handlers = []

		for milestone in userdata.pick_plan.milestones:
			action = milestoneAction(milestone.event)
			if action is not None:
				handlers.append((milestone.waypoint + milestone_offset, action))
			if milestone.event == msg.Milestone.CONTACT:
				delays.append((milestone.waypoint + milestone_offset, 0.5))
				milestone_offset += 1

		self.system.enableHeadActuation(True)
		self.system.enableVacuumPower(True)

		with ExecutionTracker('/motion_executor/trajectory_progress', handlers) as tracker:
			trajectories = [
				cam_to_bin,
				userdata.pick_plan.trajectory,
			]
			userdata.current_master_pose = None
			if not self.system.executeStitchedMotion(trajectories, delays = delays, velocity_scaling = objectVelocityScaling(userdata.job.object_type)):
				return 'error'
			userdata.current_master_pose = target
		rospy.loginfo('Retreat finished')

		vacuum2 = self.system.readVacuum()

		# Don't check vacuum for pinch strageties.
		if userdata.grasp.strategy in PINCH_STRATEGIES:
			self.system.enableVacuumPower(False)
			return 'succeeded'

		if not vacuum2:
			self.system.enableVacuumPower(False)
			return 'no_grasp'

		return 'succeeded'
		#return 'succeeded' if vacuum_made else 'no_grasp'


class DropInTote(smach.State):
	def __init__(self, tf, system):
		smach.State.__init__(self,
			outcomes=['succeeded', 'failed_pinch', 'error'],
			input_keys=['job', 'grasp', 'tote_plan', 'current_master_pose'],
			output_keys=['current_master_pose']
		)

		self.tf = tf

		self.get_camera_data = rospy.ServiceProxy('/tote_camera/get_data',           srv.GetCameraData)
		self.detect_objects  = DumpingServiceProxy('/tote_detection/detect_objects', srv.DetectObjects, dump_request = False)

		self.system = system

	def execute(self, userdata):
		tote_plan = userdata.tote_plan
		gripper_state = gripperStateFromGrasp(userdata.grasp, vacuum=False)
		gripper_state.thumb_extended = False

		def deactivateVacuum(*args):
			rospy.loginfo("At release point. Releasing vacuum.")
			self.system.setGripperState(gripper_state)

		handlers = [(len(tote_plan[0].points) + len(tote_plan[1].points) - 1, deactivateVacuum)]

		userdata.current_master_pose = None
		with ExecutionTracker('/motion_executor/trajectory_progress', handlers) as tracker:
			if not self.system.executeStitchedMotion(userdata.tote_plan, velocity_scaling = objectVelocityScaling(userdata.job.object_type)):
				return 'error'

		userdata.current_master_pose = master_pose.tote(0, master_pose.motionGroupFromGraspStrategy(userdata.grasp.strategy))

		self.system.enableVacuumPower(False)
		self.system.setGripperState(gripperStateFromGrasp(userdata.grasp, vacuum=False))

		#if userdata.job.object_type in [msg.Object.ROLODEX_JUMBO_PENCIL_CUP, msg.Object.FITNESS_GEAR_3LB_DUMBBELL]:
		#	# Verify that we dropped the pinched item in the tote
		#	return self.verifyPinchSuccess(userdata)

		return 'succeeded'

	def verifyPinchSuccess(self, userdata):
		# Move to home
		home_trajectory = self.system.getCoarseTrajectory(userdata.current_master_pose, master_pose.home())
		if home_trajectory is None:
			return 'succeeded' # Naive way of accepting a succeeded pinch if somehow we cannot move to home..
		userdata.current_master_pose = None
		if not self.system.executeStitchedMotion(home_trajectory):
			return 'error'
		userdata.current_master_pose = master_pose.home()

		# Take tote picture
		camera_data     = self.recordCameraData(dump = True)
		time            = camera_data.point_cloud.header.stamp

		tote_pose  = PoseStamped(header = camera_data.point_cloud.header, pose = tfLookup(self.tf, 'world', 'tote', time))

		# Perform the detection.
		detection = self.detect_objects(
			color       = camera_data.color,
			point_cloud = camera_data.point_cloud,
			objects     = [userdata.job.object_type],
			bin_pose    = tote_pose.pose,
			bin_index   = -1 # Something something constant YadaYada
		)

		if not detection.objects:
			return 'failed_pinch'
		return 'succeeded'

	def retrieveCameraData(self, dump, publish):
		"""
		Record camera data.
		"""

		for retry in range(2):
			try:
				return self.get_camera_data(dump=dump, publish=publish)
			except rospy.ServiceException as e:
				rospy.logwarn('Failed to retrieve data from gripper camera: {}'.format(e))
		return self.get_camera_data(dump=dump, publish=publish)

	def recordCameraData(self, dump = False, publish = False):
		"""
		Record data
		"""
		result = self.retrieveCameraData(dump, publish)
		return result

