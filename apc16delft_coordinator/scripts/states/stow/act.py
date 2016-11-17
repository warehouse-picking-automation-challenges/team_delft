import rospy
import roslib
import smach
import time

from lib.services import ServiceState, ServiceProxy as DumpingServiceProxy
from lib.system_interface import ExecutionTracker, HIGH_VELOCITY
from lib import master_pose
from lib.objects import bypassVacuumCheck

from motoman_msgs.srv import WriteSingleIO
import apc16delft_msgs.msg as msg
import apc16delft_msgs.srv as srv


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

SLOW_ITEMS = [
	msg.Object.HANES_TUBE_SOCKS,
	msg.Object.KLEENEX_PAPER_TOWELS,
	msg.Object.FITNESS_GEAR_3LB_DUMBBELL,
	msg.Object.DASANI_WATER_BOTTLE
]

def objectVelocityScaling(object_type):
	if object_type in SLOW_ITEMS:
		return 0.1
	return 0

class PickFromTote(smach.State):
	def __init__(self, system):
		super(PickFromTote, self).__init__(
			outcomes    = ['success', 'no_grasp', 'error'],
			input_keys  = ['current_master_pose', 'job', 'grasp', 'pick_plan'],
			output_keys = ['current_master_pose']
		)

		self.system = system

	def grasp(self, userdata):
		start         = userdata.current_master_pose
		target        = master_pose.tote()
		start_to_tote = self.system.getCoarseMotion(start, target)
		if not start_to_tote:
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

		def milestoneAction(milestone):
			if milestone == msg.Milestone.VACUUM_POWER_ON:
				return lambda x, y: None
			elif milestone == msg.Milestone.SUCTION_ON:
				return enableVacuum
			elif milestone == msg.Milestone.CONTACT:
				return contactEnd
			elif milestone == msg.Milestone.LIFT:
				return liftEnd
			elif milestone == msg.Milestone.THUMB_EXTENDED:
				return extendThumb
			elif milestone == msg.Milestone.SUCTION_ROTATED:
				return rotateHead
			raise Exception('Unknown milestone event: {}'.format(milestone))

		milestone_offset = len(start_to_tote.points) - 1

		delays = []
		handlers = []

		for milestone in userdata.pick_plan.milestones:
			action = milestoneAction(milestone.event)
			if action is not None:
				handlers.append((milestone.waypoint + milestone_offset, action))
			if milestone.event == msg.Milestone.CONTACT:
				delays.append((milestone.waypoint + milestone_offset, 0.5))
				milestone_offset += 1

		self.system.enableHeadActuation(True)
		if not bypassVacuumCheck(userdata.job.object_type):
			self.system.enableVacuumPower(True)
		with ExecutionTracker('/motion_executor/trajectory_progress', handlers) as tracker:
			trajectories = [
				start_to_tote,
				userdata.pick_plan.trajectory,
			]
			userdata.current_master_pose = None
			if not self.system.executeStitchedMotion(trajectories, delays=delays, velocity_scaling=objectVelocityScaling(userdata.job.object_type)):
				return 'error'
			userdata.current_master_pose = target
		rospy.loginfo('Retreat finished')

		if bypassVacuumCheck(userdata.job.object_type):
			return 'success'

		if not self.system.readVacuum():
			rospy.logwarn('No vacuum made, returning home.')
			self.system.enableVacuumPower(False)
			userdata.current_master_pose = None
			trajectory = self.system.getCoarseMotion(target, master_pose.home())
			if not trajectory or not self.system.executeStitchedMotion([trajectory], velocity_scaling=HIGH_VELOCITY):
				return 'error'
			userdata.current_master_pose = master_pose.home()
			return 'no_grasp'

		return 'success'

	def execute(self, userdata):
		return self.grasp(userdata)

class StowInBin(smach.State):
	def __init__(self, system):
		super(StowInBin, self).__init__(
			outcomes    = ['success', 'error'],
			input_keys  = ['current_master_pose', 'job', 'grasp', 'stow_plan'],
			output_keys = ['current_master_pose']
		)

		self.system = system

	def stow(self, userdata):
		start        = userdata.current_master_pose
		target       = master_pose.bin(userdata.job.target_bin)
		trajectories = self.system.getCoarseTrajectory(start, target)
		if trajectories is None:
			return 'error'

		def disableVacuum(x, y):
			self.system.setGripperState(gripperStateFromGrasp(userdata.grasp, vacuum=False))

		coarse_length = sum(map(lambda x: len(x.points) - 1, trajectories))
		trajectories.append(userdata.stow_plan.trajectory)

		handlers = []
		handlers.append((coarse_length + userdata.stow_plan.release_waypoint, disableVacuum))

		with ExecutionTracker('/motion_executor/trajectory_progress', handlers) as tracker:
			userdata.current_master_pose = None
			if not self.system.executeStitchedMotion(trajectories, velocity_scaling=objectVelocityScaling(userdata.job.object_type)):
				return 'error'
			userdata.current_master_pose = master_pose.bin(userdata.job.target_bin)

		self.system.enableVacuumPower(False)
		return 'success'

	def execute(self, userdata):
		return self.stow(userdata)
