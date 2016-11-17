import rospy
import roslib
import smach

import tf
from lib.objects import isDeformable
from lib.services import ServiceState, ServiceProxy as DumpingServiceProxy
from lib import master_pose
from geometry_msgs.msg import Pose, Point, Quaternion
import apc16delft_msgs.msg as msgs
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

def motionGroupFromGraspStrategy(strategy):
	if strategy == msgs.GraspCandidate.STRATEGY_VACUUM_V: return msgs.MasterPoseDescriptor.GROUP_TOOL1
	if strategy == msgs.GraspCandidate.STRATEGY_VACUUM_H: return msgs.MasterPoseDescriptor.GROUP_TOOL0
	if strategy == msgs.GraspCandidate.VACUUM_V_BIG: return msgs.MasterPoseDescriptor.GROUP_TOOL1
	if strategy == msgs.GraspCandidate.VACUUM_H_BIG: return msgs.MasterPoseDescriptor.GROUP_TOOL0
	if strategy == msgs.GraspCandidate.DUMBBELL_INWARDS   or strategy == msgs.GraspCandidate.DUMBBELL_SIDEWARDS or strategy == msgs.GraspCandidate.PENCIL_CUP_INWARDS or strategy == msgs.GraspCandidate.PENCIL_CUP_SIDEWAYS or strategy == msgs.GraspCandidate.PENCIL_CUP_STANDING :
		return msgs.MasterPoseDescriptor.GROUP_TOOL2
	assert False, 'Invalid grasp strategy'

class PlanBinPick(smach.State):
	def __init__(self, system):
		smach.State.__init__(self,
			outcomes=['success', 'no_plan', 'error'],
			input_keys=['job', 'bin_contents', 'source_bin_pose', 'object_pose','object_cloud','robot_transform'],
			output_keys=['grasp', 'pick_plan']
		)
		self.system                      = system
		self.synthesize_grasp            = DumpingServiceProxy('/grasp_synthesizer/synthesize_grasp',       srv.SynthesizeGrasp)
		self.synthesize_deformable_grasp = DumpingServiceProxy('/grasp_synthesizer/synthesize_deformable', srv.SynthesizeDeformable)
		self.update_candidates           = DumpingServiceProxy('/manipulation_planner/update_reachability', srv.PruneGraspCandidates)
		self.plan_manipulation           = DumpingServiceProxy('/manipulation_planner/plan_manipulation',   srv.PlanManipulation)

	def synthesizeDeformableGrasps(self, job, bin_pose, object_cloud, cloud_transform):
		grasp_request = srv.SynthesizeDeformableRequest(
			object          = msgs.Object(type = job.object_type),
			bin_index       = job.source_bin,
			bin_pose        = bin_pose,
			point_cloud     = object_cloud,
			cloud_transform = cloud_transform
		)

		rospy.loginfo("Synthesizing grasps for deformable object.")
		grasps = self.synthesize_deformable_grasp(grasp_request)
		if grasps.error:
			rospy.logerr('Synthesizing grasps failed with error {}: {}'.format(grasps.error, grasps.error))
			return None
		return grasps

	def synthesizeRigidGrasps(self, job, object_pose, bin_pose, object_cloud, cloud_transform):
		grasp_request = srv.SynthesizeGraspRequest(
			object          = msgs.Object(type = job.object_type),
			bin_index       = job.source_bin,
			object_pose     = object_pose,
			bin_pose        = bin_pose,
			point_cloud     = object_cloud,
			cloud_transform = cloud_transform
		)

		grasps = self.synthesize_grasp(grasp_request)
		if grasps.error:
			rospy.logerr('Synthesizing grasps failed with error {}: {}'.format(grasps.error, grasps.error))
			return None
		return grasps

	def step(self, userdata):
		job = userdata.job

		if isDeformable(job.object_type):
			grasps = self.synthesizeDeformableGrasps(job, userdata.source_bin_pose, userdata.object_cloud, userdata.robot_transform)
		else:
			grasps = self.synthesizeRigidGrasps(job, userdata.object_pose, userdata.source_bin_pose, userdata.object_cloud, userdata.robot_transform)

		if grasps is None:
			return 'error'

		if not grasps:
			rospy.logerr('No grasp candidates found. Cannot grasp item.')
			return 'no_plan'

		# Try to plan a collision-free path in order of score.
		grasps = sorted(grasps.candidates, key = lambda x : x.score, reverse = True)
		while grasps:
			grasp         = grasps[0]
			motion_group  = motionGroupFromGraspStrategy(grasp.strategy)
			starting_pose = master_pose.bin(job.source_bin, motion_group)

			plan_request = srv.PlanManipulationRequest(
				bin_index    = job.source_bin,
				object_type  = job.object_type,
				grasp        = grasp,
				object_pose  = userdata.object_pose,
				bin_pose     = userdata.source_bin_pose
			)

			plan = self.plan_manipulation(plan_request)

			if plan.error.code:
				rospy.logwarn("Generated grasp path has collision. Trying next grasp candidate.")
				grasps = self.update_candidates(candidates=grasps, bin_index=job.source_bin, object_type=job.object_type,bin_pose=userdata.source_bin_pose)
				grasps = sorted(grasps.pruned_candidates, key = lambda x : x.score, reverse = True)
				continue


			userdata.grasp     = grasp
			userdata.pick_plan = plan
			return 'success'

		# Loop exited, so no valid candidates found
		rospy.logerr("No collision free grasp candidates found for item.")
		return 'no_plan'

	def execute(self, userdata):
		userdata.grasp     = None
		userdata.pick_plan = None

		try:
			return self.step(userdata)
		except rospy.ServiceException as e:
			rospy.logerr("Service execution failed: %s", e)
			return 'error'

class PlanToteDrop(smach.State):
	def __init__(self, system):
		smach.State.__init__(self,
			outcomes    = ['success', 'error'],
			input_keys  = ['job', 'grasp'],
			output_keys = ['tote_plan']
		)
		self.system = system

	def step(self, userdata):
		job           = userdata.job
		motion_group  = motionGroupFromGraspStrategy(userdata.grasp.strategy)
		starting_pose = master_pose.bin(job.source_bin, motion_group)

		main_tote_master_pose = master_pose.tote(0, motion_group)
		in_tote_master_pose   = master_pose.tote(job.target_tote_cell + 1, motion_group)

		tote_plan = []
		tote_plan.append(self.system.getCoarseMotion(starting_pose, main_tote_master_pose))
		if not tote_plan[-1]:
			return 'error'
		tote_plan.append(self.system.getCoarseMotion(main_tote_master_pose, in_tote_master_pose))
		if not tote_plan[-1]:
			return 'error'
		tote_plan.append(self.system.getCoarseMotion(in_tote_master_pose, main_tote_master_pose))
		if not tote_plan[-1]:
			return 'error'

		userdata.tote_plan = tote_plan
		return 'success'

	def execute(self, userdata):
		userdata.tote_plan = None
		try:
			return self.step(userdata)
		except rospy.ServiceException as e:
			rospy.logerr("Service execution failed: %s", e)
			return 'error'
