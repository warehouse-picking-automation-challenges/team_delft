import rospy
import roslib
import smach

import tf
from lib.objects import isDeformable
from lib.services import ServiceState, ServiceProxy as DumpingServiceProxy
from lib import master_pose
from geometry_msgs.msg import Pose, Point, Quaternion
import apc16delft_msgs.msg as msg
import apc16delft_msgs.srv as srv

TOOL2_STRATEGIES = [
	msg.GraspCandidate.DUMBBELL_INWARDS,
	msg.GraspCandidate.DUMBBELL_SIDEWARDS,
	msg.GraspCandidate.PENCIL_CUP_INWARDS,
	msg.GraspCandidate.PENCIL_CUP_SIDEWAYS,
	msg.GraspCandidate.PENCIL_CUP_STANDING,
]

def motionGroupFromGraspStrategy(strategy):
	if strategy == msg.GraspCandidate.STRATEGY_VACUUM_V: return msg.MasterPoseDescriptor.GROUP_TOOL1
	if strategy == msg.GraspCandidate.STRATEGY_VACUUM_H: return msg.MasterPoseDescriptor.GROUP_TOOL0
	if strategy == msg.GraspCandidate.VACUUM_V_BIG: return msg.MasterPoseDescriptor.GROUP_TOOL1
	if strategy == msg.GraspCandidate.VACUUM_H_BIG: return msg.MasterPoseDescriptor.GROUP_TOOL0
	if strategy in TOOL2_STRATEGIES: return msg.MasterPoseDescriptor.GROUP_TOOL2
	assert False, 'Invalid grasp strategy'

def identityPose():
	return Pose(position = Point(0, 0, 0), orientation = Quaternion(0, 0, 0, 1))


class PlanToteGrasp(smach.State):
	def __init__(self, system):
		super(PlanToteGrasp, self).__init__(
			outcomes=['success', 'no_plan', 'error'],
			input_keys=['object_type', 'tote_pose', 'object_pose', 'object_cloud'],
			output_keys=['grasp', 'pick_plan']
		)

		self.synthesize_grasp            = DumpingServiceProxy('/grasp_synthesizer/synthesize_grasp',          srv.SynthesizeGrasp)
		self.synthesize_deformable_grasp = DumpingServiceProxy('/grasp_synthesizer/synthesize_deformable',     srv.SynthesizeDeformable)
		self.plan_manipulation           = DumpingServiceProxy('/manipulation_planner/plan_tote_manipulation', srv.PlanManipulation)
		self.update_candidates           = DumpingServiceProxy('/manipulation_planner/update_reachability', srv.PruneGraspCandidates)

	def synthesizeDeformableGrasps(self, object_type, tote_pose, object_cloud):
		grasp_request = srv.SynthesizeDeformableRequest(
			object          = msg.Object(type = object_type),
			bin_index       = -1, # TODO: Use a constant
			bin_pose        = tote_pose,
			point_cloud     = object_cloud,
			cloud_transform = identityPose()
		)

		rospy.loginfo("Synthesizing grasps for deformable object.")
		grasps = self.synthesize_deformable_grasp(grasp_request)
		if grasps.error:
			rospy.logerr('Synthesizing grasps failed with error {}: {}'.format(grasps.error, grasps.error))
			return None
		return grasps

	def synthesizeRigidGrasps(self, object_type, object_pose, tote_pose, object_cloud):
		grasp_request = srv.SynthesizeGraspRequest(
			object          = msg.Object(type = object_type),
			bin_index       = -1, # TODO: Use a constant
			object_pose     = object_pose,
			bin_pose        = tote_pose,
			point_cloud     = object_cloud,
			cloud_transform = identityPose()
		)

		grasps = self.synthesize_grasp(grasp_request)
		if grasps.error:
			rospy.logerr('Synthesizing grasps failed with error {}: {}'.format(grasps.error, grasps.error))
			return None
		return grasps

	def planGrasp(self, userdata):
		if isDeformable(userdata.object_type):
			grasps = self.synthesizeDeformableGrasps(userdata.object_type, userdata.tote_pose, userdata.object_cloud)
		else:
			grasps = self.synthesizeRigidGrasps(userdata.object_type, userdata.object_pose, userdata.tote_pose, userdata.object_cloud)

		if grasps is None:
			return 'error'

		if not grasps:
			rospy.logerr('No grasp candidates found. Cannot grasp item.')
			return 'no_plan'

		# Try to plan a collision-free path in order of score.
		grasps = sorted(grasps.candidates, key = lambda x : x.score, reverse = True)
		while grasps:
			grasp = grasps[0]
			motion_group = motionGroupFromGraspStrategy(grasp.strategy)

			starting_pose = master_pose.tote(0, motion_group)

			plan_request = srv.PlanManipulationRequest(
				bin_index    = -1, # TODO: Use a constant
				object_type  = userdata.object_type,
				grasp        = grasp,
				object_pose  = userdata.object_pose,
				bin_pose     = userdata.tote_pose
			)

			plan = self.plan_manipulation(plan_request)

			if plan.error.code:
				rospy.logwarn("Generated grasp path has collision. Trying next grasp candidate.")
				grasps = self.update_candidates(object_type=userdata.object_type, candidates=grasps, bin_index=-1, bin_pose=userdata.tote_pose) # TODO: Use a constant for the bin index
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
			return self.planGrasp(userdata)
		except rospy.ServiceException as e:
			rospy.logerr("Service execution failed: %s", e)
			return 'error'


class PlanBinStow(smach.State):
	def __init__(self, system):
		super(PlanBinStow, self).__init__(
			outcomes    = ['success', 'no_plan', 'error'],
			input_keys  = ['job', 'target_bin_pose', 'grasp', 'bin_contents', 'initial_bin_contents'],
			output_keys = ['stow_plan']
		)

		self.plan_manipulation = DumpingServiceProxy('/manipulation_planner/plan_tote_manipulation', srv.PlanManipulation)
		self.plan_stow_motion  = DumpingServiceProxy('/manipulation_planner/plan_stow_motion',       srv.PlanStowMotion)


	def planStow(self, userdata):
		grasp = msg.GraspCandidate(
			score                = userdata.grasp.score,
			strategy             = userdata.grasp.strategy,
			stamped_pose         = userdata.grasp.stamped_pose,
			pre_grasp_offset     = userdata.grasp.pre_grasp_offset,
			angled_approach      = userdata.grasp.angled_approach,
			side_angled_approach = userdata.grasp.side_angled_approach,
		)

		if userdata.job.object_type == msg.Object.HANES_TUBE_SOCKS:
			grasp.strategy = msg.GraspCandidate.STRATEGY_VACUUM_H

		plan_request = srv.PlanStowMotionRequest(
			bin_index       = userdata.job.target_bin,
			object_type     = userdata.job.object_type,
			bin_pose        = userdata.target_bin_pose,
			grasp_candidate = grasp
		)

		plan = self.plan_stow_motion(plan_request)

		if plan.error.code:
			rospy.logerr("No collision free grasp candidates found for item.")
			return 'no_plan'

		userdata.stow_plan = plan
		return 'success'

	def execute(self, userdata):
		userdata.stow_plan  = None

		try:
			return self.planStow(userdata)
		except rospy.ServiceException as e:
			rospy.logerr("Service execution failed: %s", e)
			return 'error'
