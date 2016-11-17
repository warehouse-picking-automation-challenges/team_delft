import rospy
import roslib
import smach

from tf import TransformListener

from lib import job as Job

from states.global_plan import ReportJobResult
from states.stow.sense  import SenseInTote, SenseTargetBin
from states.stow.plan   import PlanToteGrasp, PlanBinStow
from states.stow.act    import PickFromTote, StowInBin

def makeStowJobStateMachine(planner, tf, system):
	listener        = tf

	sm = smach.StateMachine(
		outcomes    = ['success', 'failed', 'error'],
		input_keys  = ['current_master_pose', 'job', 'bin_contents', 'tote_contents', 'blacklist'],
		output_keys = ['current_master_pose']
	)

	sm.userdata.end_master_pose = None

	with sm:
		smach.StateMachine.add(
			'Sense In Tote',
			SenseInTote(planner, tf, system),
			transitions = {
				'success'        : 'Sense Target Bin',
				'no_detection'   : 'No Detection',
				'no_object_pose' : 'No Object Pose',
				'error'          : 'error',
			},
			remapping = {
				'job'           : 'job',
				'tote_contents' : 'tote_contents',
				'tote_pose'     : 'tote_pose',
				'blacklist'     : 'blacklist',
				'object_type'   : 'object_type',
				'object_pose'   : 'object_pose',
				'object_cloud'  : 'object_cloud',
			}
		)

		smach.StateMachine.add(
			'Sense Target Bin',
			SenseTargetBin(tf, system),
			transitions = {
				'success'      : 'Plan Grasp',
				'no_detection' : 'No Detection',
				'error'        : 'error',
			},
			remapping = {
				'job'             : 'job',
				'target_bin_pose' : 'target_bin_pose',
			}
		)

		smach.StateMachine.add(
			'Plan Grasp',
			PlanToteGrasp(system),
			transitions = {
				'success' : 'Plan Stow',
				'no_plan' : 'No Plan',
				'error'   : 'error',
			},
			remapping = {
				'object_type'   : 'object_type',
				'tote_pose'     : 'tote_pose',
				'object_pose'   : 'object_pose',
				'object_cloud'  : 'object_cloud',
				'grasp'         : 'grasp',
				'grasp_plan'    : 'grasp_plan',
			}
		)

		smach.StateMachine.add(
			'Plan Stow',
			PlanBinStow(system),
			transitions = {
				'success' : 'Pick From Tote',
				'no_plan' : 'No Plan',
				'error'   : 'error',
			},
			remapping = {
				'job'             : 'job',
				'object_type'     : 'object_type',
				'target_bin_pose' : 'target_bin_pose',
				'grasp'           : 'grasp',
				'stow_plan '      : 'stow_plan',
			}
		)

		smach.StateMachine.add(
			'Pick From Tote',
			PickFromTote(system),
			transitions = {
				'success'  : 'Stow In Bin',
				'no_grasp' : 'No Grasp',
				'error'    : 'error',
			},
			remapping = {
				'current_master_pose' : 'current_master_pose',
				'job'                 : 'job',
				'grasp'               : 'grasp',
				'grasp_plan'          : 'grasp_plan',
			}
		)

		smach.StateMachine.add(
			'Stow In Bin',
			StowInBin(system),
			transitions = {
				'success'  : 'Success',
				'error'    : 'error',
			},
			remapping = {
				'current_master_pose' : 'current_master_pose',
				'job'                 : 'job',
				'grasp'               : 'grasp',
				'stow_plan'           : 'stow_plan',
			}
		)

		smach.StateMachine.add('Success',        ReportJobResult(planner, Job.StowSuccess, 'object_type'),  transitions = {'succeeded': 'success'})
		smach.StateMachine.add('No Detection',   ReportJobResult(planner, Job.NoDetection),                 transitions = {'succeeded': 'failed'})
		smach.StateMachine.add('No Object Pose', ReportJobResult(planner, Job.NoObjectPose, 'object_type'), transitions = {'succeeded': 'failed'})
		smach.StateMachine.add('No Plan',        ReportJobResult(planner, Job.NoPlan,       'object_type'), transitions = {'succeeded': 'failed'})
		smach.StateMachine.add('No Grasp',       ReportJobResult(planner, Job.NoGrasp,      'object_type'), transitions = {'succeeded': 'failed'})

	return sm

