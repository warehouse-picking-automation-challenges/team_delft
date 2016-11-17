import rospy
import roslib
import smach

from states.global_plan import ReportJobResult
from states.pick.sense  import SenseBinPick
from states.pick.plan   import PlanBinPick, PlanToteDrop
from states.pick.act    import PickFromBin, DropInTote

from lib import job as Job

def makePickJobStateMachine(tf, planner, system):
	sm = smach.StateMachine(
		outcomes    = ['success', 'failed', 'error'],
		input_keys  = ['job', 'bin_contents','current_master_pose'],
		output_keys = ['current_master_pose']
	)

	with sm:
		smach.StateMachine.add(
			'Sense Bin Pick',
			SenseBinPick(tf, system),
			transitions = {
				'success'      : 'Plan Bin Pick',
				'occluded'     : 'Occluded',
				'no_detection' : 'No Detection',
				'error'        : 'error'
			},
			remapping = {
				'current_master_pose' : 'current_master_pose',
				'job'                 : 'job',
				'bin_contents'        : 'bin_contents',
				'source_bin_pose'     : 'source_bin_pose',
				'object_pose'         : 'object_pose',
				'object_cloud'        : 'object_cloud',
				'robot_transform'     : 'robot_transform',
				'occluder'            : 'occluder',
				'grasp_occluders'     : 'grasp_occluders',
			}
		)

		smach.StateMachine.add(
			'Plan Bin Pick',
			PlanBinPick(system),
			transitions = {
				'success' : 'Plan Tote Drop',
				'no_plan' : 'No Plan',
				'error'   : 'error'
			},
			remapping = {
				'job'             : 'job',
				'bin_contents'    : 'bin_contents',
				'source_bin_pose' : 'source_bin_pose',
				'object_pose'     : 'object_pose',
				'object_cloud'    : 'object_cloud',
				'robot_transform' : 'robot_transform',
				'grasp'           : 'grasp',
				'pick_plan'       : 'pick_plan',
			}
		)

		smach.StateMachine.add(
			'Plan Tote Drop',
			PlanToteDrop(system),
			transitions = {
				'success' : 'Pick From Bin',
				'error'   : 'error'
			},
			remapping = {
				'job'             : 'job',
				'bin_contents'    : 'bin_contents',
				'source_bin_pose' : 'source_bin_pose',
				'object_pose'     : 'object_pose',
				'object_cloud'    : 'object_cloud',
				'robot_transform' : 'robot_transform',
				'grasp'           : 'grasp',
				'pick_plan'       : 'pick_plan',
			}
		)

		smach.StateMachine.add(
			'Pick From Bin',
			PickFromBin(system),
			transitions = {
				'succeeded' : 'Drop In Tote',
				'no_grasp'  : 'No Grasp',
				'error'     : 'error',
			},
			remapping = {
				'current_master_pose' : 'current_master_pose',
				'job'                 : 'job',
				'grasp'               : 'grasp',
				'pick_plan'           : 'pick_plan',
			}
		)

		smach.StateMachine.add(
			'Drop In Tote',
			DropInTote(tf, system),
			transitions = {
				'succeeded'    : 'Success',
				'failed_pinch' : 'No Grasp',
				'error'        : 'error',
			},
			remapping = {
				'current_master_pose' : 'current_master_pose',
				'job'                 : 'job',
				'grasp'               : 'grasp',
				'tote_plan'           : 'tote_plan',
			}
		)

		smach.StateMachine.add('Success',      ReportJobResult(planner, Job.Success),                                           transitions = {'succeeded': 'success'})
		smach.StateMachine.add('Occluded',     ReportJobResult(planner, Job.Occluded,    'occluder'),                           transitions = {'succeeded': 'failed'})
		smach.StateMachine.add('No Detection', ReportJobResult(planner, Job.NoDetection, 'occluder'),                           transitions = {'succeeded': 'failed'})
		smach.StateMachine.add('No Plan',      ReportJobResult(planner, Job.NoPlan,       grasp_occluders = 'grasp_occluders'), transitions = {'succeeded': 'failed'})
		smach.StateMachine.add('No Grasp',     ReportJobResult(planner, Job.NoGrasp,      grasp_occluders = 'grasp_occluders'), transitions = {'succeeded': 'failed'})

	return sm

