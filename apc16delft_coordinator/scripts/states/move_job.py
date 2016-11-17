import rospy
import roslib
import smach

from tf import TransformListener

from lib import job as Job

from states.global_plan import ReportJobResult
from states.pick.sense  import SenseBinPick
from states.stow.sense  import SenseTargetBin
from states.pick.plan   import PlanBinPick
from states.stow.plan   import PlanBinStow
from states.pick.act    import PickFromBin
from states.stow.act    import StowInBin

def makeMoveJobStateMachine(tf, planner, system):

	sm = smach.StateMachine(
		outcomes    = ['success', 'failed', 'error'],
		input_keys  = ['job', 'bin_contents', 'current_master_pose'],
		output_keys = ['current_master_pose']
	)

	with sm:
		smach.StateMachine.add(
			'Sense Bin Pick',
			SenseBinPick(tf, system),
			transitions = {
				'success'      : 'Sense Target Bin',
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
			}
		)

		smach.StateMachine.add(
			'Sense Target Bin',
			SenseTargetBin(tf, system),
			transitions = {
				'success'      : 'Plan Bin Pick',
				'no_detection' : 'No Detection',
				'error'        : 'error',
			},
			remapping = {
				'job'             : 'job',
				'target_bin_pose' : 'target_bin_pose',
			}
		)

		smach.StateMachine.add(
			'Plan Bin Pick',
			PlanBinPick(system),
			transitions = {
				'success' : 'Plan Bin Stow',
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
			'Plan Bin Stow',
			PlanBinStow(system),
			transitions = {
				'success' : 'Pick From Bin',
				'no_plan' : 'No Plan',
				'error'   : 'error'
			},
			remapping = {
				'job'           : 'job',
				'grasp'         : 'grasp',
				'stow_plan '    : 'stow_plan',
			}
		)

		smach.StateMachine.add(
			'Pick From Bin',
			PickFromBin(system),
			transitions = {
				'succeeded' : 'Stow In Bin',
				'no_grasp'  : 'No Grasp',
				'error'     : 'error',
			},
			remapping = {
				'current_master_pose' : 'current_master_pose',
				'job'                 : 'job',
				'grasp'               : 'grasp',
				'pick_plan'           : 'pick_plan',
				'retreat_trajectory'  : 'retreat_trajectory',
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

		smach.StateMachine.add('Success',      ReportJobResult(planner, Job.Success),                 transitions = {'succeeded': 'success'})
		smach.StateMachine.add('Occluded',     ReportJobResult(planner, Job.Occluded,    'occluder'), transitions = {'succeeded': 'failed'}, remapping = {'occluder': 'occluder'})
		smach.StateMachine.add('No Detection', ReportJobResult(planner, Job.NoDetection, 'occluder'), transitions = {'succeeded': 'failed'}, remapping = {'occluder': 'occluder'})
		smach.StateMachine.add('No Plan',      ReportJobResult(planner, Job.NoPlan),                  transitions = {'succeeded': 'failed'})
		smach.StateMachine.add('No Grasp',     ReportJobResult(planner, Job.NoGrasp),                 transitions = {'succeeded': 'failed'})

		return sm

