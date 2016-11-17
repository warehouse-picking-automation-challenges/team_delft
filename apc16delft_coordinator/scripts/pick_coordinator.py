#!/usr/bin/env python

import rospy
import roslib
import smach
import smach_ros
import tf

from apc16delft_msgs.msg import MasterPoseDescriptor
import std_srvs.srv

from lib.task_planner     import PickTaskPlanner
from lib.system_interface import SystemInterface

from states.init        import makeInitStateMachine
from states.global_plan import PlanNextJob
from states.pick_job    import makePickJobStateMachine
from states.move_job    import makeMoveJobStateMachine
from states.wait_state  import WaitState

def main():
	rospy.init_node('coordinator')

	planner            = PickTaskPlanner(rospy.get_param("~task_file", "path_to_apc_pick_task.json"))
	transform_listener = tf.TransformListener(True, rospy.Duration(300))
	system             = SystemInterface()
	sm                 = smach.StateMachine(outcomes = ['done','error'])
	wait_state         = WaitState()

	sm.userdata.current_master_pose = MasterPoseDescriptor(group_name=MasterPoseDescriptor.GROUP_TOOL0, type=MasterPoseDescriptor.TYPE_HOME, bin_index=0)

	with sm:
		smach.StateMachine.add('Initialize', makeInitStateMachine(),
			transitions = {
				'success' : 'Wait For Start',
				'error'   : 'error'
			}
		)

		smach.StateMachine.add('Wait For Start', wait_state, transitions = {'done': 'Plan Next Job'})

		smach.StateMachine.add('Plan Next Job', PlanNextJob(planner),
			transitions = {
				'pick'  : 'Pick',
				'move'  : 'Move',
				'stow'  : 'error',
				'empty' : 'done',
			},
			remapping = {
				'job'          : 'job',
				'bin_contents' : 'bin_contents'
			}
		)

		smach.StateMachine.add('Pick', makePickJobStateMachine(transform_listener, planner, system),
			transitions = {
				'success'      : 'Plan Next Job',
				'failed'       : 'Plan Next Job',
				'error'        : 'error'
			},
			remapping = {
				'job'                 : 'job',
				'bin_contents'        : 'bin_contents',
				'current_master_pose' : 'current_master_pose',
			}
		)

		smach.StateMachine.add('Move', makeMoveJobStateMachine(transform_listener, planner, system),
			transitions = {
				'success'      : 'Plan Next Job',
				'failed'       : 'Plan Next Job',
				'error'        : 'error'
			},
			remapping = {
				'job'                 : 'job',
				'bin_contents'        : 'bin_contents',
				'current_master_pose' : 'current_master_pose',
			}
		)

		start_service = rospy.Service('~start', std_srvs.srv.Empty, lambda x: wait_state.trigger())
		outcome = sm.execute()

if __name__ == '__main__':
	main()

