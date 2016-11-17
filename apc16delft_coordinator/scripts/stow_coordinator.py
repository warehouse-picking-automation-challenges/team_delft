#!/usr/bin/env python

import rospy
import roslib
import smach
import smach_ros
import tf

from apc16delft_msgs.msg import MasterPoseDescriptor
import std_srvs.srv

from lib.task_planner import StowTaskPlanner
from lib.system_interface import SystemInterface

from states.init        import makeInitStateMachine
from states.global_plan import PlanNextJob
from states.stow_job    import makeStowJobStateMachine
from states.wait_state  import WaitState

def main():
	rospy.init_node('coordinator')

	planner            = StowTaskPlanner(rospy.get_param("~task_file", "path_to_apc_stow_task.json"))
	transform_listener = tf.TransformListener(True, rospy.Duration(300))
	system             = SystemInterface()
	sm                 = smach.StateMachine(outcomes = ['done', 'error'])
	wait_state         = WaitState()

	sm.userdata.current_master_pose = MasterPoseDescriptor(group_name=MasterPoseDescriptor.GROUP_TOOL0, type=MasterPoseDescriptor.TYPE_HOME, bin_index=0)

	with sm:
		smach.StateMachine.add('Initialize', makeInitStateMachine(),
			transitions = {
				'success' : 'Wait For Start',
				'error'   : 'error',
			}
		)

		smach.StateMachine.add('Wait For Start', wait_state, transitions = {'done': 'Plan Next Job'})

		smach.StateMachine.add('Plan Next Job', PlanNextJob(planner),
			transitions = {
				'pick'  : 'error',
				'move'  : 'error',
				'stow'  : 'Stow',
				'empty' : 'done',
			},
			remapping = {
				'job'           : 'job',
				'bin_contents'  : 'bin_contents',
				'tote_contents' : 'tote_contents',
				'blacklist'     : 'blacklist',
			}
		)

		smach.StateMachine.add('Stow', makeStowJobStateMachine(planner, transform_listener, system),
			transitions = {
				'success'      : 'Plan Next Job',
				'failed'       : 'Plan Next Job',
				'error'        : 'error',
			},
			remapping = {
				'current_master_pose' : 'current_master_pose',
				'job'                 : 'job',
				'bin_contents'        : 'bin_contents',
				'tote_contents'       : 'tote_contents',
				'blacklist'           : 'blacklist',
			}
		)

		start_service = rospy.Service('~start', std_srvs.srv.Empty, lambda x: wait_state.trigger())

		outcome = sm.execute()

if __name__ == '__main__':
	main()

