import rospy
import roslib
import smach
from lib.services import ServiceState
from apc16delft_msgs.srv import *
from apc16delft_msgs import *


def makeInitStateMachine():
	sm = smach.StateMachine(outcomes=['success','error'])
	
	with sm:
		# Configure camera node.
		smach.StateMachine.add(
				'Configure Gripper Camera',
				ServiceState('/gripper_camera/lifecycle/configure',
					RequestTransition,
					request=RequestTransitionRequest()),
				transitions = {
					'succeeded':'Activate Gripper Camera',
					'aborted':'Activate Gripper Camera',
					'preempted':'error'}
		)
		
		# Activate camera node.
		smach.StateMachine.add(
				'Activate Gripper Camera',
				ServiceState('/gripper_camera/lifecycle/activate',
					RequestTransition,
					request=RequestTransitionRequest()),
				transitions = {
					'succeeded':'Configure Tote Camera',
					'aborted':'Configure Tote Camera',
					'preempted':'error'}
		)
		
		# Configure camera node.
		smach.StateMachine.add(
				'Configure Tote Camera',
				ServiceState('/tote_camera/lifecycle/configure',
					RequestTransition,
					request=RequestTransitionRequest()),
				transitions = {
					'succeeded':'Activate Tote Camera',
					'aborted':'Activate Tote Camera',
					'preempted':'error'}
		)
		
		# Activate camera node.
		smach.StateMachine.add(
				'Activate Tote Camera',
				ServiceState('/tote_camera/lifecycle/activate',
					RequestTransition,
					request=RequestTransitionRequest()),
				transitions = {
					'succeeded':'Configure Grasp Synthesizer',
					'aborted':'Configure Grasp Synthesizer',
					'preempted':'error'}
			)
		
		# Configure grasp synthesizer.
		smach.StateMachine.add(
				'Configure Grasp Synthesizer',
				ServiceState('/grasp_synthesizer/lifecycle/configure',
					RequestTransition,
					request=RequestTransitionRequest()),
				transitions = {
					'succeeded':'Activate Grasp Synthesizer',
					'aborted':'Activate Grasp Synthesizer',
					'preempted':'error'}
		)
		
		# Activate grasp synthesizer.
		smach.StateMachine.add(
				'Activate Grasp Synthesizer',
				ServiceState('/grasp_synthesizer/lifecycle/activate',
					RequestTransition,
					request=RequestTransitionRequest()),
				transitions = {
					'succeeded':'Configure Gripper Driver',
					'aborted':'Configure Gripper Driver',
					'preempted':'error'}
		)
		
		# Configure gripper driver.
		smach.StateMachine.add(
				'Configure Gripper Driver',
				ServiceState('/gripper_driver/lifecycle/configure',
					RequestTransition,
					request=RequestTransitionRequest()),
				transitions = {
					'succeeded':'Activate Gripper Driver',
					'aborted':'Activate Gripper Driver',
					'preempted':'error'}
		)
		
		# Activate gripper driver.
		smach.StateMachine.add(
				'Activate Gripper Driver',
				ServiceState('/gripper_driver/lifecycle/activate',
					RequestTransition,
					request=RequestTransitionRequest()),
				transitions = {
					#'succeeded':'Configure Pose Estimator',
					#'aborted':  'Configure Pose Estimator',
					'succeeded':'Configure Manipulation Planner',
					'aborted':'Configure Manipulation Planner',
					'preempted':'error'}
		)
		
		# Configure pose estimator.
		smach.StateMachine.add(
				'Configure Pose Estimator',
				ServiceState('/pose_estimation/lifecycle/configure',
					RequestTransition,
					request=RequestTransitionRequest()),
				transitions = {
					'succeeded':'Activate Pose Estimator',
					'aborted':'Activate Pose Estimator',
					'preempted':'error'}
		)
		
		# Activate pose estimator.
		smach.StateMachine.add(
				'Activate Pose Estimator',
				ServiceState('/pose_estimation/lifecycle/activate',
					RequestTransition,
					request=RequestTransitionRequest()),
				transitions = {
					'succeeded':'Configure Manipulation Planner',
					'aborted':'Configure Manipulation Planner',
					'preempted':'error'}
		)
		
		# Configure manipulation planner.
		smach.StateMachine.add(
				'Configure Manipulation Planner',
				ServiceState('/manipulation_planner/lifecycle/configure',
					RequestTransition,
					request=RequestTransitionRequest()),
				transitions = {
					'succeeded':'Activate Manipulation Planner',
					'aborted':'Activate Manipulation Planner',
					'preempted':'error'}
		)
		
		# Activate manipulation planner.
		smach.StateMachine.add(
				'Activate Manipulation Planner',
				ServiceState('/manipulation_planner/lifecycle/activate',
					RequestTransition,
					request=RequestTransitionRequest()),
				transitions = {
					'succeeded':'Configure Placement Synthesizer',
					'aborted':  'Configure Placement Synthesizer',
					'preempted':'error'}
		)
		
		# Configure Placement Synthesizer.
		smach.StateMachine.add(
				'Configure Placement Synthesizer',
				ServiceState('/placement_synthesizer/lifecycle/configure',
					RequestTransition,
					request=RequestTransitionRequest()),
				transitions = {
					'succeeded':'Activate Placement Synthesizer',
					'aborted':  'Activate Placement Synthesizer',
					'preempted':'error'}
		)
		
		# Activate Placement Synthesizer.
		smach.StateMachine.add(
				'Activate Placement Synthesizer',
				ServiceState('/placement_synthesizer/lifecycle/activate',
					RequestTransition,
					request=RequestTransitionRequest()),
				transitions = {
					'succeeded':'Move to Home',
					'aborted':  'Move to Home',
					'preempted':'error'}
		)
		
		# Move robot to home position.
		smach.StateMachine.add(
				'Move to Home',
				ServiceState('/motion_executor/move_to_home',
					MoveToHome,
					request=MoveToHomeRequest()),
				transitions = {
					'succeeded':'success',
					'aborted':'error',
					'preempted':'error'}
		)
		
		return sm

