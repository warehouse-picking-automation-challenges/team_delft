import random
import rospy
import roslib
import smach

from lib.bins import binIndexToString, binIndexFromString
from lib.task_planner import PickTaskPlanner, StowTaskPlanner
from lib import job as Job
from apc16delft_msgs.objects import objectTypeToString

class PlanNextJob(smach.State):
	def __init__(self, planner):
		smach.State.__init__(
			self,
			outcomes    = ['pick', 'move', 'stow', 'empty'],
			output_keys = ['job', 'bin_contents', 'tote_contents', 'blacklist']
		)

		self.planner = planner

	def execute(self, userdata):
		userdata.job          = None
		userdata.bin_contents = None
		userdata.blacklist    = None

		if isinstance(self.planner, PickTaskPlanner):
			rospy.loginfo('Planning new task, main queue size: {}, failed queue size: {}'.format(len(self.planner.pending_orders), len(self.planner.failed_orders)))
		elif isinstance(self.planner, StowTaskPlanner):
			userdata.blacklist = self.planner.blacklist
			rospy.loginfo('Planning new task, tote contents: {}, blacklist: {}'.format(len(self.planner.tote_contents), len(self.planner.blacklist)))

		job = self.planner.nextJob()

		if job is None:
			rospy.loginfo('No job returned by task planner.')
			return 'empty'

		userdata.job           = job
		userdata.bin_contents  = self.planner.bin_contents
		userdata.tote_contents = self.planner.tote_contents

		if isinstance(job, Job.Pick):
			rospy.loginfo('Selected PICK JOB for item {} in bin {}.'.format(objectTypeToString[job.object_type], job.source_bin));
			return 'pick'
		if isinstance(job, Job.Stow):
			rospy.loginfo('Selected STOW JOB to fill bin {}.'.format(job.target_bin));
			return 'stow'
		elif isinstance(job, Job.Move):
			rospy.loginfo('Selected MOVE JOB for item {} in bin {}, move to {}.'.format(objectTypeToString[job.object_type], job.source_bin, job.target_bin));
			return 'move'
		else:
			raise Exception('Unsupported job type returned by task planner')

class ReportJobResult(smach.State):
	def __init__(self, planner, result_class, *args, **kwargs):
		smach.State.__init__(
			self,
			outcomes   = ['succeeded'],
			input_keys = list(args) + list(kwargs.values())
		)
		self.planner       = planner
		self.result_class  = result_class
		self.args          = args
		self.kwargs        = kwargs

	def execute(self, userdata):
		args = [getattr(userdata, x) for x in self.args]

		kwargs = {}
		for key in self.kwargs:
			kwargs[key] = getattr(userdata, self.kwargs[key])

		self.planner.taskDone(self.result_class(*args, **kwargs))
		return 'succeeded'
