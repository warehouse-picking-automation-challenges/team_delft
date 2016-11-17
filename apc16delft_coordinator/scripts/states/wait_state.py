import threading
import smach
import rospy

class WaitState(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['done'])
		self.__event = threading.Event()

	def trigger(self):
		self.__event.set()

	def execute(self, userdate):
		self.__event.clear()
		rospy.loginfo("Waiting for continuation signal.")
		self.__event.wait()
		rospy.loginfo("Continuing")
		return 'done'
