import threading
import rospy

class SimpleSubscriber:
	"""
	A topic subscriber that caches the last received message.
	"""
	def __init__(self, topic, type):
		"""
		Create a simple subscriber.
		"""
		self.__subsciber = rospy.Subscriber(topic, type, self.__callback)
		self.__message   = None
		self.__timestamp = rospy.Time(0)
		self.__event     = threading.Event()

	def __callback(self, message):
		"""
		Called when a new message is received.
		"""
		self.__message   = message
		self.__timestamp = rospy.Time.now()

	def isValid(self, max_age = None):
		"""
		Check if a valid message has been received.
		If max_age is not None, a message older than max_age is considered invalid.
		"""
		if self.__message is None: return False
		if max_age is None: return True
		return rospy.Time.now() - self.__timestamp < max_age

	def waitForValid(self, max_age, timeout):
		"""
		Wait for a valid message.
		If a valid message is already available, this function return immediately.
		"""
		if self.isValid(max_age): return True
		return self.__event.wait(timeout.secs)

	def message(self, max_age = None, wait = False, timeout = rospy.Duration(1)):
		"""
		Get the cached message, or None if it is not valid.
		If max_age is not None, a message older than max_age is considered invalid.
		If wait is True, this function will wait (up to a maximum time of timeout) for a valid message.
		"""
		if not wait:
			if not self.isValid(max_age): return None
			return self.__message
		else:
			if not self.waitForValid(max_age, timeout): return None
			return self.__message
