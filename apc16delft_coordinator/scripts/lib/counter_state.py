import smach

class Counter:
	def __init__(self, max_count, value = 0):
		self.max_count = max_count
		self.value = value

	def bump(self):
		self.value += 1
		if (self.value >= self.max_count):
			self.value = 0
			return True
		return False

	def reset(self):
		self.value = 0

class ResetCounterState(smach.State):
	def __init__(self, counter):
		self.counter = counter
		super(ResetCounterState, self).__init__(
			outcomes = ['succeeded'],
			output_keys = ['counter']
		)

	def execute(self, userdata):
		self.counter.reset()
		userdata.counter = self.counter.value
		return 'succeeded'

class BumpCounterState(smach.State):
	def __init__(self, counter):
		self.counter = counter
		super(BumpCounterState, self).__init__(
			outcomes = ['retry','overflow'],
			output_keys = ['counter']
		)

	def execute(self, userdata):
		overflowed = self.counter.bump()
		userdata.counter = self.counter.value
		return 'overflow' if overflowed else 'retry'

def makeCounterStates(max_count, initial_value = 0):
	counter = Counter(max_count, initial_value)
	return ResetCounterState(counter), BumpCounterState(counter)
