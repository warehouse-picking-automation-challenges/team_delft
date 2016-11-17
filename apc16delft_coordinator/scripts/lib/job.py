class PickOrder():
	def __init__(self, object_type, source_bin):
		self.object_type = object_type
		self.source_bin  = source_bin
		self.history     = []

class Pick():
	def __init__(self, object_type, source_bin, target_tote_cell):
		self.object_type      = object_type
		self.source_bin       = source_bin
		self.target_tote_cell = target_tote_cell

class Stow():
	def __init__(self, object_type, target_bin):
		self.object_type = object_type
		self.target_bin  = target_bin

class Move():
	def __init__(self, object_type, source_bin, target_bin):
		self.object_type = object_type
		self.source_bin  = source_bin
		self.target_bin  = target_bin

class Success():
	pass
class StowSuccess():
	def __init__(self, object_type):
		self.object_type = object_type
class UnspecifiedFailure():
	pass
class NoDetection():
	def __init__(self, occluder = None):
		self.occluder = occluder
class NoObjectPose():
	def __init__(self, object_type = None):
		self.object_type = object_type
class Occluded():
	def __init__(self, occluder):
		self.occluder = occluder
class NoPlan():
	def __init__(self, object_type = None, grasp_occluders = None):
		self.object_type     = object_type
		self.grasp_occluders = grasp_occluders
class NoGrasp():
	def __init__(self, object_type = None, grasp_occluders = None):
		self.object_type     = object_type
		self.grasp_occluders = grasp_occluders
class ProbablyDropped():
	pass
class DependencyFailed():
	def __init__(self, nested_result):
		self.nested_result = nested_result
