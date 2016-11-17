from apc16delft_msgs import msg

def motionGroupFromGraspStrategy(strategy):
	if strategy == msg.GraspCandidate.STRATEGY_VACUUM_V: return msg.MasterPoseDescriptor.GROUP_TOOL1
	if strategy == msg.GraspCandidate.STRATEGY_VACUUM_H: return msg.MasterPoseDescriptor.GROUP_TOOL0
	assert False, 'Invalid grasp strategy'

def home(motion_group = msg.MasterPoseDescriptor.GROUP_TOOL0):
	return msg.MasterPoseDescriptor(
		type         = msg.MasterPoseDescriptor.TYPE_HOME,
		group_name = motion_group,
		bin_index    = 0
	)

def bin(bin_index, motion_group = msg.MasterPoseDescriptor.GROUP_TOOL0):
	return msg.MasterPoseDescriptor(
		type         = msg.MasterPoseDescriptor.TYPE_BIN,
		group_name   = motion_group,
		bin_index    = bin_index
	)

def cam(bin_index, motion_group = msg.MasterPoseDescriptor.GROUP_TOOL0):
	return msg.MasterPoseDescriptor(
		type         = msg.MasterPoseDescriptor.TYPE_CAM,
		group_name   = motion_group,
		bin_index    = bin_index
	)

def tote(cell = 0, motion_group = msg.MasterPoseDescriptor.GROUP_TOOL0):
	return msg.MasterPoseDescriptor(
		type         = msg.MasterPoseDescriptor.TYPE_TOTE,
		group_name   = motion_group,
		bin_index    = cell
	)
