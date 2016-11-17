#include "master_pose.hpp"

namespace apc16delft_msgs {
	bool operator< (apc16delft_msgs::MasterPoseDescriptor const & a, apc16delft_msgs::MasterPoseDescriptor const & b) {
		return apc16delft::masterPoseToString(a) < apc16delft::masterPoseToString(b);
	}
}

namespace apc16delft {

std::string masterPoseToString(int const & bin_index) {
		return "bin" + std::to_string(bin_index);
}

std::string masterPoseToString(apc16delft_msgs::MasterPoseDescriptor const & descriptor) {
	if (descriptor.type == apc16delft_msgs::MasterPoseDescriptor::TYPE_BIN || descriptor.type == apc16delft_msgs::MasterPoseDescriptor::TYPE_CAM) {
		return descriptor.type + std::to_string(descriptor.bin_index);
	} else if (descriptor.type == apc16delft_msgs::MasterPoseDescriptor::TYPE_TOTE && descriptor.bin_index > 0) {
		int index = 0;
		std::string orientation = "";

		index = descriptor.bin_index - 1;
		orientation = (descriptor.group_name == descriptor.GROUP_TOOL1 ? "v" : "h");
		int x = index / 3;
		int y = index % 3;
		return descriptor.type + std::to_string(x) + std::to_string(y) + orientation;
	} else {
		return descriptor.type;
	}
}

}
