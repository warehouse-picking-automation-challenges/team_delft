#pragma once
#include <apc16delft_msgs/MasterPoseDescriptor.h>
#include <string>
#include <stdexcept>

namespace apc16delft_msgs {
	bool operator< (apc16delft_msgs::MasterPoseDescriptor const & a, apc16delft_msgs::MasterPoseDescriptor const & b);
}

namespace apc16delft {
	/// Convert a master pose descriptor to string.
	std::string masterPoseToString(apc16delft_msgs::MasterPoseDescriptor const & descriptor);
	std::string masterPoseToString(int const & bin_index);

	inline apc16delft_msgs::MasterPoseDescriptor binMasterPose(int bin_index, std::string const & group_name = apc16delft_msgs::MasterPoseDescriptor::GROUP_TOOL0) {
		if(bin_index == -1) {
			throw std::logic_error("binMasterPose has not been implemented for bin index -1 (tote)");
		}
		apc16delft_msgs::MasterPoseDescriptor result;
		result.type = apc16delft_msgs::MasterPoseDescriptor::TYPE_BIN;
		result.bin_index = bin_index;
		result.group_name = group_name;
		return result;
	}

	inline apc16delft_msgs::MasterPoseDescriptor camMasterPose(int bin_index) {
		apc16delft_msgs::MasterPoseDescriptor result;
		result.type = apc16delft_msgs::MasterPoseDescriptor::TYPE_CAM;
		result.bin_index = bin_index;
		result.group_name = result.GROUP_TOOL0;
		return result;
	}

	inline apc16delft_msgs::MasterPoseDescriptor homeMasterPose() {
		apc16delft_msgs::MasterPoseDescriptor result;
		result.type = apc16delft_msgs::MasterPoseDescriptor::TYPE_HOME;
		result.bin_index = 0;
		result.group_name = result.GROUP_TOOL0;
		return result;
	}

	/// Get a master pose for a container.
	/**
	 * Index -1 is the tote, other indices are a bin index.
	 */
	inline apc16delft_msgs::MasterPoseDescriptor containerMasterPose(int container_index, std::string const & group_name = apc16delft_msgs::MasterPoseDescriptor::GROUP_TOOL0) {
		if (container_index == -1) {
			apc16delft_msgs::MasterPoseDescriptor result;
			result.group_name = group_name;
			result.type       = result.TYPE_TOTE;
			result.bin_index  = 0;
			return result;
		}
		return binMasterPose(container_index, group_name);
	}
}
