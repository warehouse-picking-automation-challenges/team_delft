#pragma once

#include <apc16delft_util/util.hpp>

namespace apc16delft {

struct LocalPoseParams {
	double facing_back_tolerance;
	double facing_up_tolerance;

	double top_bin_edge_clearing;
	double bottom_bin_edge_clearing;
	double side_bin_edge_clearing;
	double back_bin_edge_clearing;
	double side_approach_angle;
	double approach_angle;
	double pre_grasp_offset;
	double knn_offset;

	bool knn;

	LocalPoseParams(const LocalPoseParams & params) {
		facing_back_tolerance    = params.facing_back_tolerance;
		facing_up_tolerance      = params.facing_up_tolerance;
		top_bin_edge_clearing    = params.top_bin_edge_clearing;
		bottom_bin_edge_clearing = params.bottom_bin_edge_clearing;
		side_bin_edge_clearing   = params.side_bin_edge_clearing;
		back_bin_edge_clearing   = params.back_bin_edge_clearing;
		approach_angle           = params.approach_angle;
		side_approach_angle      = params.side_approach_angle;
		pre_grasp_offset         = params.pre_grasp_offset;
		knn                      = params.knn;
		knn_offset               = params.knn_offset;
	}

	LocalPoseParams() {
		top_bin_edge_clearing    = 0.03;
		bottom_bin_edge_clearing = 0.05;
		side_bin_edge_clearing   = 0.05;
		back_bin_edge_clearing   = 0.02;
		facing_back_tolerance    = 0.15;
		facing_up_tolerance      = 0.1;
		approach_angle           = deg2rad(15.0);
		side_approach_angle      = deg2rad(15.0);
		pre_grasp_offset         = 0.02;
		knn_offset               = 0.01;
		knn                      = false;
	}
};

} //namespace
