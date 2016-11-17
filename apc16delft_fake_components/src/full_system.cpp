#include <ros/ros.h>

#include <apc16delft_msgs/RequestTransition.h>
#include <apc16delft_msgs/MoveToHome.h>
#include <apc16delft_msgs/GetCameraData.h>
#include <apc16delft_msgs/EstimateBinPose.h>
#include <apc16delft_msgs/DetectObjects.h>
#include <apc16delft_msgs/FilterObject.h>
#include <apc16delft_msgs/CropPointCloud.h>
#include <apc16delft_msgs/PublishStaticPointCloud.h>
#include <apc16delft_msgs/EstimateObjectPose.h>
#include <apc16delft_msgs/DetectOcclusion.h>
#include <std_srvs/Empty.h>

#include <apc16delft_msgs/SynthesizeGrasp.h>
#include <apc16delft_msgs/SynthesizeDeformable.h>
#include <apc16delft_msgs/PruneGraspCandidates.h>
#include <apc16delft_msgs/PlanManipulation.h>
#include <apc16delft_msgs/PlanStowMotion.h>
#include <apc16delft_msgs/SynthesizeStow.h>

#include <apc16delft_msgs/ExecuteStitchedMotion.h>
#include <apc16delft_msgs/GetCoarseMotion.h>
#include <apc16delft_msgs/ExecuteCoarseMotion.h>
#include <apc16delft_msgs/PlanCameraMotion.h>
#include <apc16delft_msgs/ExecuteFineMotion.h>
#include <apc16delft_msgs/SetGripperState.h>
#include <motoman_msgs/ReadSingleIO.h>
#include <motoman_msgs/WriteSingleIO.h>

#include <string>

namespace apc16delft {

template<typename T>
class FakeServiceProvider {
	using Request  = typename T::Request;
	using Response = typename T::Response;

	std::string name;
	ros::ServiceServer server;
	Response response;


public:
	FakeServiceProvider(ros::NodeHandle & node, std::string const & name, Response response = Response()) : name(name), response(response) {
		server = node.advertiseService(name, &FakeServiceProvider::onService, this);
	}

private:
	bool onService(Request &, Response & response) {
		response = this->response;
		ROS_INFO_STREAM("Service `" << name << "' called.");
		return true;
	}
};

}

bool onGetGripperCameraData(apc16delft_msgs::GetCameraData::Request &, apc16delft_msgs::GetCameraData::Response & response) {
	ROS_INFO_STREAM("Service `/gripper_camera/get_data' called.");
	response.point_cloud.header.frame_id = "robot_tool0";
	response.point_cloud.header.stamp    = ros::Time::now();
	return true;
}

bool onGetToteCameraData(apc16delft_msgs::GetCameraData::Request &, apc16delft_msgs::GetCameraData::Response & response) {
	ROS_INFO_STREAM("Service `/tote_camera/get_data' called.");
	response.point_cloud.header.frame_id = "robot_tool0";
	response.point_cloud.header.stamp    = ros::Time::now();
	return true;
}

bool onEstimateBinPose(apc16delft_msgs::EstimateBinPose::Request & request, apc16delft_msgs::EstimateBinPose::Response & response) {
	ROS_INFO_STREAM("Service `/bin_pose_estimation/estimate_bin_pose' called.");
	response.pose.header              = request.scene.header;
	return true;
}

bool onDetectObjects(apc16delft_msgs::DetectObjects::Request & request, apc16delft_msgs::DetectObjects::Response & response) {
	ROS_INFO_STREAM("Service `/object_detection_server/detect_objects' called.");
	for (int type : request.objects) {
		apc16delft_msgs::ObjectDetection detection;
		detection.point_cloud.header = request.point_cloud.header;
		detection.object.type        = type;
		response.objects.push_back(detection);
	}
	return true;
}

bool onEstimateObjectPose(apc16delft_msgs::EstimateObjectPose::Request & request, apc16delft_msgs::EstimateObjectPose::Response & response) {
	ROS_INFO_STREAM("Service `/pose_estimation/estimate_object_pose' called.");
	std_msgs::Header header           = request.object.point_cloud.header;
	response.pose.header              = header;
	response.transformed_model.header = header;
	return true;
}

bool onSynthesizeGrasps(apc16delft_msgs::SynthesizeGrasp::Request & request, apc16delft_msgs::SynthesizeGrasp::Response & response) {
	ROS_INFO_STREAM("Service `/grasp_synthesizer/synthesize_grasp' called.");
	apc16delft_msgs::GraspCandidate candidate;
	candidate.stamped_pose.header = request.point_cloud.header;
	candidate.strategy            = candidate.STRATEGY_VACUUM_H;
	candidate.score               = 1;
	response.candidates.push_back(candidate);
	return true;
}

bool onSynthesizeDeformbale(apc16delft_msgs::SynthesizeDeformable::Request & request, apc16delft_msgs::SynthesizeDeformable::Response & response) {
	ROS_INFO_STREAM("Service `/grasp_synthesizer/synthesize_deformable' called.");
	apc16delft_msgs::GraspCandidate candidate;
	candidate.stamped_pose.header = request.point_cloud.header;
	candidate.strategy            = candidate.STRATEGY_VACUUM_H;
	candidate.score               = 1;
	response.candidates.push_back(candidate);
	return true;
}

bool onSynthesizeStow(apc16delft_msgs::SynthesizeStow::Request &, apc16delft_msgs::SynthesizeStow::Response & response) {
	ROS_INFO_STREAM("Service `/stowing_logic/synthesize_stow' called.");
	apc16delft_msgs::PlacementCandidate candidate;
	candidate.score = 1;
	response.candidates.push_back(candidate);
	return true;
}

bool onReadSingleIo(motoman_msgs::ReadSingleIO::Request &, motoman_msgs::ReadSingleIO::Response & response) {
	ROS_INFO_STREAM("Service `/read_single_io' called.");
	response.value = true;
	return true;
}

bool onCropBox(apc16delft_msgs::CropPointCloud::Request & request, apc16delft_msgs::CropPointCloud::Response & response) {
	for (auto const & box : request.boxes) {
		response.cropped_point_clouds.push_back({});
		response.centroids.push_back({});
		response.cropped_point_clouds.back().header = request.point_cloud.header;
	}

	return true;
}

int main(int argc, char * * argv) {
	ros::init(argc, argv, ROS_PACKAGE_NAME "_full_system");
	ros::NodeHandle node("~");

	using apc16delft::FakeServiceProvider;

	FakeServiceProvider<apc16delft_msgs::RequestTransition> configure_gripper_camera        ( node, "/gripper_camera/lifecycle/configure"        ) ;
	FakeServiceProvider<apc16delft_msgs::RequestTransition> activate_gripper_camera         ( node, "/gripper_camera/lifecycle/activate"         ) ;
	FakeServiceProvider<apc16delft_msgs::RequestTransition> configure_tote_camera           ( node, "/tote_camera/lifecycle/configure"           ) ;
	FakeServiceProvider<apc16delft_msgs::RequestTransition> activate_tote_camera            ( node, "/tote_camera/lifecycle/activate"            ) ;
	FakeServiceProvider<apc16delft_msgs::RequestTransition> configure_grasp_synthesizer     ( node, "/grasp_synthesizer/lifecycle/configure"     ) ;
	FakeServiceProvider<apc16delft_msgs::RequestTransition> activate_grasp_synthesizer      ( node, "/grasp_synthesizer/lifecycle/activate"      ) ;
	FakeServiceProvider<apc16delft_msgs::RequestTransition> configure_gripper_driver        ( node, "/gripper_driver/lifecycle/configure"        ) ;
	FakeServiceProvider<apc16delft_msgs::RequestTransition> activate_gripper_driver         ( node, "/gripper_driver/lifecycle/activate"         ) ;
	FakeServiceProvider<apc16delft_msgs::RequestTransition> configure_pose_estimation       ( node, "/pose_estimation/lifecycle/configure"       ) ;
	FakeServiceProvider<apc16delft_msgs::RequestTransition> activate_pose_estimation        ( node, "/pose_estimation/lifecycle/activate"        ) ;
	FakeServiceProvider<apc16delft_msgs::RequestTransition> configure_manipulation_planner  ( node, "/manipulation_planner/lifecycle/configure"  ) ;
	FakeServiceProvider<apc16delft_msgs::RequestTransition> activate_manipulation_planner   ( node, "/manipulation_planner/lifecycle/activate"   ) ;
	FakeServiceProvider<apc16delft_msgs::RequestTransition> configure_placement_synthesizer ( node, "/placement_synthesizer/lifecycle/configure" ) ;
	FakeServiceProvider<apc16delft_msgs::RequestTransition> activate_placement_synthesizer  ( node, "/placement_synthesizer/lifecycle/activate"  ) ;



	FakeServiceProvider<apc16delft_msgs::FilterObject           > filter_objects           (node, "/point_cloud_crop/filter_object"                      );
	FakeServiceProvider<apc16delft_msgs::PublishStaticPointCloud> publish_point_cloud      (node, "/static_pointcloud_publisher/publish_point_cloud"     );
	FakeServiceProvider<apc16delft_msgs::DetectOcclusion        > detect_occlusions        (node, "/occlusion_detection/detect_occlusion"                );
	FakeServiceProvider<std_srvs::Empty                         > clear_static_point_cloud (node, "/static_pointcloud_publisher/clear_static_point_cloud");
	FakeServiceProvider<std_srvs::Empty                         > clear_octomap            (node, "/clear_octomap"                                       );

	FakeServiceProvider<apc16delft_msgs::PruneGraspCandidates   > update_candidates        (node, "/manipulation_planner/update_reachability"    );
	FakeServiceProvider<apc16delft_msgs::PlanManipulation       > plan_manipulation        (node, "/manipulation_planner/plan_manipulation"      );
	FakeServiceProvider<apc16delft_msgs::PlanManipulation       > plan_tote_manipulation   (node, "/manipulation_planner/plan_tote_manipulation" );
	FakeServiceProvider<apc16delft_msgs::PlanStowMotion         > plan_stow_motion         (node, "/manipulation_planner/plan_stow_motion"       );


	FakeServiceProvider<apc16delft_msgs::MoveToHome            > motion_move_to_home     (node, "/motion_executor/move_to_home"             );
	FakeServiceProvider<apc16delft_msgs::ExecuteStitchedMotion > execute_stitched_motion (node, "/motion_executor/execute_stitched_motion"  );
	FakeServiceProvider<apc16delft_msgs::GetCoarseMotion       > get_coarse_motion       (node, "/motion_executor/get_coarse_motion"        );
	FakeServiceProvider<apc16delft_msgs::ExecuteCoarseMotion   > execute_coarse_motion   (node, "/motion_executor/execute_coarse_motion"    );
	FakeServiceProvider<apc16delft_msgs::PlanCameraMotion      > plan_camera_motion      (node, "/manipulation_planner/plan_camera_motion"  );
	FakeServiceProvider<apc16delft_msgs::ExecuteFineMotion     > execute_fine_motion     (node, "/motion_executor/execute_fine_motion"      );
	FakeServiceProvider<apc16delft_msgs::SetGripperState       > set_gripper_state       (node, "/gripper_driver/set_state"                 );
	FakeServiceProvider<motoman_msgs::WriteSingleIO            > write_single_io         (node, "/write_single_io"                          );

	ros::ServiceServer get_gripper_camera_data     = node.advertiseService("/gripper_camera/get_data",                 &onGetGripperCameraData);
	ros::ServiceServer get_tote_camera_data        = node.advertiseService("/tote_camera/get_data",                    &onGetToteCameraData);
	ros::ServiceServer estimate_bin_pose           = node.advertiseService("/bin_pose_estimation/estimate_bin_pose",   &onEstimateBinPose);
	ros::ServiceServer detect_objects              = node.advertiseService("/object_detection_server/detect_objects",  &onDetectObjects);
	ros::ServiceServer detect_bin_objects          = node.advertiseService("/tote_detection/detect_objects",           &onDetectObjects);
	ros::ServiceServer detect_tote_objects         = node.advertiseService("/bin_detection/detect_objects",            &onDetectObjects);
	ros::ServiceServer estimate_object_pose        = node.advertiseService("/pose_estimation/estimate_object_pose",    &onEstimateObjectPose);
	ros::ServiceServer synthesize_grasp            = node.advertiseService("/grasp_synthesizer/synthesize_grasp",      &onSynthesizeGrasps);
	ros::ServiceServer synthesize_deformable_grasp = node.advertiseService("/grasp_synthesizer/synthesize_deformable", &onSynthesizeDeformbale);
	ros::ServiceServer synthesize_stow             = node.advertiseService("/stowing_logic/synthesize_stow",           &onSynthesizeStow);
	ros::ServiceServer read_single_io              = node.advertiseService("/read_single_io",                          &onReadSingleIo);
	ros::ServiceServer crop_point_cloud            = node.advertiseService("/point_cloud_crop/crop_point_cloud",       &onCropBox);

	ros::spin();
}
