#include "scene_updater.hpp"
#include <string>
#include <dr_param/param.hpp>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/Mesh.h>
namespace apc16delft {

SceneUpdater::SceneUpdater () :
	node_handle_("~")
{
	collision_object_publisher_  = node_handle_.advertise<moveit_msgs::CollisionObject>("collision_object", 1, true);
	collision_object_visualizer_ = node_handle_.advertise<visualization_msgs::Marker>("collision_object_marker", 1, true);
	publish_timer_               = node_handle_.createTimer(publish_rate_, &SceneUpdater::publishCollisionObject, this);
	psi_                         = new moveit::planning_interface::PlanningSceneInterface();
	std::string default_stl_path = "package://apc16delft_description/meshes/bin0.stl";
	add_collision_object_        = node_handle_.advertiseService("add_collision_object", &SceneUpdater::addCollisionObject, this);
}

bool SceneUpdater::addCollisionObject(apc16delft_msgs::AddCollisionObject::Request &req, apc16delft_msgs::AddCollisionObject::Response &res) {

	std::string path_to_stl = "package://apc16delft_description/meshes/bin" + std::to_string(req.bin_index) + ".stl";
	ROS_INFO_STREAM("Loading STL file from: " << path_to_stl << "...");
	shapes::Mesh* mesh = shapes::createMeshFromResource(path_to_stl);
	ROS_INFO_STREAM("Successfully loaded mesh.");

	shape_msgs::Mesh collision_object_mesh;
	shapes::ShapeMsg collision_object_mesh_msg;
	shapes::constructMsgFromShape(mesh, collision_object_mesh_msg);
	collision_object_mesh = boost::get<shape_msgs::Mesh>(collision_object_mesh_msg);
	collision_object_.meshes.resize(1);
	collision_object_.mesh_poses.resize(1);
	collision_object_.meshes[0] = collision_object_mesh;
	collision_object_.header.frame_id = req.pose_collision_object.header.frame_id;
	collision_object_.id = "shelf";

	collision_object_.mesh_poses[0].position.x = req.pose_collision_object.pose.position.x;
	collision_object_.mesh_poses[0].position.y = req.pose_collision_object.pose.position.y;
	collision_object_.mesh_poses[0].position.z = req.pose_collision_object.pose.position.z;
	collision_object_.mesh_poses[0].orientation.x = req.pose_collision_object.pose.orientation.x;
	collision_object_.mesh_poses[0].orientation.y = req.pose_collision_object.pose.orientation.y;
	collision_object_.mesh_poses[0].orientation.z = req.pose_collision_object.pose.orientation.z;
	collision_object_.mesh_poses[0].orientation.w = req.pose_collision_object.pose.orientation.w;

	collision_object_.meshes.push_back(collision_object_mesh);
	collision_object_.mesh_poses.push_back(collision_object_.mesh_poses[0]);
	collision_object_.operation = collision_object_.ADD;

	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.push_back(collision_object_);

	psi_->addCollisionObjects(collision_objects);

	//Visualize the collision object in rviz.
	collision_object_marker_.header.frame_id = req.pose_collision_object.header.frame_id;
	collision_object_marker_.ns = "apc16delft";
	collision_object_marker_.id = 0;
	collision_object_marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
	collision_object_marker_.action = visualization_msgs::Marker::ADD;
	collision_object_marker_.pose = req.pose_collision_object.pose;
	collision_object_marker_.scale.x = 1;
	collision_object_marker_.scale.y = 1;
	collision_object_marker_.scale.z = 1;
	collision_object_marker_.color.a = 1.0;
	collision_object_marker_.color.r = 0.54*0.75; //These magic numbers are for getting the bin color!!
	collision_object_marker_.color.g = 0.270*0.75;
	collision_object_marker_.color.b = 0.06*0.75;
	collision_object_marker_.mesh_resource = path_to_stl;

	res.error.code = apc16delft_msgs::Error::SUCCESS;
	res.error.message = "Added collision object to planning scene and published it to rviz.";

	return true;
}

void SceneUpdater::publishCollisionObject(ros::TimerEvent const &) {
	collision_object_.header.stamp = ros::Time::now();
	collision_object_marker_.header.stamp = ros::Time::now();
	collision_object_publisher_.publish(collision_object_);
	collision_object_visualizer_.publish(collision_object_marker_);
}

} // namespace

int main(int argc, char ** argv) {
	ros::init(argc, argv, "scene_updater");
	apc16delft::SceneUpdater scene_updater;
	ros::spin();
	
	return 0;
}

