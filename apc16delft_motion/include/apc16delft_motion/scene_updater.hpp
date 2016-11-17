#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <visualization_msgs/Marker.h>
#include <apc16delft_msgs/AddCollisionObject.h>
#include <apc16delft_msgs/Error.h>

namespace apc16delft {

// Update the planning scene with new collision models.
class SceneUpdater {
	public:
		SceneUpdater();

	private:
		ros::NodeHandle node_handle_;
		ros::Publisher collision_object_publisher_;
		ros::Publisher collision_object_visualizer_;
		ros::Rate publish_rate_{10};
		ros::Timer publish_timer_;
		ros::ServiceServer add_collision_object_;

		moveit_msgs::CollisionObject collision_object_;
		visualization_msgs::Marker collision_object_marker_;
		
		// Current planning scene
		moveit::planning_interface::PlanningSceneInterface *psi_;

	protected:
		bool addCollisionObject(apc16delft_msgs::AddCollisionObject::Request &req, apc16delft_msgs::AddCollisionObject::Response &res);
		void publishCollisionObject(ros::TimerEvent const &);
};

} // namespace

