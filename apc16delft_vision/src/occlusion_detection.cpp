#include <apc16delft_util/util.hpp>
#include <apc16delft_msgs/DetectOcclusion.h>
#include <apc16delft_msgs/ObjectDetection.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <dr_param/param.hpp>

#include <apc16delft_msgs/objects.hpp>

#include <boost/optional.hpp>

namespace apc16delft {

class OcclusionDetectionNode {

using Point          = pcl::PointXYZ;
using PointCloud     = pcl::PointCloud<Point>;

public:
	OcclusionDetectionNode() : node_handle_("~") {
		occlusion_detection_server_ = node_handle_.advertiseService("detect_occlusion", &OcclusionDetectionNode::detectOcclusion, this);
		
		bin_size_ = dr::getParam<double>(node_handle_, "discretization_bin_size", bin_size_);
		publish_clouds_ = dr::getParam<bool>(node_handle_, "publish_clouds", publish_clouds_);
		if(publish_clouds_) {
			occlusion_map_publisher_    = node_handle_.advertise<sensor_msgs::PointCloud2>("occlusion_map", 1, true);
			occluding_points_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>("occluding_points", 1, true);
		}
	}

	~OcclusionDetectionNode() {
		occlusion_detection_server_.shutdown();
		if(publish_clouds_) {
			occlusion_map_publisher_.shutdown();
			occluding_points_publisher_.shutdown();
		}
	}

	bool detectOcclusion(apc16delft_msgs::DetectOcclusion::Request & req, apc16delft_msgs::DetectOcclusion::Response & res) {
		ROS_INFO_STREAM("Received occlusion detection request.");
		// These are for debugging purposes and will not be published unless necessary.
		PointCloud::Ptr occlusion_map{new PointCloud};
		PointCloud::Ptr occluding_points{new PointCloud};

		// Determine the transformation to align the occlusion_direction with the z-axis.
		Eigen::Vector3d occlusion_direction = dr::toEigen(req.direction);
		Eigen::Affine3d transform = Eigen::Quaterniond::FromTwoVectors(occlusion_direction, dr::axes::z()) * Eigen::Translation3d(0,0,0);

		// Create a raster
		Point target_min;
		Point target_max;
		size_t bins_x, bins_y;
		
		PointCloud::Ptr target_cloud{new PointCloud};
		pcl::fromROSMsg(req.target_model, *target_cloud);
		std::vector<int> nan_indices;
		pcl::removeNaNFromPointCloud(*target_cloud, *target_cloud, nan_indices);
		PointCloud::Ptr rotated_target_cloud{new PointCloud};
		pcl::transformPointCloud(*target_cloud, *rotated_target_cloud, transform);

		Point target_centroid;
		pcl::computeCentroid(*rotated_target_cloud, target_centroid);

		pcl::getMinMax3D(*rotated_target_cloud, target_min, target_max);

		// Widen the area to make bins fit slightly around the object
		target_min.x -= bin_size_ / 2.0;
		target_min.y -= bin_size_ / 2.0;
		target_max.x += bin_size_ / 2.0;
		target_max.y += bin_size_ / 2.0;
		
		// Figure out the amount of bins
		double dx = target_max.x - target_min.x;
		double dy = target_max.y - target_min.y;
		double mid_x = target_min.x + dx / 2.0;
		double mid_y = target_min.y + dy / 2.0;

		bins_x = std::ceil(dx / bin_size_);
		bins_y = std::ceil(dy / bin_size_);

		// Adjust min x and y to perfectly fit the amount of bins
		target_min.x = mid_x - dx / 2.0;
		target_min.y = mid_y - dy / 2.0;
		target_max.x = mid_x + dx / 2.0;
		target_max.y = mid_y + dy / 2.0;


		std::vector<boost::optional<Point>> nearest_target_points;
		nearest_target_points.resize(bins_x * bins_y);
		unsigned int num_nearest_points = 0;

		for(Point & p : *rotated_target_cloud) {
			boost::optional<size_t> index = discretize(p, target_min, target_max, bins_x, bins_y);
			if(!index) {
				res.error.code = apc16delft_msgs::DetectOcclusion::Response::E_INPUT_SANITY;
				res.error.message = "Could not make sense of target point cloud";
				return true;
			}
			if(!nearest_target_points[*index] ||
			   nearest_target_points[*index]->z > p.z) {
				if(!nearest_target_points[*index]) {
					++num_nearest_points;
				}
				nearest_target_points[*index] = p;
			}
		}
		ROS_INFO_STREAM("Amount of grid points:                  " << nearest_target_points.size());
		ROS_INFO_STREAM("Amount of grid points filled by target: " << num_nearest_points);

		std::vector<bool> target_occluded;
		target_occluded.resize(bins_x * bins_y, false);

		Point non_target_centroid;
		// Now process the non-target objects.
		int i = 0;
		for(const apc16delft_msgs::ObjectDetection & obj : req.non_target_objects) {
			ROS_INFO_STREAM("Processing non-target object number: " << i++);
			ROS_INFO_STREAM("Object type is: " << apc16delft_msgs::objectTypeToString(obj.object.type));
			// Steps:
			// Loop over point cloud
			//   Get point discretization
			//   Compare z-value to relevant target z-value
			// Count occlusions
			// Make a decision.

			PointCloud::Ptr obj_cloud{new PointCloud};
			pcl::fromROSMsg(obj.point_cloud, *obj_cloud);
			pcl::removeNaNFromPointCloud(*obj_cloud, *obj_cloud, nan_indices);
			pcl::transformPointCloud(*obj_cloud, *obj_cloud, transform);

			std::vector<bool> occluding;
			occluding.resize(bins_x * bins_y, false);
			unsigned int num_occluding_points = 0;

			// Loop over point cloud
			int point_num = 0;
			for(const Point & p : *obj_cloud) {
			//   Get point discretization
				boost::optional<size_t> index = discretize(p, target_min, target_max, bins_x, bins_y);
			//   Compare z-value to relevant target z-value
				if(!index || occluding[*index] || !nearest_target_points[*index] || p.z >= nearest_target_points[*index]->z){
					continue;
				}
				else {
			// Count occlusions
					occluding[*index] = true;
					target_occluded[*index] = true;
					++num_occluding_points;

					// For debug purposes, add this point to the list of occluding points.
					occluding_points->push_back(p);
				}
			}

			// Make a decision.
			ROS_INFO_STREAM("This object has " << num_occluding_points << " points that occlude the target object.");
			if(num_occluding_points > 0) {
				apc16delft_msgs::Occlusion occlusion_msg;
				occlusion_msg.occluder = obj.object;
				occlusion_msg.occlusion_percentage = double(num_occluding_points)/double(num_nearest_points);

				ROS_INFO_STREAM("This object occludes the target object by " << occlusion_msg.occlusion_percentage);
				
				ROS_INFO_STREAM("Starting naivification.");
				pcl::computeCentroid(*obj_cloud, non_target_centroid);
				if(non_target_centroid.z <= target_centroid.z) {
					ROS_INFO_STREAM("Object centroid closer than target object centroid, so naively saying this object occludes the target.");
					res.occlusions.push_back(occlusion_msg);
				} else {
					ROS_INFO_STREAM("Object centroid further than target object centroid, so naively saying this object does not occlude the target.");
				}
			}
		}

		res.occlusion_percentage = 0.0;
		for(bool count_this : target_occluded) {
			if(count_this) {
				res.occlusion_percentage += 1.0;
			}
		}
		res.occlusion_percentage /= double(num_nearest_points);

		ROS_INFO_STREAM("The target object is occluded (non-naive estimation) by " << res.occlusion_percentage);

		ROS_INFO_STREAM("There are " << res.occlusions.size() << " occluders.");
		if(res.occlusions.size() > 0) {
			ROS_INFO_STREAM("Naively setting occlusion to the sum of occlusions. This may be higher than the real amount of occlusion.");
			res.occlusion_percentage = 0.0;
			for(const apc16delft_msgs::Occlusion & occl : res.occlusions) {
				res.occlusion_percentage += occl.occlusion_percentage;
			}
			ROS_INFO_STREAM("Occlusion is set to be " << res.occlusion_percentage << " as a naive estimation.");
		} else {
			ROS_INFO_STREAM("Naively setting occlusion to 0.");
			res.occlusion_percentage = 0;
		}

		if(publish_clouds_) {
			ROS_INFO_STREAM("Publishing debug clouds.");
			// Publish debug data.
			for(size_t i = 0; i < nearest_target_points.size(); ++i) {
				if(nearest_target_points[i]) {
					Point p = reverseBinLookup(i, target_min, bins_x, bins_y);
					//occlusion_map->push_back(p);
					occlusion_map->push_back(*(nearest_target_points[i]));
				}
			}
			pcl::transformPointCloud(*occlusion_map, *occlusion_map, transform.inverse());
			occlusion_map->header = target_cloud->header;
			sensor_msgs::PointCloud2 occlusion_map_msg;
			pcl::toROSMsg(*occlusion_map, occlusion_map_msg);
			occlusion_map_publisher_.publish(occlusion_map_msg);

			occluding_points->header = target_cloud->header;
			pcl::transformPointCloud(*occluding_points, *occluding_points, transform.inverse());
			sensor_msgs::PointCloud2 occluding_points_msg;
			pcl::toROSMsg(*occluding_points, occluding_points_msg);
			occluding_points_publisher_.publish(occluding_points_msg);

		}

		return true;
	}

private:

	boost::optional<size_t> discretize(const Point & p, const Point & min, const Point & max, size_t bins_x, size_t bins_y) {
		if(p.x < min.x || p.x > max.x ||
		   p.y < min.y || p.y > max.y) {
			return boost::none;
		}
		size_t idx_x = std::floor( (p.x - min.x) / bin_size_);
		size_t idx_y = std::floor( (p.y - min.y) / bin_size_);

		size_t idx = idx_x + bins_x * idx_y;

//		ROS_INFO_STREAM("min     : " << min);
//		ROS_INFO_STREAM("p       : " << p);
//		ROS_INFO_STREAM("bins_x  : " << bins_x);
//		ROS_INFO_STREAM("bins_y  : " << bins_y);
//		ROS_INFO_STREAM("bin_size: " << bin_size_);
//		ROS_INFO_STREAM("idx_x   : " << idx_x);
//		ROS_INFO_STREAM("idx_y   : " << idx_y);
//		ROS_INFO_STREAM("idx     : " << idx);

		return idx;
	}

	Point reverseBinLookup(size_t index, const Point & min, size_t bins_x, size_t bins_y) {
		Point p{min};
		p.z = 0.0;
		size_t idx_y = index / bins_x;
		size_t idx_x = index - idx_y;
		p.x += idx_x * bin_size_ + bin_size_/2.0;
		p.y += idx_y * bin_size_ + bin_size_/2.0;
		return p;
	}

	// Necessary ROS-stuff
	ros::NodeHandle node_handle_;
	ros::ServiceServer occlusion_detection_server_;

	/// Publishers for debugging.
	ros::Publisher occlusion_map_publisher_;
	ros::Publisher occluding_points_publisher_;

	double bin_size_ = 0.01;
	bool publish_clouds_ = false;
};


} // namespace

int main(int argc, char ** argv) {
	ros::init(argc, argv, "occlusion_detection");
	apc16delft::OcclusionDetectionNode node;
	ros::spin();
}
