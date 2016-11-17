#include <apc16delft_msgs/CropPointCloud.h>
#include <apc16delft_msgs/FilterObject.h>
#include <apc16delft_msgs/Object.h>
#include <apc16delft_msgs/objects.hpp>

#include <apc16delft_util/util.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>

#include <Eigen/Eigen>

namespace apc16delft {

class PointCloudCropNode {

using Point          = pcl::PointXYZ;
using PointCloud     = pcl::PointCloud<Point>;
using SearchMethod   = pcl::search::KdTree<Point>;
using CropResult     = std::tuple<PointCloud::Ptr, geometry_msgs::Point>;

public:
	PointCloudCropNode() : node_handle("~") {
		// service servers
		crop_point_cloud_server = node_handle.advertiseService("crop_point_cloud", &PointCloudCropNode::cropPointCloud, this);
		filter_object_server    = node_handle.advertiseService("filter_object", &PointCloudCropNode::filterObject, this);

		// parameters
		node_handle.param<double>("filter_distance", filter_distance, 0.01);

		loadModels();
	}

	~PointCloudCropNode() {
		// deactivate services
		crop_point_cloud_server.shutdown();
	}

protected:

	void loadModels() {
		// index 0 is nothing, so add an empty pointcloud for that index
		models.emplace_back(new PointCloud);

		for (int i = 1; i < apc16delft_msgs::Object::MAX_OBJECT; i++) {
			std::string model_path;
			if (!node_handle.getParam("/item/" + apc16delft_msgs::objectTypeToString(i) + "/model_path", model_path)) {
				ROS_WARN_STREAM("Failed to read model path for object: " << apc16delft_msgs::objectTypeToString(i));
				models.emplace_back(new PointCloud);
				continue;
			}

			// resolve package://* to file package path
			model_path = apc16delft::resolvePackagePath(model_path);

			// check if file exists
			if (!boost::filesystem::exists(model_path)) {
				ROS_WARN_STREAM("Model file for item " << apc16delft_msgs::objectTypeToString(i) << " does not exist. Path: " << model_path);
				models.emplace_back(new PointCloud);
				continue;
			}

			// read model
			PointCloud::Ptr model(new PointCloud);
			if (pcl::io::loadPCDFile(model_path, *model) == -1) {
				ROS_WARN_STREAM("No model found. Model path: " << model_path);
				models.emplace_back(new PointCloud);
				continue;
			}

			// check for empty model
			if (model->empty()) {
				ROS_ERROR_STREAM("Model point cloud is empty.");
				models.emplace_back(new PointCloud);
				continue;
			}

			// remove NaN's
			std::vector<int> indices;
			pcl::removeNaNFromPointCloud(*model, *model, indices);

			ROS_INFO_STREAM("Adding model for " << apc16delft_msgs::objectTypeToString(i) << " to cache.");

			// add to cache
			models.push_back(model);
		}
	}

	PointCloud::Ptr getSegmentedScene(PointCloud::ConstPtr scene, PointCloud::ConstPtr transformed_model) {
		pcl::ScopeTime t("Segmentation");

		SearchMethod search;
		search.setInputCloud(scene);

		std::vector<int> nn_indices;
		std::vector<float> nn_dists;
		std::set<int> indices;

		// find nearby points
		for (Point p: transformed_model->points) {
			search.radiusSearch(p, filter_distance, nn_indices, nn_dists);
			for (int i: nn_indices)
				indices.insert(i);
		}

		// copy indices to PointIndices struct
		pcl::PointIndices::Ptr point_indices(new pcl::PointIndices);

		std::copy(indices.begin(), indices.end(), std::back_inserter(point_indices->indices));

		// remove indices from scene
		PointCloud::Ptr segmented_scene(new PointCloud(*scene));
		pcl::ExtractIndices<Point> ei;
		ei.setIndices(point_indices);
		ei.setNegative(true);

		ei.filterDirectly(segmented_scene);

		return segmented_scene;
	}

	/// Service call for filtering an object out of a point cloud.
	bool filterObject(apc16delft_msgs::FilterObject::Request & req, apc16delft_msgs::FilterObject::Response & res) {
		// convert sensor_msgs/PointCloud2 to pcl::PointCloud
		PointCloud::Ptr scene(new PointCloud);
		pcl::fromROSMsg(req.scene, *scene);

		if (!scene || scene->empty()) {
			ROS_ERROR_STREAM("Scene is empty.");
			res.error.code    = res.E_SCENE_EMPTY;
			res.error.message = "Scene is empty.";
			return true;
		}

		if (req.object.type <= 0 || req.object.type >= apc16delft_msgs::Object::MAX_OBJECT) {
			ROS_ERROR_STREAM("Invalid object given: " << req.object.type);
			res.error.code    = res.E_INVALID_OBJECT;
			res.error.message = "Invalid object given: " + std::to_string(req.object.type);
			return true;
		}

		// get model from cache
		PointCloud::ConstPtr model = models[req.object.type];
		if (!model || model->empty()) {
			ROS_ERROR_STREAM("No model found.");
			res.error.code    = res.E_NO_MODEL;
			res.error.message = "No model found.";
			return true;
		}

		boost::optional<CropResult> crop_result = cropPointCloudImpl(scene, req.crop_box);

		if (!crop_result) {
			ROS_ERROR_STREAM("Failed to crop the bounding box.");
			res.error.code = res.E_BOUNDING_BOX_CROP;
			res.error.message = "Failed to crop the bounding box.";
			return true;
		}

		// transform the model using the object pose
		PointCloud::Ptr transformed_model(new PointCloud);
		pcl::transformPointCloud(*model, *transformed_model, Eigen::Affine3d{dr::toEigen(req.pose)});

		// segment the scene using the object model
		PointCloud::Ptr segmented_scene = getSegmentedScene(std::get<0>(*crop_result), transformed_model);
		segmented_scene->header = std::get<0>(*crop_result)->header;

		// convert to ROS message
		pcl::toROSMsg(*segmented_scene, res.filtered_scene);

		return true;
	}

	boost::optional<CropResult> cropPointCloudImpl(PointCloud::ConstPtr cloud, apc16delft_msgs::CropBox2D const & crop_box) {
		apc16delft_msgs::BoundingBox2D const & box = crop_box.box;
		bool crop_inside = crop_box.crop_inside.data;

		// prepare result
		PointCloud::Ptr cropped_cloud;
		if (crop_inside) {
			// construct a new (smaller) point cloud
			cropped_cloud = PointCloud::Ptr(new PointCloud);
		} else {
			// copy the original point cloud and remove points
			cropped_cloud = PointCloud::Ptr(new PointCloud(*cloud));
		}

		// check for invalid bounding boxes
		int width = box.x2 - box.x1;
		int height = box.y2 - box.y1;
		if (width <= 0 || height <= 0 ||
			box.x1 < 0 || box.y1 < 0 ||
			box.x2 >= int(cloud->width) || box.y2 >= int(cloud->height) ||
			box.x1 >= box.x2 || box.y1 >= box.y2) {
			ROS_ERROR_STREAM("The bounding box is invalid.");
			return boost::none;
		}

		// select the points from the point cloud (excluding NaNs)
		geometry_msgs::Point centroid;
		cropped_cloud->header   = cloud->header;
		cropped_cloud->is_dense = cloud->is_dense;

		int count = 0;
		const float bad_point = std::numeric_limits<float>::quiet_NaN();

		for (int y = box.y1; y < box.y2; y++) {
			for (int x = box.x1; x < box.x2; x++) {
				// get the point at this position
				pcl::PointXYZ const & p = cloud->at(x, y);

				if (crop_inside) {
					// add selected bounding box, we crop inside
					cropped_cloud->push_back(p);
				} else {
					// delete points from selected bounding box, we invert the crop
					cropped_cloud->at(x, y).x = bad_point;
					cropped_cloud->at(x, y).y = bad_point;
					cropped_cloud->at(x, y).z = bad_point;
				}

				// add for centroid calculation
				if (!std::isnan(p.x) &&
					!std::isnan(p.y) &&
					!std::isnan(p.z))
				{
					++count;
					centroid.x += p.x;
					centroid.y += p.y;
					centroid.z += p.z;
				}

			}
		}

		// set the width and height correctly
		if (crop_inside) {
			cropped_cloud->width  = width;
			cropped_cloud->height = height;
		}

		// compute the centroid
		centroid.x /= count;
		centroid.y /= count;
		centroid.z /= count;

		return std::make_tuple(cropped_cloud, centroid);
	}

	/// Service call for estimating the pose of an object.
	bool cropPointCloud(apc16delft_msgs::CropPointCloud::Request & req, apc16delft_msgs::CropPointCloud::Response & res) {
		// convert sensor_msgs/PointCloud2 to pcl::PointCloud
		PointCloud::Ptr cloud(new PointCloud);
		pcl::fromROSMsg(req.point_cloud, *cloud);

		// check for empty point clouds
		if (cloud->empty()) {
			ROS_ERROR_STREAM("Input point cloud is empty.");
			return false;
		}

		for (apc16delft_msgs::CropBox2D & crop_box : req.boxes ) {
			boost::optional<CropResult> crop_result = cropPointCloudImpl(cloud, crop_box);

			if (crop_result) {
				// copy output to response
				res.centroids.push_back(std::get<1>(*crop_result));
				sensor_msgs::PointCloud2 cropped_cloud_msg;
				pcl::toROSMsg(*std::get<0>(*crop_result), cropped_cloud_msg);
				res.cropped_point_clouds.push_back(cropped_cloud_msg);
			} else {
				ROS_ERROR_STREAM("Failed to filter point cloud, couldn't crop it.");
				return false;
			}
		}

		return true;
	}

	// The ROS node handle.
	ros::NodeHandle node_handle;

	/// Service server for cropping point clouds.
	ros::ServiceServer crop_point_cloud_server;

	/// Service server for filtering an object from a point cloud.
	ros::ServiceServer filter_object_server;

	/// List of cached models.
	std::vector<PointCloud::Ptr> models;

	/// The distance with which to filter objects from point clouds.
	double filter_distance;
};

}

int main(int argc, char ** argv) {
	ros::init(argc, argv, "point_cloud_crop");
	apc16delft::PointCloudCropNode node;
	ros::spin();
}
