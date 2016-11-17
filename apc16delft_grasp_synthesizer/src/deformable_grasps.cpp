#include "deformable_grasps.hpp"
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

namespace apc16delft {

DeformableGraspsResult synthesizeDeformableGrasps(
	PointCloud::ConstPtr cloud, ///< The input cloud.
	double normal_radius,       ///< The radius with which to estimate normals.
	double leaf_size            ///< The size of the voxel grid filter for downsampling the number of normals generated.
) {
	SearchMethod::Ptr search(new SearchMethod);

	std::vector<int> indices;
	PointCloud::Ptr input(new PointCloud);
	pcl::removeNaNFromPointCloud(*cloud, *input, indices);

	// create the filtering object
	pcl::VoxelGrid<Point> voxel_filter;
	voxel_filter.setInputCloud(input);
	voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
	voxel_filter.filter(*input);

	/*
	NormalEstimation normal_estimation;
	normal_estimation.setInputCloud(cloud);
	normal_estimation.setNormalEstimationMethod(normal_estimation.AVERAGE_3D_GRADIENT);
	normal_estimation.setMaxDepthChangeFactor(normal_radius);
	normal_estimation.setNormalSmoothingSize(15);
	normal_estimation.setViewPoint(0, 0, std::numeric_limits<float>::max());
	*/

	NormalEstimation normal_estimation;
	normal_estimation.setInputCloud(input);
	normal_estimation.setSearchMethod(search);
	normal_estimation.setRadiusSearch(normal_radius);
	//normal_estimation.setViewPoint(0, std::numeric_limits<float>::max(), 0); // ASSUMPTION: Point cloud is in world frame.
	normal_estimation.setViewPoint(0, 1000.0, 0); // ASSUMPTION: Point cloud is in world frame.

	PointCloudNormal::Ptr cloud_normals(new PointCloudNormal);
	normal_estimation.compute(*cloud_normals);

	return std::make_tuple(input, cloud_normals);
}

}

