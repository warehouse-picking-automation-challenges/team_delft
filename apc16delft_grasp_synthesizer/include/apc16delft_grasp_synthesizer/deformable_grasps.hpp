#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d_omp.h>

namespace apc16delft {

using Point                  = pcl::PointXYZ;
using PointNormal            = pcl::Normal;
using PointCloud             = pcl::PointCloud<Point>;
using PointCloudNormal       = pcl::PointCloud<PointNormal>;
using NormalEstimation       = pcl::NormalEstimationOMP<Point, PointNormal>;
using SearchMethod           = pcl::search::KdTree<Point>;
using DeformableGraspsResult = std::tuple<PointCloud::Ptr, PointCloudNormal::Ptr>;

DeformableGraspsResult synthesizeDeformableGrasps(
	PointCloud::ConstPtr cloud, ///< The input cloud.
	double normal_radius,       ///< The radius with which to estimate normals.
	double leaf_size            ///< The size of the voxel grid filter for downsampling the number of normals generated.
);

}
