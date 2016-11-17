#include "deformable_grasps.hpp"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char * * argv) {
	if (argc != 2) {
		std::cout << "Usage: " << argv[0] << " <point_cloud>" << std::endl;
		return 0;
	}

	apc16delft::PointCloud::Ptr cloud(new apc16delft::PointCloud);
	pcl::io::loadPCDFile(argv[1], *cloud);

	apc16delft::DeformableGraspsResult result = apc16delft::synthesizeDeformableGrasps(cloud, 0.005, 0.02);
	pcl::io::savePCDFile("normals.pcd", *std::get<1>(result));

	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	viewer.setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
	viewer.addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer.addPointCloudNormals<apc16delft::Point, apc16delft::PointNormal>(std::get<0>(result), std::get<1>(result), 1, 0.05, "normals");
	viewer.addCoordinateSystem(0.5);
	viewer.initCameraParameters();

	while (!viewer.wasStopped()) {
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds (100000));
	}

	return 0;
}
