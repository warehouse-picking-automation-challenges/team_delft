#include <apc16delft_msgs/EstimateBinDensity.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <ros/init.h>
#include <geometry_msgs/Point.h>
#include <eigen_conversions/eigen_msg.h>

#include <boost/bind.hpp>

namespace apc16delft {

using Point           = pcl::PointXYZ;
using PointCloud      = pcl::PointCloud<Point>;
using BinDensityReq   = apc16delft_msgs::EstimateBinDensity::Request;
using BinDensityRes   = apc16delft_msgs::EstimateBinDensity::Response;
using Viewer          = pcl::visualization::PCLVisualizer;


class BinDensityNode {

public:

/// BinDensityNode constructor, starting communication with ros server
BinDensityNode(){
	density_server_ = node_handle_.advertiseService("/bin_density/estimate_bin_density", &BinDensityNode::binDensity, this);

	node_handle_.param<bool>("/bin_density/viewer", viewer_, false);
	node_handle_.param<double>("/bin_density/acceptable_angle_", acceptable_angle_, 0.1*(M_PI/180));
}

protected:

/// Callback to continue and stop the viewer
void keyboardEventOccurred(pcl::visualization::KeyboardEvent const & event) {
	if (event.getKeySym() == "space" && event.keyDown()) {
		viewer_ = false;
	}
}

/// Ros node handle
ros::NodeHandle node_handle_;

/// Ros service server for calculating density map
ros::ServiceServer density_server_;

/// Use the pcl viewer to show the result
bool viewer_;

/// Threshold parameter for assuming that the point would be seen by the camera
double acceptable_angle_;

double calculateAngle(Point const & point1, Point const & point2) {
	Eigen::Vector3f vector1 = point1.getVector3fMap();
	Eigen::Vector3f vector2 = point2.getVector3fMap();
	double dot_product = vector1.dot(vector2);
	double norm1 = vector1.norm();
	double norm2 = vector2.norm();
	double angle = std::acos(dot_product / (norm1 * norm2));
	return angle;
}

/// Returns true if the grid point would be visible by the camera
bool isVisible(PointCloud::ConstPtr bin, Point const & grid_point, Eigen::Isometry3d const & bin_pose, Eigen::Isometry3d const & lens_pose, double const acceptable_angle) {
	// Transform point back to point cloud frame
	pcl::transformPoint(grid_point, bin_pose.inverse().cast<float>());

	// Transform point to left camera lens frame, effectively placing lens at the origin
	pcl::transformPoint(grid_point, lens_pose.inverse().cast<float>());

	// Calculate if line point-lens is close to other points
	double min_angle = std::numeric_limits<double>::max();
	for (auto const & bin_point : *bin) {
			double angle = calculateAngle(grid_point, bin_point);
		if (angle < min_angle) {
			min_angle = angle;
		}
	}

	// Return true if close to other points of an object
	if (min_angle > acceptable_angle) {
		std::cerr << "visible: " << (180/M_PI) * min_angle << std::endl;
		return true;
	} else {
		std::cerr << "not visible: " << (180/M_PI) * min_angle << std::endl;
		return false;
	}
}

/// Returns true if the point can be seen by the camera and false if the point is blocked
PointCloud::Ptr gridToCandidates(PointCloud::ConstPtr bin, PointCloud::ConstPtr grid, Eigen::Isometry3d const & bin_pose, Eigen::Isometry3d const & lens_pose, double const acceptable_angle) {
	PointCloud::Ptr candidates (new PointCloud);
	for (auto const & grid_point : *grid) {
		if (isVisible(bin, grid_point, bin_pose, lens_pose, acceptable_angle)) {
			candidates->push_back(grid_point);
		}
	}
	return candidates;
}

/// Calculates placement positions, defining a 2D grid on the bottom of the bin.
PointCloud::Ptr calculateGrid(double const bin_width, double const bin_depth, double const step_size) {
	// Loop over candidates and calculate a measure of density based on point density above bottom of bin
	PointCloud::Ptr grid (new PointCloud);
	int x_steps = std::floor(bin_width / step_size);
	int y_steps = std::floor(bin_depth / step_size);
	double x_start = (bin_width - x_steps * step_size) / 2.0;
	double y_start = (bin_depth - y_steps * step_size) / 2.0;
	double x_end = x_start + x_steps * step_size;
	double y_end = y_start + y_steps * step_size;
	for (double x(x_start); x<=(x_end); x+=step_size) {
		for (double y(y_start); y<=(y_end); y+=step_size) {
			Point current_point;
			current_point.x = x;
			current_point.y = y;
			current_point.z = 0;
			grid->push_back(current_point);
		}
	}
	return grid;
}

/// Calculates the density map in the bin, assuming that the cloud is already transformed to the expected pose
std::vector<unsigned int> calculateDensities(
	PointCloud::ConstPtr bin,
	PointCloud::Ptr candidates,
	float const radius
) {
	if (bin->empty()) {
		ROS_ERROR_STREAM("Failed to calculate densities. Received an empty bin point cloud.");
		return std::vector<unsigned int>();
	}

	// Create a kd tree to speed up radius search
	pcl::KdTreeFLANN<Point> kdtree;
	kdtree.setInputCloud(bin);

	std::vector<unsigned int> densities;
	for (auto const & point : *candidates) {
		std::vector<int> indices;
		std::vector<float> distances;
		kdtree.radiusSearch(point, radius, indices, distances);
		densities.push_back(indices.size());
	}

	return densities;
}

/// Show the result in the pcl viewer
void viewResult(PointCloud::ConstPtr bin, PointCloud::ConstPtr candidates, std::vector<double> const & scores){
	if (bin->empty()) {
		ROS_ERROR_STREAM("Failed to show point cloud. Bin point cloud is empty.");
		return;
	}
	if (candidates->empty()) {
		ROS_ERROR_STREAM("Failed to show candidates. Candidates point cloud is empty.");
		return;
	}
	if (scores.empty()) {
		ROS_ERROR_STREAM("Failed to show scores, because it's empty.");
		return;
	}

	Viewer viewer("Viewer");
	viewer.setBackgroundColor(0.3, 0.3, 0.3);
	ROS_INFO_STREAM("Press space on keyboard to continue and stop the viewer.");
	viewer.registerKeyboardCallback(boost::bind(&apc16delft::BinDensityNode::keyboardEventOccurred, this, _1));

	// Show point cloud of bin
	viewer.addPointCloud(bin, "bin");
	viewer.addCoordinateSystem();

	// Show line from candidates to origin
	for (size_t i=0; i < candidates->size(); i++) {
		viewer.addLine(Point(0,0,0), candidates->at(i), std::to_string(i) + "line");
	}

	// Show scores
	unsigned int counter(0);
	for (auto point : *candidates) {
		std::string id = std::to_string(counter);
		if (scores.at(counter) == 0) {
			viewer.addSphere(point, 0.005, 0, 1, 1, id);
		} else {
			viewer.addSphere(
				point,
				0.005,
				1 - scores.at(counter),
				1 - scores.at(counter),
				scores.at(counter),
				id
			);
		}
		counter++;
	}

	while (!viewer.wasStopped() && viewer_) {
		viewer.spinOnce ();
	}
	viewer.close();
}

/*
 * Calculates the scores for various placement positions.
 * Higher density of points means better placement position.
 * For now, score is equal to density.
*/
std::vector<double> calculateScores(PointCloud::ConstPtr candidates, std::vector<unsigned int> const & densities) {
	std::vector<double> scores(densities.begin(), densities.end());
	return scores;
}

/// Convert point cloud to array of geometry msgs point stamped
std::vector<geometry_msgs::PointStamped> toPointStampedArray(PointCloud::ConstPtr cloud) {
	// Create header
	std_msgs::Header header;
	pcl_conversions::fromPCL(cloud->header, header);

	// Create points stamped array
	std::vector<geometry_msgs::PointStamped> result;
	for (auto const & p : *cloud) {
		geometry_msgs::PointStamped data;
		data.point.x = p.x;
		data.point.y = p.y;
		data.point.z = p.z;
		data.header = header;
		result.push_back(data);
	}
	return result;
}

/// Callback function to exectute
bool binDensity(BinDensityReq & req, BinDensityRes & res){

	// Convert request to pcl cloud
	PointCloud::Ptr bin (new PointCloud);
	pcl::fromROSMsg(req.bin, *bin);
	if (bin->empty()) {
		ROS_ERROR_STREAM("Received empty point cloud from service call.");
		res.error.code = BinDensityRes::E_CLOUD_EMPTY;
		res.error.message = "Received empty bin point cloud";
		return true;
	}

	// Transform point cloud to bin frame
	Eigen::Isometry3d bin_pose;
	tf::poseMsgToEigen(req.bin_pose, bin_pose);
	pcl::transformPointCloud(*bin, *bin, bin_pose.cast<float>());

	// Get bin width and depth, because it may vary for the different bin indices
	double bin_width;
	node_handle_.param<double>(std::string("/bin/") + std::to_string(req.bin_index) + "/dimensions/x", bin_width, 0.268);
	double bin_depth;
	node_handle_.param<double>(std::string("/bin/") + std::to_string(req.bin_index) + "/dimensions/y", bin_depth, 0.43);

	// Returns grid points that can be seen by camera
	PointCloud::Ptr grid = calculateGrid(bin_width, bin_depth, req.step_size);

	// Get the pose of the lens of the left camera
	Eigen::Isometry3d lens_pose;
	tf::poseMsgToEigen(req.lens_pose, lens_pose);

	// Checks for each grid point if camera could see it, saves the observable points in candidates
	PointCloud::Ptr candidates = gridToCandidates(bin, grid, bin_pose, lens_pose, acceptable_angle_);

	std::cerr << "grid size: " << grid->size();
	std::cerr << "candidates size: " << candidates->size();

	// Add frame_id to candidates header
	candidates->header = bin->header;

	// Perform the actual calculation of bin density
	std::vector<unsigned int> densities = calculateDensities(bin, candidates, req.radius);

	if (densities.empty()) {
		ROS_ERROR_STREAM("Densities empty");
		res.error.code = BinDensityRes::E_SANITY_DENSITIES;
		res.error.message = "Sanity check on densities failed.";
		return true;
	}
	if (candidates->empty()) {
		ROS_ERROR_STREAM("Candidates empty");
		res.error.code = BinDensityRes::E_SANITY_CANDIDATES;
		res.error.message = "Sanity check on candidates failed.";
		return true;
	}

	// Fill the response
	res.candidates = toPointStampedArray(candidates);

	std::vector<double> scores = calculateScores(candidates, densities);
	res.scores = scores;

	// Show result in pcl viewer if viewer is set to true
	if (viewer_) {
		viewResult(bin, candidates, scores);
	}

	return true;
}

};

} // namespace


int main(int argc, char * * argv) {
	ros::init(argc, argv, "bin_density");
	ROS_INFO_STREAM("Started node 'bin_density'");
	apc16delft::BinDensityNode node;
	ros::spin();
}

