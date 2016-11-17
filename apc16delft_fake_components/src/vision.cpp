#include <ros/ros.h>
#include <apc16delft_util/managed_node.hpp>
#include <apc16delft_msgs/DetectObjects.h>
#include <apc16delft_msgs/CropPointCloud.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>

namespace apc16delft {

void mouseCallback(int event, int x, int y, int flags, void * userdata);

class VisionNode : ManagedNode {
protected:

	bool detectObjects(apc16delft_msgs::DetectObjects::Request & req, apc16delft_msgs::DetectObjects::Response & res) {
		cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(req.color, sensor_msgs::image_encodings::BGR8);

		if (req.objects.size() == 0) {
			ROS_ERROR_STREAM("No given content in bin.");
			return false;
		}

		// create an opencv window
		cv::namedWindow("Image", cv::WINDOW_NORMAL);
		cv::setMouseCallback("Image", mouseCallback, this);

		// show the image
		cv::imshow("Image", cv_image->image);
		image_buffer = cv_image->image.clone();

		// wait for confirmation
		cv::waitKey();

		// copy to response
		apc16delft_msgs::ObjectDetection object;
		object.object.type = req.objects[0];
		object.score       = 1.0;
		object.box.x1      = std::min(start_drag.x, end_drag.x);
		object.box.y1      = std::min(start_drag.y, end_drag.y);
		object.box.x2      = std::max(start_drag.x, end_drag.x);
		object.box.y2      = std::max(start_drag.y, end_drag.y);

		apc16delft_msgs::CropPointCloud crop_srv;
		crop_srv.request.point_cloud = req.point_cloud;
		apc16delft_msgs::CropBox2D crop_box;
		crop_box.crop_inside.data = true;
		crop_box.box.x1 = object.box.x1;
		crop_box.box.y1 = object.box.y1;
		crop_box.box.x2 = object.box.x2;
		crop_box.box.y2 = object.box.y2;
		crop_srv.request.boxes.push_back(crop_box);

		if (!crop_cloud_client.call(crop_srv) ||
			crop_srv.response.cropped_point_clouds.size() == 0 ||
			crop_srv.response.centroids.size() == 0) {
			ROS_ERROR_STREAM("Failed to crop point cloud.");
			return false;
		}

		object.point_cloud = crop_srv.response.cropped_point_clouds[0];
		object.centroid    = crop_srv.response.centroids[0];

		res.objects.push_back(object);

		// destroy the window
		cv::destroyWindow("Image"); // CH: this does not seem to work for me
		cv::waitKey(20);

		return true;
	}

	/// The ROS service server for detecting an object in an image.
	ros::ServiceServer detect_objects_server;

	/// Client for cropping point clouds.
	ros::ServiceClient crop_cloud_client;

	bool dragging;
	cv::Point start_drag;
	cv::Point end_drag;
	cv::Mat image_buffer;

public:
	VisionNode() {
		// activate camera data service server
		detect_objects_server = node_handle.advertiseService("detect_objects", &VisionNode::detectObjects, this);

		// connect crop point cloud tool
		crop_cloud_client = node_handle.serviceClient<apc16delft_msgs::CropPointCloud>("/point_cloud_crop/crop_point_cloud");

		dragging = false;
	}

	void mouseCallbackImpl(int event, int x, int y, int, void *) {
		if (dragging) {
			end_drag = cv::Point(x, y);
			cv::Mat draw = image_buffer.clone();
			cv::rectangle(draw, start_drag, end_drag, cv::Scalar(0, 255, 0), 3);
			cv::imshow("Image", draw);
		}

		if (event == cv::EVENT_LBUTTONDOWN) {
			start_drag = cv::Point(x, y);
			dragging = true;
		} else if (event == cv::EVENT_LBUTTONUP) {
			dragging = false;
		}
	}
};

void mouseCallback(int event, int x, int y, int flags, void * userdata) {
	((VisionNode *)userdata)->mouseCallbackImpl(event, x, y, flags, userdata);
}

}

int main(int argc, char ** argv) {
	ros::init(argc, argv, ROS_PACKAGE_NAME "_vision");
	apc16delft::VisionNode node;
	ros::spin();
}
