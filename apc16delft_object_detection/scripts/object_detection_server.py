#!/usr/bin/python
from concurrent.futures import ThreadPoolExecutor

import rospy
import roslib
import numpy as np
import cv2
import os
import sys
import itertools

from apc16delft_msgs.srv import DetectObjects, DetectObjectsResponse, CropPointCloud
from apc16delft_msgs.msg import BoundingBox2D, Object, ObjectDetection, CropBox2D
from apc16delft_object_detection.DetectObjectBoundingBox import BoundingBoxDetection
import apc16delft_msgs.objects

from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from tf import transformations

# Object detection node
class ObjectDetectionServer:
	# Constructor.
	def __init__(self, executor):
		# intialize ROS stuff
		service                         = rospy.Service("~detect_objects"    , DetectObjects, self.detectObjects)
		self.detection_result_publisher = rospy.Publisher("~detection_result", Image        , queue_size=1, latch=True)
		self.marker_publisher           = rospy.Publisher("~markers"         , MarkerArray  , queue_size=1, latch=True)
		self.bridge                     = CvBridge()
		self.executor                   = executor
		self.crop_point_cloud_client    = rospy.ServiceProxy("/point_cloud_crop/crop_point_cloud", CropPointCloud)

		# initialize classifier
		self.initClassifier()

		# initialize bin and tote parameters
		self.initParameters()

		# read parameters
		self.visualize = rospy.get_param("~visualize", False)
		self.bin_margin = np.zeros((3))
		self.bin_margin[0] = rospy.get_param("~bin_margin/x", 0.05)
		self.bin_margin[1] = rospy.get_param("~bin_margin/y", 0.05)
		self.bin_margin[2] = rospy.get_param("~bin_margin/z", 0.05)

		rospy.loginfo("Object detection service ready to go.")

	# Method for initializing the neural network.
	def initClassifier(self):
		# path must be set nicer, but ok for now
		caffe_model = rospy.get_param("~caffe_model")
		prototxt    = rospy.get_param("~prototxt")

		# check existing files
		if not os.path.isfile(caffe_model):
			rospy.logerr("No caffemodel found.")
			sys.exit()
		if not os.path.isfile(prototxt):
			rospy.logerr("No model definition (prototxt) found.")
			sys.exit()

		# Initialize classifier in a seperate thread.
		# This is needed to ensure the proper thread owns the CUDA context.
		def init():
			self.classifier = BoundingBoxDetection(caffe_model, prototxt)
		self.executor.submit(init).result()

		rospy.loginfo("Classification module initialized.")

	# Read and store parameters
	def initParameters(self):
		# Initialize bin dimensions
		self.bin_dimensions = []
		for bin_index in range(12):
			# check if bin dimensions exist in ros param
			if not rospy.has_param("/bin/{}/dimensions/x".format(bin_index)) or \
			   not rospy.has_param("/bin/{}/dimensions/y".format(bin_index)) or \
			   not rospy.has_param("/bin/{}/dimensions/z".format(bin_index)):
				rospy.logerr("Bin dimensions parameter not found.")
				return None

			# get bin dimensions
			bin_dimensions = [
				rospy.get_param("/bin/{}/dimensions/x".format(bin_index)), \
				rospy.get_param("/bin/{}/dimensions/y".format(bin_index)), \
				rospy.get_param("/bin/{}/dimensions/z".format(bin_index)) \
			]

			# store bin dimensions
			self.bin_dimensions += [bin_dimensions]

		# Initialize tote dimensions
		if not rospy.has_param("/tote/dimensions/x") or \
		   not rospy.has_param("/tote/dimensions/y") or \
		   not rospy.has_param("/tote/dimensions/z"):
			rospy.logerr("Tote dimensions parameter not found.")
			return None

		# get bin dimensions
		self.tote_dimensions = [
			rospy.get_param("/tote/dimensions/x"), \
			rospy.get_param("/tote/dimensions/y"), \
			rospy.get_param("/tote/dimensions/z") \
		]


	# Method for detecting objects in an image.
	# Returns scores and bounding boxes for detected objects.
	def classify(self, image):
		rospy.loginfo("Classification called.")
		# Run the classifier in its own thread so that it always owns the CUDA context.
		future = self.executor.submit(self.classifier.getObjectBoundingBox, image)
		scores, boxes = future.result()
		return scores, boxes

	def publishMarkers(self, objects, pruned_objects, bin_pose, camera_to_bin, bin_dimensions, header):
		# construct centroids marker array
		marker_array = MarkerArray()

		# clear old markers
		marker = Marker()
		marker.action = 3 # secret action to delete all markers
		marker_array.markers.append(marker)
		self.marker_publisher.publish(marker_array)
		marker_array.markers = []

		# add centroid markers
		for index, object in enumerate(objects):
			marker               = Marker()
			marker.header        = header
			marker.type          = marker.SPHERE
			marker.action        = marker.ADD
			marker.scale.x       = 0.02
			marker.scale.y       = 0.02
			marker.scale.z       = 0.02
			marker.color.g       = 1.0
			marker.color.a       = 0.5
			marker.pose.position = object.centroid
			marker.ns            = apc16delft_msgs.objects.objectTypeToString[object.object.type]
			marker.id            = len(marker_array.markers)

			marker_array.markers.append(marker)

		# add pruned centroid markers
		for index, object in enumerate(pruned_objects):
			marker               = Marker()
			marker.header        = header
			marker.type          = marker.SPHERE
			marker.action        = marker.ADD
			marker.scale.x       = 0.02
			marker.scale.y       = 0.02
			marker.scale.z       = 0.02
			marker.color.r       = 1.0
			marker.color.a       = 0.5
			marker.pose.position = object.centroid
			marker.ns            = apc16delft_msgs.objects.objectTypeToString[object.object.type]
			marker.id            = len(marker_array.markers)

			marker_array.markers.append(marker)

		# add bin marker
		marker         = Marker()
		marker.header  = header
		marker.type    = marker.CUBE
		marker.action  = marker.ADD
		marker.scale.x = bin_dimensions[0] + self.bin_margin[0] * 2
		marker.scale.y = bin_dimensions[1] + self.bin_margin[1] * 2
		marker.scale.z = bin_dimensions[2] + self.bin_margin[2] * 2
		marker.color.b = 1.0
		marker.color.a = 0.2
		marker.ns      = "bin_marker"
		marker.id      = len(marker_array.markers)

		# transform so that the marker is originating from the bin_pose
		position                = np.append(np.array(bin_dimensions) * 0.5, 1)
		position                = transformations.inverse_matrix(camera_to_bin).dot(position)
		marker.pose.position.x  = position[0]
		marker.pose.position.y  = position[1]
		marker.pose.position.z  = position[2]
		marker.pose.orientation = bin_pose.orientation

		marker_array.markers.append(marker)

		self.marker_publisher.publish(marker_array)

	def filterDetections(self, content, scores, boxes, point_cloud, camera_to_bin, bin_dimensions):
		rospy.loginfo("Detection filter called")

		# initialize empty data structures
		objects        = []
		pruned_objects = []

		crop_boxes        = []
		crop_boxes_result = dict((key, []) for key in scores.keys())

		# iterate over the items in the bins
		for item in np.unique(content):
			# skip empty items
			if len(scores[item]) == 0 or len(boxes[item]) == 0:
				continue

			# iterate over the scores/boxes of the current class type
			for score, box in zip(scores[item], boxes[item]):
				# fill a CropBox2D message
				crop_box                  = CropBox2D(box=BoundingBox2D(box[0], box[1], box[2], box[3]))
				crop_box.crop_inside.data = True

				# add the message to the list of messages
				crop_boxes += [crop_box]

		# check if we have any results at all
		if len(crop_boxes) == 0:
			rospy.logerr("No cropboxes generated.")
			return [], []

		# crop point cloud using external node
		cropped_point_cloud_res = self.crop_point_cloud_client(crop_boxes, point_cloud)

		# convert list back to dict
		for item in np.unique(content):
			for i in range(len(scores[item])):
				crop_boxes_result[item]                     += [(cropped_point_cloud_res.centroids[0], cropped_point_cloud_res.cropped_point_clouds[0])]
				cropped_point_cloud_res.centroids            = np.delete(cropped_point_cloud_res.centroids, 0, 0)
				cropped_point_cloud_res.cropped_point_clouds = np.delete(cropped_point_cloud_res.cropped_point_clouds, 0, 0)

		# these should now be empty
		assert(len(cropped_point_cloud_res.centroids) == 0)
		assert(len(cropped_point_cloud_res.cropped_point_clouds) == 0)

		# find a best match for every item
		for item in content:
			# skip item 0, it is background
			if item == 0:
				continue

			# skip empty arrays
			if len(scores[item]) == 0:
				continue

			# find the most confident detection
			index   = np.argmax(scores[item])
			score   = scores[item][index]
			box     = boxes[item][index]
			cropbox = crop_boxes_result[item][index]

			# delete this result from the dataset
			scores[item]            = np.delete(scores[item], index, 0)
			boxes[item]             = np.delete(boxes[item], index, 0)
			crop_boxes_result[item] = np.delete(crop_boxes_result[item], index, 0)

			# compute centroid in bin frame
			centroid = np.array([cropbox[0].x, cropbox[0].y, cropbox[0].z, 1])
			centroid = camera_to_bin.dot(centroid)

			# fill output
			detected_object             = ObjectDetection()
			detected_object.object      = Object(item)
			detected_object.score       = score
			detected_object.box         = BoundingBox2D(box[0], box[1], box[2], box[3])
			detected_object.centroid    = cropbox[0]
			detected_object.point_cloud = cropbox[1]

			# check if centroid is inside bin dimensions
			if np.any(centroid[:3] < -self.bin_margin) or np.any(centroid[:3] > bin_dimensions + self.bin_margin):
				class_name = apc16delft_msgs.objects.objectTypeToString[item]
				rospy.logwarn("{} detected outside of bin.".format(class_name))
				rospy.logwarn("Bin dimensions: {}.".format(bin_dimensions))
				rospy.logwarn("Centroid in given frame: {}.".format([cropbox[0].x, cropbox[0].y, cropbox[0].z]))
				rospy.logwarn("Centroid in bin frame: {}.".format(centroid))
				rospy.logwarn("Transform from camera to bin frame: \n{}.".format(camera_to_bin))

				# add to list of pruned objects
				pruned_objects += [detected_object]

				# place item back in content because we did not detect an item in our bin
				content += [item]
				continue

			# add to list of classifications
			objects += [detected_object]

		rospy.loginfo("Found {} object(s).".format(len(objects)))
		rospy.loginfo("Pruned {} object(s).".format(len(pruned_objects)))

		return objects, pruned_objects

	def detectObjects(self, req):
		rospy.loginfo("Got object detection request.")

		if req.bin_index >=0 and req.bin_index < 12:
			bin_dimensions = self.bin_dimensions[req.bin_index]
		elif req.bin_index == -1:
			if 'tote_dimensions' in dir(self):
				bin_dimensions = self.tote_dimensions
				rospy.loginfo("Using tote dimensions.")
			else:
				rospy.logerr("Tote dimensions required, but not loaded.")
				return None
		else:
			rospy.logerr("Invalid bin index given: {}.".format(req.bin_index))
			return None


		# convert bin_pose to numpy transformation matrix
		rotation      = transformations.quaternion_matrix([req.bin_pose.orientation.x, req.bin_pose.orientation.y, req.bin_pose.orientation.z, req.bin_pose.orientation.w])
		translation   = transformations.translation_matrix([req.bin_pose.position.x, req.bin_pose.position.y, req.bin_pose.position.z])
		camera_to_bin = transformations.inverse_matrix(translation.dot(rotation))

		# detect and filter objects based on bin contents and bin pose
		image                   = self.bridge.imgmsg_to_cv2(req.color)
		scores, boxes           = self.classify(image)
		objects, pruned_objects = self.filterDetections(list(req.objects), scores, boxes, req.point_cloud, camera_to_bin, bin_dimensions)

		# publish markers for debugging
		self.publishMarkers(objects, pruned_objects, req.bin_pose, camera_to_bin, bin_dimensions, req.point_cloud.header)

		# visualize detections if necessary
		if(self.visualize):
			for object in objects:
				box = object.box
				class_name = apc16delft_msgs.objects.objectTypeToString[object.object.type]

				cv2.rectangle(image, (box.x1, box.y1), (box.x2, box.y2), (0, 255, 0), 4)
				cv2.putText(image, class_name + " " + str(object.score), (int(box.x1 + 10), int(box.y1 + 30)), cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 0, 255), 2)

			self.detection_result_publisher.publish(self.bridge.cv2_to_imgmsg(image, encoding="bgr8"))

		# set output for service call
		response         = DetectObjectsResponse()
		response.objects = objects
		return response

# main object detection function
if __name__ == "__main__":
	rospy.init_node('object_detection_server')
	with ThreadPoolExecutor(max_workers=1) as executor:
		pipeline = ObjectDetectionServer(executor)
		rospy.spin()
