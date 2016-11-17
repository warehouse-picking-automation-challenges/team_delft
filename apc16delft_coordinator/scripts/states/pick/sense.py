import math
import rospy
import roslib
import smach

import numpy
import tf
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Bool, Header
from std_srvs.srv import Empty, EmptyRequest

from apc16delft_msgs import msg, srv
from apc16delft_msgs.objects import objectTypeToString

from lib.system_interface import HIGH_VELOCITY
from lib.bins import binIndexToString
from lib.counter_state import makeCounterStates
from lib.geometry_numpy import vector3FromList, quaternionToMatrix
from lib.objects import isDeformable
from lib.services import ServiceState, ServiceProxy as DumpingServiceProxy
from lib import master_pose

def tfPositionToGeometry(position):
	result = Point()
	result.x = position[0]
	result.y = position[1]
	result.z = position[2]
	return result

def tfOrientationToGeometry(orientation):
	result = Quaternion()
	result.x = orientation[0]
	result.y = orientation[1]
	result.z = orientation[2]
	result.w = orientation[3]
	return result

def tfPoseToGeometry(pose):
	result = Pose()
	result.position    = tfPositionToGeometry(pose[0])
	result.orientation = tfOrientationToGeometry(pose[1])
	return result

def tfLookup(tf, target_frame, source_frame, time, timeout = rospy.Duration(1)):
	tf.waitForTransform(target_frame, source_frame, time, timeout)
	return tfPoseToGeometry(tf.lookupTransform(target_frame, source_frame, time))

def tfTransformPose(tf, target_frame, pose, timeout = rospy.Duration(1)):
	tf.waitForTransform(target_frame, pose.header.frame_id, pose.header.stamp, timeout)
	return tf.transformPose(target_frame, pose)

def adjustedCameraPose(attempt):
	# TODO: Come up with useful values.
	header = Header(frame_id = 'gripper_tool0', stamp = rospy.Time.now())
	if attempt == 0: return PoseStamped(header=header, pose=Pose(position=Point(0.05,  0.0, 0.0), orientation=Quaternion(0, 0, 0, 1)))
	if attempt == 1: return PoseStamped(header=header, pose=Pose(position=Point(-0.05,  0.0, 0.0), orientation=Quaternion(0, 0, 0, 1)))
	if attempt == 2: return PoseStamped(header=header, pose=Pose(position=Point(0.0, 0.05, 0.0), orientation=Quaternion(0, 0, 0, 1)))
	raise Exception("No camera adjustments left.")

def selectProbableOccluder(target_type, detections):
	# Select the detection that is not our target closest to the camera (lowest Z in calibrated frame).
	candidates = filter(lambda x: x.object.type != target_type, detections)
	if not candidates:
		return None
	return min(candidates, key = lambda x: x.centroid.z)


MOVE_BLACKLIST = [
	msg.Object.FITNESS_GEAR_3LB_DUMBBELL,
	msg.Object.HANES_TUBE_SOCKS,
	msg.Object.KLEENEX_PAPER_TOWELS,
	msg.Object.ROLODEX_JUMBO_PENCIL_CUP,
]

def getProbableGraspOccluders(target_detection, other_detections):
	candidates = [x for x in other_detections if x.object.type not in MOVE_BLACKLIST]
	if not candidates:
		return None

	closest     = min(candidates, key = lambda x: x.centroid.z)
	min_z       = closest.centroid.z;
	front_items = filter(lambda x: x.centroid.z < min_z + 0.04, candidates)
	return front_items

class SenseBinPick(smach.State):
	def __init__(self, tf, system):
		self.tf           = tf
		self.system       = system

		smach.State.__init__(self,
			outcomes     = ['success', 'occluded', 'no_detection', 'error'],
			input_keys   = ['current_master_pose', 'job', 'bin_contents'],
			output_keys  = ['source_bin_pose', 'object_pose', 'object_cloud', 'robot_transform', 'occluder', 'grasp_occluders', 'current_master_pose']
		)

		self.get_camera_data          = rospy.ServiceProxy('/gripper_camera/get_data',                         srv.GetCameraData)
		self.estimate_bin_pose        = DumpingServiceProxy('/bin_pose_estimation/estimate_bin_pose',          srv.EstimateBinPose, dump_request = False)
		self.detect_objects           = DumpingServiceProxy('/bin_detection/detect_objects',                   srv.DetectObjects,   dump_request = False)
		self.filter_objects           = rospy.ServiceProxy('/point_cloud_crop/filter_object',                  srv.FilterObject)
		self.crop_point_cloud         = rospy.ServiceProxy('/point_cloud_crop/crop_point_cloud',               srv.CropPointCloud)
		self.publish_point_cloud      = rospy.ServiceProxy('/static_pointcloud_publisher/publish_point_cloud', srv.PublishStaticPointCloud)
		self.estimate_object_pose     = DumpingServiceProxy('/pose_estimation/estimate_object_pose',           srv.EstimateObjectPose)
		self.detect_occlusions        = DumpingServiceProxy('/occlusion_detection/detect_occlusion',           srv.DetectOcclusion)
		self.clear_static_point_cloud = DumpingServiceProxy('/static_pointcloud_publisher/clear_static_point_cloud', Empty)
		self.clear_octomap            = DumpingServiceProxy('/clear_octomap', Empty)

	def retrieveCameraData(self, dump, publish):
		"""
		Record camera data.
		"""
		self.clear_static_point_cloud(EmptyRequest())
		self.clear_octomap(EmptyRequest())

		for retry in range(2):
			try:
				return self.get_camera_data(dump=dump, publish=publish)
			except rospy.ServiceException as e:
				rospy.logwarn('Failed to retrieve data from gripper camera: {}'.format(e))
		return self.get_camera_data(dump=dump, publish=publish)

	def recordCameraData(self, dump = False, publish = False):
		"""
		Record data with LEDs enabled.
		"""
		self.system.enableLed(True)
		result = self.retrieveCameraData(dump, publish)
		self.system.enableLed(False)
		return result

	def estimateBinPose(self, scene_cloud, initial_guess, bin_index):
		"""
		Estimate the pose of the bin.
		Returns a stamped pose, or None if the estimation fails.
		Also returns None if the estimation error is too high.
		"""
		estimation = self.estimate_bin_pose(
			scene               = scene_cloud,
			shelf_initial_guess = initial_guess,
			bin_index           = bin_index
		)

		if estimation.error.code:
			rospy.logwarn('Bin pose estimation failed with error {}: {}.'.format(estimation.error.code, estimation.error.message))
			return None

		return estimation.pose


	def estimateObjectPose(self, target_object, container_index, container_pose):
		"""
		Estimate the pose of an object.
		Returns a (stamped pose, transformed model cloud) tuple, or None if the estimation fails.
		Also returns None if the estimation error is too high.
		"""
		try:
			object_estimation = self.estimate_object_pose(
				object = target_object,
				container_index = container_index,
				container_pose = container_pose
			)
		except rospy.ServiceException as e:
			rospy.logwarn('Failed to estimate object pose: {}'.format(e))
			return None

		# TODO: What limit?
		if object_estimation.estimation_error > 0.2:
			rospy.logwarn('Estimated object pose is too unreliable, estimation error is {}.'.format(object_estimation.estimation_error))
			return None

		return object_estimation.pose, object_estimation.transformed_model

	def checkOcclusion(self, target_object, target_model, other_objects, camera_pose):
		"""
		Check if a target point cloud is occluded by other objects.
		"""

		def boundingboxOverlap(box1, box2):
			x1 = max([box1.x1, box2.x1])
			y1 = max([box1.y1, box2.y1])
			x2 = min([box1.x2, box2.x2])
			y2 = min([box1.y2, box2.y2])

			if x2 <= x1 or y2 <= y1:
				return 0

			area = (x2 - x1) * (y2 - y1)
			box1area = (box1.x2 - box1.x1) * (box1.y2 - box1.y1)
			return float(area)/box1area

		"""
		Perform the naive 3D occlusion detection.
		"""
		# Extract the occlusion axis (bin Y) for the occlusion detector.
		occlusion_axis = quaternionToMatrix(camera_pose.orientation) * numpy.matrix([[0.0], [0.0], [1.0], [0.0]])
		occlusion_axis = vector3FromList(occlusion_axis[:-1])

		result = self.detect_occlusions(
			target_model       = target_model,
			non_target_objects = other_objects,
			direction          = occlusion_axis
		)
		if result.error.code:
			raise Exception("Occlusion detection reported error {}: {}.".format(result.error.code, result.error.message))

		"""
		Check occlusions in 2D
		"""

		occlusion_score = 0
		occluders = []
		occluder_scores = []
		good_detection_score = 0.9 # TODO: Magic number
		for other_object in other_objects:
			overlap = boundingboxOverlap(target_object.box, other_object.box)
			rospy.loginfo("Overlap: {}".format(overlap))
			if overlap > 0:
				close_detection_scores = numpy.absolute(target_object.score - other_object.score) < 0.1 # TODO: Magic number
				if target_object.score < other_object.score and target_object.score < good_detection_score and not close_detection_scores:
					rospy.loginfo("Not a great detection, with overlap and the overlapping object does have a good detection.")
					occlusion_score += overlap
					occluder_scores.append(overlap)
					occluders.append(other_object)
				elif target_object.score >= good_detection_score or close_detection_scores:
					if close_detection_scores:
						rospy.loginfo("Object detection scores are pretty close, difference is: {}".format(numpy.absolute(target_object.score - other_object.score)))
					else:
						rospy.loginfo("Object detection score is good, but I still probably have an occluder, with overlap {}.".format(overlap))
					# Check the occlusion detection result from 3D checking?
					if not any(x.occluder.type == other_object.object.type for x in result.occlusions):
						rospy.loginfo("According to naive 3D occlusion detection no occluder has been found for this overlapping object...")
					else:
						rospy.loginfo("Found an occlusion in 2D and 3D")
						occlusion_score += overlap
						occluder_scores.append(overlap)
						occluders.append(other_object)
				else:
					rospy.loginfo("Got here, on a place that I wasn't expecting much...")
			else:
				rospy.loginfo("   ---  No overlap")

		if len(occluders) == 0 or occlusion_score < 0.1: # TODO: magic number...
			return None

		return max(zip(occluders, occluder_scores), key = lambda x: x[1])[0]

	def step(self, userdata, bin_estimation):
		# Record data and poses.
		camera_data     = self.recordCameraData(dump = True)
		time            = camera_data.point_cloud.header.stamp
		robot_pose      = tfLookup(self.tf, 'world', 'robot_tool0', time)
		camera_pose     = tfLookup(self.tf, 'robot_tool0', 'gripper_camera_depth', time)

		# Estimate the bin pose.
		if bin_estimation is None:
			shelf_pose     = tfLookup(self.tf, 'robot_tool0', 'shelf', time)
			bin_estimation = self.estimateBinPose(camera_data.point_cloud, shelf_pose, userdata.job.source_bin)
			if bin_estimation is None: return 'retry', bin_estimation
			bin_estimation = tfTransformPose(self.tf, 'world', bin_estimation)
			userdata.source_bin_pose = bin_estimation

		bin_estimation.header.stamp = time
		bin_estimation_in_robot_tool0 = tfTransformPose(self.tf, 'robot_tool0', bin_estimation)

		detection = self.detect_objects(
			color       = camera_data.color,
			point_cloud = camera_data.point_cloud,
			objects     = userdata.bin_contents[userdata.job.source_bin],
			bin_pose    = bin_estimation_in_robot_tool0.pose,
			bin_index   = userdata.job.source_bin
		)

		# Find the target object in the detections.
		object_types = [x.object.type for x in detection.objects]
		rospy.loginfo("Found objects: %s", ', '.join([objectTypeToString[x] for x in object_types]))

		# Set a probable occluder in case we fail at a later stage.
		# Occlusion detection will override the output key if needed.
		probable_occluder = selectProbableOccluder(userdata.job.object_type, detection.objects)
		if probable_occluder:
			userdata.occluder = probable_occluder.object.type

		if userdata.job.object_type not in object_types:
			rospy.logwarn("Wanted object (%s) not found", objectTypeToString[userdata.job.object_type])
			return 'retry', bin_estimation

		target_index     = object_types.index(userdata.job.object_type)
		other_objects    = list(detection.objects)
		target_detection = other_objects.pop(target_index)

		# Set the type of the closes neighbour, for reporting to task planner if grasp planning fails (could be None).
		userdata.grasp_occluders = getProbableGraspOccluders(target_detection, other_objects)

		# Store results in user data.
		userdata.object_cloud    = target_detection.point_cloud
		userdata.robot_transform = robot_pose

		crop_box = msg.CropBox2D(box=target_detection.box, crop_inside=Bool(data=False))

		# Skip pose estimation for deformables.
		if isDeformable(userdata.job.object_type):
			filter_response = self.crop_point_cloud(boxes=[crop_box], point_cloud=camera_data.point_cloud)
			self.publish_point_cloud(point_cloud=filter_response.cropped_point_clouds[0], transform=robot_pose)
			return 'success', bin_estimation

		# Get the object pose estimation (and tranformed model cloud).
		estimation = self.estimateObjectPose(target_detection, userdata.job.source_bin, bin_estimation.pose)
		if estimation is None: return 'retry', bin_estimation
		object_pose, object_model_cloud = estimation

		# Check for occlusion.
		occluder = self.checkOcclusion(target_detection, object_model_cloud, other_objects, camera_pose)
		if occluder is not None:
			userdata.occluder = occluder.object.type
			return 'occluded', bin_estimation

		# Publish point cloud, because.... ?
		filter_response = self.filter_objects(object=target_detection.object, pose=object_pose.pose, scene=camera_data.point_cloud, crop_box=crop_box)
		self.publish_point_cloud(point_cloud=filter_response.filtered_scene, transform=robot_pose)

		# Set object pose in world frame.
		userdata.object_pose = tfTransformPose(self.tf, 'world', object_pose)

		return 'success', bin_estimation

	def adjustCameraPose(self, attempt, bin_index, return_trajectory):
		"""
		Adjust the camera pose for a new attempt.
		"""
		camera_offset = adjustedCameraPose(attempt)
		plan          = self.system.planCameraMotion(bin_index, camera_offset)
		if plan is None: return False, None

		if return_trajectory:
			if not self.system.executeStitchedMotion([return_trajectory, plan.camera_trajectory]): return False, plan.reverse_trajectory
		else:
			if not self.system.executeFineMotion(plan.camera_trajectory): return False, plan.reverse_trajectory
		return True, plan.reverse_trajectory

	def moveToBin(self, userdata):
		"""
		Move to the desired bin (from a master pose) with the camera.
		"""
		start  = userdata.current_master_pose
		target = master_pose.cam(userdata.job.source_bin)

		if start == target:
			rospy.loginfo("Already at bin camera pose, not moving robot.")
			return True

		rospy.loginfo("Moving to bin camera pose {}".format(userdata.job.source_bin))
		trajectories = self.system.getCoarseTrajectory(start, target)
		if not trajectories:
			return False

		userdata.current_master_pose = None
		if not self.system.executeStitchedMotion(trajectories, velocity_scaling=HIGH_VELOCITY):
			return False
		userdata.current_master_pose = target
		return True

	def stepRetryTf(self, userdata, bin_estimation):
		"""
		Retry the record step in case it fails due to TF.
		"""
		for tf_retry in range(3):
			try:
				return self.step(userdata, bin_estimation)
			except rospy.ServiceException as e:
				rospy.logerr("Service execution failed: %s", e)
				break
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
				rospy.logwarn("TF lookup/transform failed: %s", e)
				continue
		return 'error', bin_estimation

	def execute(self, userdata):
		userdata.source_bin_pose = None
		userdata.object_pose     = None
		userdata.object_cloud    = None
		userdata.robot_transform = None
		userdata.occluder        = None

		if not self.moveToBin(userdata):
			return 'error'

		return_trajectory = None
		result = 'no_detection'
		bin_estimation = None
		for attempt in range(3):
			result, bin_estimation = self.stepRetryTf(userdata, bin_estimation)
			if result == 'error':
				return 'error'
			if result in ('success', 'occluded'):
				break
			if result == 'retry':
				return 'no_detection'
				#success, return_trajectory = self.adjustCameraPose(attempt, userdata.job.source_bin, return_trajectory)
				#if not success:
				#	return 'error'
			raise Exception('Unknown step result: {}'.format(result))

		# Return to master pose.
		if return_trajectory is not None and not self.system.executeFineMotion(return_trajectory):
			return 'error'
		# Maximum attempts exceeded.
		return result
