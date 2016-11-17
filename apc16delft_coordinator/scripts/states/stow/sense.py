import rospy
import smach

import tf
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

import std_msgs.msg
from std_msgs.msg import Bool, Header
from std_srvs.srv import Empty, EmptyRequest

from apc16delft_msgs import srv, msg
from apc16delft_msgs.objects import objectTypeToString

from lib.services import ServiceState, ServiceProxy as DumpingServiceProxy
from lib import geometry_numpy
from lib.objects import isDeformable

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

def getBinPoseGuess(index):
	return geometry_numpy.paramToPose(rospy.get_param("/bin/{}/pose".format(index)))

class SenseInTote(smach.State):
	def __init__(self, planner, tf, system):
		self.planner      = planner
		self.tf           = tf
		self.system       = system

		smach.State.__init__(self,
			outcomes     = ['success', 'no_detection', 'no_object_pose', 'error'],
			input_keys   = ['job', 'tote_contents', 'blacklist'],
			output_keys  = ['tote_pose', 'object_type', 'object_cloud', 'object_pose']
		)

		self.get_camera_data          = rospy.ServiceProxy('/tote_camera/get_data',                                  srv.GetCameraData)
		self.detect_objects           = DumpingServiceProxy('/tote_detection/detect_objects',                        srv.DetectObjects, dump_request=False)
		self.estimate_object_pose     = DumpingServiceProxy('/pose_estimation/estimate_object_pose',                 srv.EstimateObjectPose)
		self.crop_point_cloud         = rospy.ServiceProxy('/point_cloud_crop/crop_point_cloud',                     srv.CropPointCloud)
		self.publish_point_cloud      = rospy.ServiceProxy('/static_pointcloud_publisher/publish_point_cloud',       srv.PublishStaticPointCloud)
		self.clear_static_point_cloud = DumpingServiceProxy('/static_pointcloud_publisher/clear_static_point_cloud', Empty)
		self.clear_octomap            = DumpingServiceProxy('/clear_octomap',                                        Empty)
		self.filter_objects           = rospy.ServiceProxy('/point_cloud_crop/filter_object',                        srv.FilterObject)

	def retrieveCameraData(self, dump = True, publish = True):
		"""
		Record camera data.
		"""
		self.clear_static_point_cloud(EmptyRequest())
		self.clear_octomap(EmptyRequest())

		for retry in range(2):
			try:
				return self.get_camera_data(dump=dump, publish=publish)
			except rospy.ServiceException as e:
				rospy.logwarn('Failed to retrieve data from tote camera: {}'.format(e))
		return self.get_camera_data(dump=dump, publish=publish)


	def estimateObjectPose(self, target_object, container_index, container_pose):
		"""
		Estimate the pose of an object.
		Returns a (stamped pose, transformed model cloud) tuple, or None if the estimation fails.
		Also returns None if the estimation error is too high.
		"""
		try:
			object_estimation = self.estimate_object_pose(
				object          = target_object,
				container_index = container_index,
				container_pose  = container_pose
			)
		except rospy.ServiceException as e:
			rospy.logwarn('Failed to estimate object pose: {}'.format(e))
			return None

		# TODO: What limit?
		if object_estimation.estimation_error > 0.2:
			rospy.logwarn('Estimated object pose is too unreliable, estimation error is {}.'.format(object_estimation.estimation_error))
			return None

		return object_estimation.pose, object_estimation.transformed_model

	def step(self, userdata):
		# Record data and poses.
		camera_data     = self.retrieveCameraData(dump = True)
		header          = camera_data.point_cloud.header
		time            = header.stamp

		tote_pose  = PoseStamped(header = header, pose = tfLookup(self.tf, 'world', 'tote', time))

		detection = self.detect_objects(
			color       = camera_data.color,
			point_cloud = camera_data.point_cloud,
			objects     = userdata.tote_contents,
			bin_pose    = tote_pose.pose,
			bin_index   = -1 # TODO: Use a constant
		)

		blacklist = list(userdata.blacklist)
		blacklist.append(msg.Object.ROLODEX_JUMBO_PENCIL_CUP)
		blacklist.append(msg.Object.FITNESS_GEAR_3LB_DUMBBELL)
		detection.objects = filter(lambda x: x.object.type not in blacklist, detection.objects)
		rospy.loginfo("Found objects: %s", ', '.join([objectTypeToString[x.object.type] for x in detection.objects]))

		# Check if any objects were found.
		if not detection.objects:
			return 'no_detection'

		target_detection = max(detection.objects, key = lambda x: x.centroid.z)
		crop_box = msg.CropBox2D(box=target_detection.box, crop_inside=Bool(data=False))

		pinch_items = [msg.Object.FITNESS_GEAR_3LB_DUMBBELL, msg.Object.ROLODEX_JUMBO_PENCIL_CUP]
		if target_detection.object.type in pinch_items:
			# Check tote contents, if it doesn't contain any items other than one of te pinch items, continue, otherwise return no detection
			if any(x in pinch_items for x in userdata.tote_contents):
				return 'no_object_pose'

		# Set object pose in world frame.
		userdata.tote_pose       = tote_pose
		userdata.object_type     = target_detection.object.type
		userdata.object_cloud    = target_detection.point_cloud
		self.planner.selectTargetBin(target_detection.object.type)

		identity_pose = Pose()
		identity_pose.orientation.w = 1

		if isDeformable(target_detection.object.type):
			filter_response = self.crop_point_cloud(boxes=[crop_box], point_cloud=camera_data.point_cloud)
			self.publish_point_cloud(point_cloud=filter_response.cropped_point_clouds[0], transform=identity_pose)
			return 'success'

		# Get the object pose estimation (and tranformed model cloud).
		estimation = self.estimateObjectPose(target_detection, -1, tote_pose.pose) # TODO: Use a constant for -1.
		if estimation is None:
			return 'no_object_pose'
		object_pose, object_model_cloud = estimation

		# Set object pose in world frame.
		userdata.tote_pose       = tote_pose
		userdata.object_pose     = tfTransformPose(self.tf, 'world', object_pose)

		# Publish point cloud, because.... ?
		filter_response = self.filter_objects(object=target_detection.object, pose=object_pose.pose, scene=camera_data.point_cloud, crop_box=crop_box)
		self.publish_point_cloud(point_cloud=filter_response.filtered_scene, transform=identity_pose)

		return 'success'

	def stepRetryTf(self, userdata):
		"""
		Retry the record step in case it fails due to TF.
		"""
		for tf_retry in range(3):
			try:
				return self.step(userdata)
			except rospy.ServiceException as e:
				rospy.logerr("Service execution failed: %s", e)
				break
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
				rospy.logwarn("TF lookup/transform failed: %s", e)
				continue
		return 'error'


	def execute(self, userdata):
		userdata.tote_pose       = None
		userdata.object_type     = None
		userdata.object_cloud    = None
		userdata.object_pose     = None
		return self.stepRetryTf(userdata)

class SenseTargetBin(smach.State):
	def __init__(self, tf, system):
		self.tf           = tf
		self.system       = system

		smach.State.__init__(self,
			outcomes     = ['success', 'no_detection', 'error'],
			input_keys   = ['job'],
			output_keys  = ['target_bin_pose']
		)

	def step(self, userdata):
		time = rospy.Time.now()
		shelf_pose = tfLookup(self.tf, 'world', 'shelf', time)
		target_bin_pose   = geometry_numpy.poseToMatrix(shelf_pose) * geometry_numpy.poseToMatrix(getBinPoseGuess(userdata.job.target_bin))
		target_bin_pose   = geometry_numpy.poseFromMatrix(target_bin_pose)
		target_bin_pose   = PoseStamped(header = std_msgs.msg.Header(frame_id = "world", stamp = time), pose = target_bin_pose)

		# Set object pose in world frame.
		userdata.target_bin_pose = target_bin_pose

		return 'success'

	def stepRetryTf(self, userdata):
		"""
		Retry the record step in case it fails due to TF.
		"""
		for tf_retry in range(3):
			try:
				return self.step(userdata)
			except rospy.ServiceException as e:
				rospy.logerr("Service execution failed: %s", e)
				break
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
				rospy.logwarn("TF lookup/transform failed: %s", e)
				continue
		return 'error'

	def execute(self, userdata):
		userdata.target_bin_pose = None
		return self.stepRetryTf(userdata)
