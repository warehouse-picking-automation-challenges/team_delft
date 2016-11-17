import tf
import numpy
from geometry_msgs import msg

def quaternionToMatrix(q):
	return numpy.matrix(tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w]))

def translationToMatrix(v):
	return numpy.matrix(tf.transformations.translation_matrix([v.x, v.y, v.z]))

def poseToMatrix(pose):
	return translationToMatrix(pose.position) * quaternionToMatrix(pose.orientation)

def vector3FromList(v):
	return msg.Vector3(x = float(v[0]), y = float(v[1]), z = float(v[2]))

def pointFromList(v):
	return msg.Point(x = float(v[0]), y = float(v[1]), z = float(v[2]))

def quaternionFromList(q):
	return msg.Quaternion(x = float(q[0]), y = float(q[1]), z = float(q[2]), w = float(q[3]))

def quaternionFromMatrix(m):
	q = tf.transformations.quaternion_from_matrix(m)
	return quaternionFromList(q)

def poseFromMatrix(m):
	translation = m[numpy.ix_(range(3), [3])]
	orientation = quaternionFromMatrix(m)
	return msg.Pose(position = pointFromList(translation), orientation = orientation)

def paramToPoint(param):
	return msg.Point(param['x'], param['y'], param['z'])

def paramToQuaternion(param):
	return msg.Quaternion(param['x'], param['y'], param['z'], param['w'])

def paramToPose(param):
	return msg.Pose(position = paramToPoint(param['position']), orientation = paramToQuaternion(param['orientation']))
