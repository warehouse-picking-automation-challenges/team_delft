import rospy
from apc16delft_msgs.objects import objectTypeToString
from apc16delft_msgs.msg import Object

def loadItemBonus(item):
	return rospy.get_param('/item/{}/bonus'.format(objectTypeToString[item]))

def loadItemSize(item):
	return rospy.get_param('/item/{}/size'.format(objectTypeToString[item]))

def loadItemFragility(item):
	return rospy.get_param('/item/{}/fragility'.format(objectTypeToString[item]))

def loadItemWeight(item):
	return rospy.get_param('/item/{}/weight'.format(objectTypeToString[item]))

def loadItemBonuses():
	result = {}
	for type in range(1, 40):
		result[type] = loadItemBonus(type)
	return result

def isDeformable(item):
	return rospy.get_param('item/{}/sample_space'.format(objectTypeToString[item]))[0]['shape'] == 'deformable'

def bypassVacuumCheck(item):
	return item == Object.FITNESS_GEAR_3LB_DUMBBELL or item == Object.ROLODEX_JUMBO_PENCIL_CUP
