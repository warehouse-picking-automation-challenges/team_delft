#!/usr/bin/env python

import rospy
import roslib

import json

from operator import itemgetter
from apc16delft_msgs.objects import objectTypeToString
from apc16delft_msgs.srv import *
from geometry_msgs.msg import Pose

items = {}
tote_state = []

##############################
def main():
	rospy.init_node('get_place_in_tote')
	srv = rospy.Service('get_place_in_tote', GetPlaceInTote, get_place_cb)
	global items
	items = rospy.get_param( "/item/" )

	global tote_state 
	volumes_x_ = rospy.get_param( "/tote/volumes/x",  3)
	volumes_y_ = rospy.get_param( "/tote/volumes/y",  2)

	tote_state = [[[] for x in range(volumes_x_)] for y in range(volumes_y_)]

	rospy.spin()
	

##############################
# SERVICE CALLBACK
# returns: a Pose in world coordinates
##############################
def get_place_cb(req):
	global items
	item = objectTypeToString[req.item.type]

	# decide where to place the item
	free_places = get_available_places(item)

	resp = GetPlaceInToteResponse()

	if not free_places:
		resp.error = 1
		resp.message = 'No place available in tote'
		resp.place_pose = ''
		return resp

	(i, j) = select_place(item, free_places)

	# update tote state with the item
	tote_state[i][j].append(item)

	printToteState()

	resp.error = 0
	resp.message = 'available place pose retrieved'
	resp.place_pose = 'tote'+str(i)+str(j)
	return resp


##############################
# for heavy items returns only empty places
def get_available_places(item):
	places = []
	global tote_state
	global items
	for i in range( len(tote_state) ) :
		for j in range( len(tote_state[0]) ) :

			if len( tote_state[i][j] ) == 0 :
				places.append((i, j))

			# for heavy items returns only empty places
			elif  items[item]['weight'] == 'high' :
				continue

			# for large items returns only empty places
			elif  items[item]['size'] == 'large' :
				continue

			# do not include places that are full of items
			elif tote_place_complete(tote_state[i][j]) :
				continue

			# if
			elif items[ tote_state[i][j][-1] ]['fragile'] and items[item]['weight']!= 'low' :
				continue

			else:
				places.append((i, j))
	print places
	return places


##############################
# - item is a string with the name of the item to place
# - places: is a list of tuples indexing tote volumes
# - returns the indices of the best tote volume
def select_place(item, places):
	global items

	if not places:
		return None

	# return just the first place unless aditional logic is added below
	min_items_place = max_items_place  = places[0]

	# find the available places with more and less items
	min_items = max_items = len( tote_state[0][0] )
	for p in places:

		if len( tote_state[ p[0] ][ p[1] ] ) > max_items :
			m = len( tote_state[ p[0] ][ p[1] ] )
			max_items_place = p

		if len( tote_state[ p[0] ][ p[1] ] ) < min_items :
			min_items = len( tote_state[ p[0] ][ p[1] ] )
			min_items_place = p


	# put fragile items on top of the place with more items
	# if not fragile select the place with less items
	if items[item]['fragile'] :	
		return max_items_place
	else:
		return min_items_place


##############################
# - contents: is a list of string, corresponding to the current items in a tote volume
# 
# Logic: a place is complete if  
# there is a large item
# OR 2 or more medium items
# OR more than 3 small items
# OR 1 medium and 2 small
def tote_place_complete(contents):
	sizes = [ items[item]['size'] for item in contents ]
	if 'large' in sizes :
		return True
	elif sizes.count('medium') > 1 :
		return True
	elif sizes.count('small') > 3 : 
		return True
	elif sizes.count('medium') == 1 and sizes.count('small') > 2 :
		return True
	else:
		return False


## nice screen output ##############################
def printToteState():
	global tote_state

	print "\n Tote contents:\n --------------\n"
	for i in range( len(tote_state) ) :
		for j in range( len(tote_state[0]) ) :
			print "(%d, %d): %s" % (i, j, tote_state[i][j])
	print

	print "\n Tote fulfilment:\n ----------------\n"
	for i in range( len(tote_state) ) :
		for j in range( len(tote_state[0]) ) :
			print "|",
			for c in tote_state[i][j] :
				print '#',
			for c in range(5-len(tote_state[i][j])):
				print " ",
		
		print '|'

		for j in range( len(tote_state[0]) ) :
			print " ",
			for c in tote_state[i][j] :
				print '_',
			for c in range(5-len(tote_state[i][j])):
				print '_',
		print ' '


############### MAIN ######################################
if __name__ == '__main__':	
	main()

