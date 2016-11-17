#!/usr/bin/env python

## Integration tests


from __future__ import print_function

PKG = 'apc16delft_place_synthesizer'

import sys
import unittest

import random

import rospy
import rostest

from apc16delft_msgs.msg import *
from apc16delft_msgs.srv import *

class TestTotePlacement(unittest.TestCase):
	
	def __init__(self, *args):
		super(TestTotePlacement, self).__init__(*args)


	def setUp(self):
		rospy.init_node("tests_tote_placement")
		rospy.wait_for_service('/get_place_in_tote')


	# minimum test: request to place the 12 items in an amazon order
	def testa_place_random_item(self):
		test_output = True

		get_tote_place = rospy.ServiceProxy('/get_place_in_tote', GetPlaceInTote)
		req = GetPlaceInToteRequest()
		req.item.type = random.randint(1, 39)
		test_output = get_tote_place(req) and test_output

		return test_output	

	# simple test: request to place the 12 items in an amazon order
	def testb_amazon_task(self):
		test_output = True

		get_tote_place = rospy.ServiceProxy('/get_place_in_tote', GetPlaceInTote)
		req = GetPlaceInToteRequest()

		items = random.sample(range(1, 39), 12)
		for i in items:
			req.item.type = i
			test_output = get_tote_place(req) and test_output

		return test_output


if __name__ == '__main__':
	rostest.rosrun(PKG, 'tests_tote_placement', TestTotePlacement, sys.argv)