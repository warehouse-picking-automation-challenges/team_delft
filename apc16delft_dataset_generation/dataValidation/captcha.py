#!/usr/bin/env python
import cv2
import os
from classes import inverse_classes_short, inverse_classes, getShortName, getLongName
import numpy as np
import sys

class object_selection():
	"""A pop-out object selection method.
	"""
	def __init__(self, multi=False):
		img_path = self.parse_imgpath()
		self.img = cv2.imread(img_path)
		self.num_x = 6                      # divide into 6 in width
		self.num_y = 7                      # divide into 7 in height
		self.w = self.img.shape[1] / self.num_x
		self.h = self.img.shape[0] / self.num_y
		self.interface = self.img.copy()
		self.winName = 'Select object and press (o)btainable or (u)nobtainable.'
		self.grids = self.get_grids()
		self.objects = self.get_objects()
		self.objects_selected = []
		self.grids_selected = []
		self.objects_candidates = [obj for row in self.objects for obj in row if obj != None]
		self.grids_candidates = [grid for row in self.grids for grid in row if grid not in [self.grids[6][2], self.grids[6][3], self.grids[6][4]]]
		self.active = False
		self.multiSelect = multi
	
	def parse_imgpath(self):
		cwdpath = os.getcwd()
		pathsplit = cwdpath.split('/')
		parentpaths = pathsplit[0:pathsplit.index('team_delft')+1]
		parentpath = ''
		for elem in parentpaths:
			parentpath = parentpath + elem + '/'
		imgpath = parentpath + 'apc16delft_dataset_generation/dataValidation/captcha.jpg'
		print imgpath
		return imgpath



	def setActive(self, state):
		cv2.namedWindow(self.winName,cv2.WINDOW_AUTOSIZE)
		if state == False:
			self.active = False
			alpha = 0.3
			gamma = 0
			stateMask = np.ones(self.img.shape,dtype=np.uint8)
			#print "Shape: "+ str(stateMask.shape)
			#print "Type: " + str(stateMask.dtype)
			#print "Shape: "+ str(self.img.shape)
			#print "Type: " + str(self.img.dtype)
			#print "Max: " + str(self.img.max())
			newIm = cv2.addWeighted(self.img, alpha, stateMask, 1-alpha,gamma)
			cv2.imshow(self.winName, newIm)
		else: # state == True
			cv2.imshow(self.winName, self.img)
			self.active = True

	def get_grids(self):
		"""get grid coordinates and return an array of 7 * 6 (xmin,ymin,xmax,ymax) boxes"""
		grids = []
		for i in xrange(0, self.num_y):
			grids.append([])
			for j in xrange(0, self.num_x):
				xmin = j * self.w
				ymin = i * self.h
				xmax = xmin + self.w
				ymax = ymin + self.h
				grids[i].append((xmin,ymin,xmax,ymax))
		return grids

	def get_objects(self):
		"""get object names and return an array of 7 * 6 object names"""
		index = [[38, 6, 31, 22, 5, 24], \
				[11, 7, 36, 12, 14, 15], \
				[33, 26, 37, 3, 27, 10], \
				[16, 8, 39, 2, 1, 34,], \
				[18, 32, 30, 35, 28, 9], \
				[17, 25, 13, 19, 29, 21], \
				[4, 23, 0, 0, 0, 20 ]]
		objects = []
		for i in xrange(0, self.num_y):
			objects.append([])
			for j in xrange(0, self.num_x):
				idx = index[i][j]
				if idx:
					obj = inverse_classes[index[i][j]]
					objects[i].append(obj)
				else:
					objects[i].append(None)
		return objects


	def object_parser(self, x, y, grids, objects):
		i = y / self.h
		j = x / self.w
		if 0 <= i < self.num_y and 0 <= j < self.num_x:
			return grids[i][j], objects[i][j]
		else:
			return None, None	

	def update_interface(self, obj, mode='add'):
		""""update menu interface with three modes
		'add': add a rectangle (default: can be used as show_interface)
		'remove': remove a rectangle
		'show_cand': show the available candates to select"""
		self.interface = self.img.copy()
		textcolor = (0,0,0)
		boxcolor = (0,255,255)
		font = cv2.FONT_HERSHEY_SIMPLEX

		# show available candidates for single target selection
		if not self.multiSelect:
			for grid in self.grids_candidates:
				cv2.rectangle(self.interface, (grid[0],grid[1]), (grid[2],grid[3]), (0,255,0), 5)

		if mode == 'remove':
			textcolor = (0,0,255)

		for grid in self.grids_selected:
			# selected box
			cv2.rectangle(self.interface, (grid[0],grid[1]), (grid[2],grid[3]), boxcolor, 5)
			# selected obj
			cv2.putText(self.interface, obj, (self.grids[6][2][0], self.grids[6][2][3]-20), font, 1, textcolor, 2)
			# duplicated or not
			if self.grids_selected.count(grid) > 1:
				cv2.putText(self.interface, 'x2', (grid[0], grid[3]-20), font, 1, textcolor, 2)




	def valid_selection(self, obj, reg):
		"check validity of selection"
		# clean the selected history if only requires one single object
		if not self.multiSelect:
			self.objects_selected = []
			self.grids_selected = []
			if len(self.objects_candidates) == len(self.grids_candidates):
				if obj in self.objects_candidates and reg in self.grids_candidates:
					return True
			else:
				print self.objects_candidates
				print self.grids_candidates
				print 'Error: objects_candidates and grids_candidates do not match.'
				sys.exit()
		elif self.multiSelect:
			if len(self.objects_selected) == len(self.grids_selected):
				if obj not in self.objects_selected and reg not in self.grids_selected:
					return True
			else:
				print 'Error: objects_selected and grids_selected do not match.'
				sys.exit()



	def click(self, event, x, y, flags, param):
		if self.active == True: # callback only active when the window is active
			# left click to select
			if event == cv2.EVENT_LBUTTONDOWN:
				# parse selected object and corresponding region
				reg, obj = self.object_parser(x, y, self.grids, self.objects)
				if reg != None and obj != None:
					# add object selected and show on interface
					if self.valid_selection(obj, reg):
						self.objects_selected.append(obj)
						self.grids_selected.append(reg)
						self.update_interface(getShortName(obj), mode='add')

			# double click to remove for multiseletion
			if event == cv2.EVENT_LBUTTONDBLCLK and self.multiSelect:
				# parse object to remove and its corresponding region
				reg, obj = self.object_parser(x, y, self.grids, self.objects)
				if reg != None and obj != None:
					# remove the object from list and update interface
					if obj in self.objects_selected and reg in self.grids_selected:
						self.objects_selected = filter(lambda x: x!=obj, self.objects_selected)
						self.grids_selected = filter(lambda x: x!=reg, self.grids_selected)
						self.update_interface(getShortName(obj), mode='remove')

			# right click to add duplicated items
			if event == cv2.EVENT_RBUTTONDOWN and self.multiSelect:
				# parse selected object and corresponding region
				reg, obj = self.object_parser(x, y, self.grids, self.objects)
				if reg != None and obj != None:
					# add object selected and show on interface
					if self.valid_selection(obj, reg):
						self.objects_selected.append(obj)
						self.objects_selected.append(obj)
						self.grids_selected.append(reg)
						self.grids_selected.append(reg)
						self.update_interface(getShortName(obj), mode='add')

			#cv2.imshow(self.winName, self.interface)

	def select(self):
		self.setActive(True)
		#self.update_interface('Selection Display')

		cv2.setMouseCallback(self.winName, self.click)

		# flag which determines if the robot can grasp the object
		obtainable = True
		while True:
			cv2.imshow(self.winName, self.interface)
			key = cv2.waitKey(1) & 0xFF
			if (key == ord('o') or key == ord('u')) and self.objects_selected != []: # user presses 'o'btainable or 'u'nobtainable
				obtainable = key == ord('o')
				break
			elif (key == 32 or key == 10) and self.objects_selected != []:
				self.destroyWindow()
				return self.objects_selected
			elif key == 27: # user presses escape
				self.destroyWindow()
				self.objects_selected = []
				return self.objects_selected

		self.setActive(False)
		return self.objects_selected, obtainable
	
	def find_2dindex(self, srclist, target):
		for i, row in enumerate(srclist):
			for j, elem in enumerate(row):
				if elem == target:
					return (i, j)
		return None

	def read(self, objectsin):
		self.objects_candidates = objectsin
		inds = [self.find_2dindex(self.objects, obj) for obj in objectsin]
		self.grids_candidates = [self.grids[ind[0]][ind[1]] for ind in inds if ind != None]
		if self.multiSelect:
			self.objects_selected = self.objects_candidates
			self.grids_selected = self.grids_candidates



	def destroyWindow(self):
		cv2.destroyWindow(self.winName)
