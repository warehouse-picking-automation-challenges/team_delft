#!/usr/bin/env python

import cv2
import os
import numpy as np
import xml.etree.ElementTree as ET
import glob
import copy
import sys
import argparse

from classes import getShortName

from dataValidation.captcha import object_selection

# read PATHS from paths directory
from PATHS import testoriginalDir, testimageSetDir, testxmlDir
from dataLib.writeAnnotations import toXML, get_annotation_BBox

class validator_test():

	def __init__(self):
		
		self.originalDir	= testoriginalDir
		self.xmlDir			= testxmlDir
		self.imageSetDir	= testimageSetDir
		
		self.objectSelector = object_selection()

		
		self.windowName = 'Is the bounding box correct? [(y)es/(r)eject/(p)revious/(s)how/(c)lear/(q)uit]'
		cv2.namedWindow(self.windowName,cv2.WINDOW_NORMAL)
		cv2.setMouseCallback(self.windowName,self.draw_BBox)

	def setImage(self,img_path):
		self.imageFileName	= os.path.basename(img_path)
		self.imageName		= os.path.splitext(self.imageFileName)[0]
		
		self.img	= cv2.imread(img_path)
		self.tmpImg = copy.deepcopy(self.img)

		self.width	= self.img.shape[1]
		self.height	= self.img.shape[0]
		
		annotation_path		= os.path.join(self.xmlDir,self.imageName+'.xml')
		self.objs = get_annotation_BBox(annotation_path)
		self.name = ""
		self.xmin = 0
		self.ymin = 0
		self.xmax = 0
		self.ymax = 0
		self.ix = -1
		self.iy = -1
		self.drawing = False

		self.active = True

	def setActive(self, state):
		if state == False:
			self.active = False
			alpha = 0.4
			gamma = 0
			stateMask = np.ones(self.tmpImg.shape,dtype=np.uint8)
			print "Shape: "+ str(stateMask.shape)
			print "Type: " + str(stateMask.dtype)
			print "Shape: "+ str(self.tmpImg.shape)
			print "Type: " + str(self.tmpImg.dtype)
			print "Max: " + str(self.tmpImg.max())
			newIm = cv2.addWeighted(self.tmpImg, alpha, stateMask, 1-alpha,gamma)
			cv2.imshow(self.windowName, newIm)	
		else: # state == True
			self.active = True

	# Mouse callback for drawing a bounding box
	def draw_BBox(self, event, x, y, flags, params):
		if self.active: # this callback is only valid when the window is active
			if event == cv2.EVENT_LBUTTONDOWN:
				if x < self.width and  y < self.height and x > 0 and y > 0:
					self.drawing = True
					self.ix, self.iy = x, y

			elif event == cv2.EVENT_MOUSEMOVE:
				self.tmpImg = copy.deepcopy(self.img)
				if self.drawing == True:
					cv2.rectangle(self.tmpImg,(self.ix,self.iy),(x,y),(0,0,0),2)
					cv2.rectangle(self.tmpImg,(self.ix,self.iy),(x,y),(255,255,255),1)
				else: # if user is not drawing, always show the other objects with their names, also show guide lines
					width	= self.tmpImg.shape[1]
					height	= self.tmpImg.shape[0]
					#draw horizontal and vertical line
					cv2.line(self.tmpImg,pt1=(0,y),pt2=(width,y),color=(0,0,0),thickness=2)
					cv2.line(self.tmpImg,pt1=(x,0),pt2=(x,height),color=(0,0,0),thickness=2)
					cv2.line(self.tmpImg,pt1=(0,y),pt2=(width,y),color=(255,255,255),thickness=1)
					cv2.line(self.tmpImg,pt1=(x,0),pt2=(x,height),color=(255,255,255),thickness=1)
					# draw objects
					#self.draw_Objects()
			elif event == cv2.EVENT_LBUTTONUP:
				if self.drawing:
					self.drawing = False
					cv2.rectangle(self.tmpImg,(self.ix,self.iy),(x,y),(255,255,255),2)
					# find max and min values for x and y and also clip to boundaries of window
					self.xmin = max(min(self.ix, x),0)
					self.ymin = max(min(self.iy, y),0)
					self.xmax = min(max(self.ix, x),self.width)
					self.ymax = min(max(self.iy, y),self.height)

					# only perform next step if the user has actually drawn something
					if self.xmax - self.xmin > 1 and self.ymax - self.ymin > 1:
						self.setActive(False)
						name, obtainable = self.selectName()
						self.setActive(True)
						if name:
							self.objs.append({"name":name, "bbox":(self.xmin,self.ymin,self.xmax-self.xmin,self.ymax-self.ymin), "obtainable":obtainable})
							self.showStatus()
						else: # no object has been selected, update screen 
							self.tmpImg = copy.deepcopy(self.img)
			elif event == cv2.EVENT_RBUTTONUP: # remove the bounding box in which the mouse is located (only if it is unambiguous)
				obj_indx = None
				for i_obj, obj in enumerate(self.objs):
					xb,yb,w,h = obj["bbox"]
					if x > xb and x < xb +w and y > yb and y < yb + h:
						if obj_indx == None:
							obj_indx = i_obj
						else: # More than one bounding box is a candidate
							obj_indx = None
							break
				if not obj_indx == None:
					del self.objs[obj_indx]
				self.tmpImg = copy.deepcopy(self.img)
	
	# Draw the boundingboxes + names of all registered objects
	def draw_Objects(self):
		font = cv2.FONT_HERSHEY_SIMPLEX
		# First draw the black part of the rectangles
		for obj in self.objs:
			x,y,w,h = obj["bbox"]
			cv2.rectangle(self.tmpImg, (x,y), (x+w, y+h), (0,0,0), 2)
		# Then draw the white part
		for obj in self.objs:
			x,y,w,h = obj["bbox"]
			cv2.rectangle(self.tmpImg, (x,y), (x+w, y+h), (0, 255, 0) if obj["obtainable"] else (0, 0, 255), 1)
		# Then draw the text
		for obj in self.objs:
			name = obj["name"]
			x,y,w,h = obj["bbox"]
			shortName = getShortName(name)
			offset_y = +25
			offset_x = +5
			cv2.putText(img=self.tmpImg,text=shortName,org=(x+offset_x,y+offset_y),fontFace=cv2.FONT_HERSHEY_SIMPLEX,fontScale=1,color=(0,0,0),thickness=3,lineType=cv2.CV_AA)
			cv2.putText(img=self.tmpImg,text=shortName,org=(x+offset_x,y+offset_y),fontFace=cv2.FONT_HERSHEY_SIMPLEX,fontScale=1,color=(255,255,255),thickness=2,lineType=cv2.CV_AA)

	def validate(self):
		self.showStatus()
		while True:
			if self.drawing == False:
				self.draw_Objects()
			cv2.imshow(self.windowName,self.tmpImg)
			k = cv2.waitKey(1) & 0xFF
			if k == ord('c'):
				self.objs = []
				self.name = None
				self.xmin = 0
				self.ymin = 0
				self.xmax = 0
				self.ymax = 0
				self.tmpImg = copy.deepcopy(self.img)
			elif k == ord('z'):
				self.undoBbox()
			elif k == ord('a'):
				toXML(self.xmlDir,self.img.shape,self.imageFileName,self.objs)
				return 'accept'
			elif k == ord('p'):
				return 'previous'
			elif k == ord('r'):
				return 'reject'
			elif k == ord('q'):
				return 'quit'

		return "accepted"

	def undoBbox(self):
		# Remove last bounding box
		if len(self.objs)>0:
			del self.objs[-1]
		self.tmpImg = copy.deepcopy(self.img)
		self.draw_Objects()

	def selectName(self):
		name, obtainable = self.objectSelector.select()
		if len(name):
			return name[0], obtainable
		else:
			return None, obtainable

	def showStatus(self):
		print "Status:"
		for obj in self.objs:
			print(obj["name"],obj["bbox"])

	def getValImg(self):
		return self.validatedImageNames



