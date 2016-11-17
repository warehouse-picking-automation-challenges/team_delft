#!/usr/bin/env python
import os
import sys
import glob
import re
import copy
import cv2
import matplotlib.pyplot as plt
from skimage.segmentation import felzenszwalb, quickshift, slic
import numpy as np
import time
from copy import deepcopy
from dataLib.writeAnnotations import toXML
##
from classes import inverse_classes

## WARNING: DO NOT READ ANYTHING FROM THE PATHS,
## correct paths are retrieved from the master validator through the slave validator and
## depend on the mode of the validation process ('traindata' or 'testdata')

def getBoundingBox(img, margin=0):
	## Takes an image as argument and returns the x,y,w,h parameters of the bounding box that has been found
# 1. segment image
# 2. find bounding box
# 3. return

	#print 'enter getBoundingBox'

	mask = img

	points = cv2.findNonZero(mask)
	if not points == None:
		boundingBox = cv2.boundingRect(points)
		#boundingBox = (x - margin, y - margin, w + margin * 2, h + margin * 2)
	else:
		boundingBox = 0,0,0,0
	
	return boundingBox

def storeImage(img, mask, imageDir,  imageFileName):
	## Places a random background behind the masked imaged and stores it
	fname = os.path.append(imageDir, imageFileName)
	print fname
	cv2.imwrite(fname, img);

def calcSegmentation(imgClassDir, imgFileNameList):
# imgClassDir is complete path to directory of the class folder (in the Original folder)
# imgFileName as list of [name+extension] or just name+extension if for one image the segmentation needs to be calculated
# if imgFileName is a list (of images of the same angle) then also output a list
	
	_isList = True
	# place imgFileNameList in a list if only one filename is given
	if not isinstance(imgFileNameList, list):
		imgFileNameList = [imgFileNameList]
		_isList = False

	print imgClassDir
	#print imgFileNameList

	# angle MUST be the same for all images in the list, initialize with the angle of the first element in the list of imgFileNames
	angle		= getAngle(imgFileNameList[0])
	
	## check backgroundcolor and perform for each one
	backGroundColors = []
	for imageFilename in imgFileNameList:
		bgColor = imageFilename.split('_')[1] # filename like angle_backgroundcolor_restOfFilename, get the backgroundcolor
		if not bgColor in backGroundColors:
			backGroundColors.extend(bgColor)
	print "BacgroundColors: "+ str(backGroundColors)

	# some objects are recorded on multiple backgrounds
	for bg in backGroundColors:
		## First get all background images
		backgroundDir	= os.path.join(imgClassDir,'background')
		background_Images = [f for f in os.listdir(backgroundDir)
				if re.match("{}_{}_*".format(angle,bg),f)]
		
		print "{}_{}_*".format(angle,bg)
		#print background_Images
		
		## Add all background images to the backgroundsubstractor
		
		## TODO: this hould be per object/class
		varThresh = {"b":20, "w":150, "g":10, "r":100 }
		print "Threshold: " + str(varThresh[bg])
		n_history = len(background_Images)
		print "History length: "+str(n_history)
		bs = cv2.BackgroundSubtractorMOG2(history = n_history, varThreshold = varThresh[bg])

		for img in background_Images:
			background = cv2.imread(os.path.join(backgroundDir,img))
			background = cv2.cvtColor(background,cv2.COLOR_BGR2HSV)
			# channel selection
			if bg =='b' or bg == 'w' or bg == 'g':
				background_channel = background[:,:,0] # use value
			else:
				background_channel = background[:,:,2] # use hue
			
			bs.apply(copy.deepcopy(background_channel))
	
		segmentationList = []
		foreground_Images = [f for f in imgFileNameList if re.match("{}_{}_*".format(angle,bg),f)]
		for imgFileName in foreground_Images:
			foregroundDir	= os.path.join(imgClassDir,'foreground')
			imgPath			= os.path.join(foregroundDir,imgFileName)
		
			foreground		= cv2.imread(imgPath)
			foreground		= cv2.cvtColor(foreground, cv2.COLOR_BGR2HSV)
			# channel selection
			if bg =='b' or bg == 'w' or bg == 'g':
				foreground_channel = foreground[:,:,0] # use value
			else:
				foreground_channel = foreground[:,:,2] # use hue

			bscpy			= bs # make a copy of the backgroundsubstractor?
			segmentation	= bscpy.apply(foreground_channel)
			mask = segmentation >= 50
			#mask = segmentation == 255
			mask = mask * 255
			segmentationList.append(copy.deepcopy(mask))
	
	if _isList:
		# return a list of segmentations (possiblye with only one element)
		return segmentationList
	else:
		# return not a list, but only one segmentation (if only one filename is given as argument)
		return segmentationList[0]

def storeSegmentation(segmentation, segmentationPath):
	# check if path exists, if not then create the directory for the specific class
	if not os.path.exists(os.path.dirname(segmentationPath)):
		print "Creating directory:\'"+os.path.dirname(segmentationPath)+"\'"
		os.mkdir(os.path.dirname(segmentationPath))
	cv2.imwrite(segmentationPath, segmentation)
	
def loadSegmentation(segmentationPath):
	segmentation = cv2.imread(segmentationPath,cv2.IMREAD_GRAYSCALE)
	return segmentation

def getAngle(imgFileName):
	# imgFileName as name+extension
	imgName = os.path.splitext(imgFileName)[0]
	angle = imgName[0:2] # not returned as int, but as string, because '00' is also an angle
	return angle

def getClass(imgPath):
	## imgFileName as complete path to image
	imgClass	= imgPath.split(os.sep)[-3]
	return imgClass

