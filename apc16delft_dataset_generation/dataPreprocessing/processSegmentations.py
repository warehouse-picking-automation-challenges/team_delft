#!/usr/bin/env python

## processSegmentations
# This script calculates and saves the segmentations of the training data
import os
import sys
import glob
import re
import cv2
import matplotlib.pyplot as plt
from skimage.segmentation import felzenszwalb, quickshift, slic
import numpy as np
import time
from copy import deepcopy
from PATHS import augmentationDir
from dataLib.writeAnnotations import toXML
import random

from processFunctions import storeSegmentation, calcSegmentation, getClass

#import the path settings
from PATHS import originalDir, segmentationDir
from classes import classes

def processSegmentations(classDirName):
	# Get a list of all the class folders in the jpeg_original Directory
	#classDirs = [d for d in os.listdir(originalDir) if os.path.isdir(os.path.join(originalDir, d))]
	classDirs = [classDirName]
	print "classes: ", classDirs
	for imgClass in classDirs:
		print imgClass
		imgClassDir = os.path.join(originalDir, imgClass)
		# batch wise calculation of the segmentations (per angle)
		for angle in ['00', '20', '45', '70', '90']:
			# get a list of all the fileNames (name+extension)
			imgFileNameList = [os.path.basename(imgPath) for imgPath in glob.glob(os.path.join(originalDir,imgClass,'foreground','%s_*.png'%angle))]
			
			segmentationList = calcSegmentation(imgClassDir, imgFileNameList)
			
			# store each segmentation as a file
			for i, segmentation in enumerate(segmentationList):
				segmentationPath = os.path.join(segmentationDir, imgClass, imgFileNameList[i])
				storeSegmentation(segmentation, segmentationPath)
	return


def main(argv):
	print "Start calculating the segmentations"
	print "Directory: ", argv[1]
	processSegmentations(argv[1])
	print "Finished calculating the segmentations"

if __name__ == "__main__":
	main(sys.argv[:])
