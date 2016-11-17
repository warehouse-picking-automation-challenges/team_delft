#!/usr/bin/env python

import cv2
import os
import numpy as np
import xml.etree.ElementTree as ET
import glob
import copy
import sys
import argparse

#import the path settings
from PATHS import jpegOriginalDir, jpegDir, xmlDir, imageSetDir, libDir, class2Index
from PATHS import testOriginalDir, testjpegDir, testxmlDir, testimageSetDir

sys.path.append(libDir)
from writeAnnotations import toXML

class dataValidation():

	def __init__(self,imgPath,xmlDir,validated_imageNames):

		self.imageFile = imgPath
		print self.imageFile
		self.image_file_name = os.path.basename(self.imageFile)
		self.imageName = os.path.splitext(self.image_file_name)[0]
		self.xmlDir = xmlDir
		self.img = cv2.imread(self.imageFile)
		self.tmpImg = None

		self.objs = []
		self.name = ""
		self.xmin = 0
		self.ymin = 0
		self.xmax = 0
		self.ymax = 0
		self.ix = -1
		self.iy = -1
		self.iw = 0
		self.ih = 0
		self.drawing = False

	def annotation_BBox(self):
		file_annotation = str(self.xmlDir)+str(self.imageName)+'.xml'
		if DEBUG:
			print('Read annotation: ' + file_annotation)

		if os.path.exists(file_annotation):
			tree = ET.parse(file_annotation)
			root = tree.getroot()
			for obj in root.iter("object"):
				self.name = obj.find('name').text
				bbox = obj.find('bndbox')
				self.xmin = int(bbox.find('xmin').text)
				self.ymin = int(bbox.find('ymin').text)
				self.xmax = int(bbox.find('xmax').text)
				self.ymax = int(bbox.find('ymax').text)
				self.objs.append({"name":self.name,"bbox":(self.xmin,self.ymin,self.xmax-self.xmin,self.ymax-self.ymin)})
		else:
			print file_annotation + " not found"


	def draw_BBox(self, event, x, y, flags, params):

		if event == cv2.EVENT_LBUTTONDOWN:
			self.drawing = True
			self.ix, self.iy = x, y

		elif event == cv2.EVENT_MOUSEMOVE:
			if self.drawing == True:
				self.tmpImg = copy.deepcopy(self.img)
				cv2.rectangle(self.tmpImg,(self.ix,self.iy),(x,y),(0,255,0),2)

		elif event == cv2.EVENT_LBUTTONUP:
			self.drawing = False
			self.iw = x-self.ix
			self.ih = y-self.iy
			cv2.rectangle(self.tmpImg,(self.ix,self.iy),(x,y),(0,255,0),2)
			self.xmin = min(self.ix, x)
			self.ymin = min(self.iy, y)
			self.xmax = max(self.ix, x)
			self.ymax = max(self.iy, y)
			selected = self.selectName()
			if selected:
				self.objs.append({"name":self.name, "bbox":(self.xmin,self.ymin,self.xmax-self.xmin,self.ymax-self.ymin)})
				self.showStatus()

	def manual_adjustment(self,mode):
		self.tmpImg = copy.deepcopy(self.img)
		font = cv2.FONT_HERSHEY_SIMPLEX
		for obj in self.objs:
			name = obj["name"]
			x,y,w,h = obj["bbox"]
			cv2.putText(self.tmpImg,name,(x,y), 1, 4,(255,255,255),2)
			cv2.rectangle(self.tmpImg, (x,y), (x+w, y+h), (255,0,0), 2)

		windowName = 'Is the bounding box correct? [(y)es/(r)eject/(p)revious/(s)how/(c)lear/(q)uit]'
		cv2.namedWindow(windowName,cv2.WINDOW_AUTOSIZE)
		cv2.setMouseCallback(windowName,self.draw_BBox)

		while(1):
			cv2.imshow(windowName,self.tmpImg)
			k = cv2.waitKey(1) & 0xFF
			if k == ord('c'):
				self.objs = []
				self.name = None
				self.xmin = 0
				self.ymin = 0
				self.xmax = 0
				self.ymax = 0
				self.manual_adjustment(mode)
				break
			elif k == ord('n'):
				self.manual_adjustment(mode)
				break
			elif k == ord('y'):
				print "xmlDir:"
				print self.xmlDir
				toXML(self.xmlDir,self.img.shape,self.image_file_name,self.objs)
				self.showStatus()
				break
			elif k == ord('p'):
				return "previous"
			elif k == ord('r'):
				print self.imageName + " rejected."
				return "rejected"
			elif k == ord('q'):
				sys.exit()

		return "accepted"

	def selectName(self):
		inverse_class2Index = {v: k for k, v in class2Index.items()}
		print "Please select object name ", inverse_class2Index
		while True:
			key = cv2.waitKey(33) & 0xFF

			if key == 27:
				print "Bounding box cancelled"
				return False

			label = key - 48
			if label > 0 and label <= len(class2Index):
				try:
					self.name = inverse_class2Index[key - 48]
					return True
				except ValueError,KeyError:
					print "Invalid key pressed: ", key

		if not self.name == None:
			print "Bounding box for " + self.name + " was selected"

	def showStatus(self):
		print "status:"
		for obj in self.objs:

			print(obj["name"],obj["bbox"])

	def getValImg(self):
		return self.validatedImageNames


def readAllImages(image_path):
	image_paths = glob.glob(image_path)
	if DEBUG:
		print('Read PNG images from: ' + image_path)
	return image_paths


def readOnlyInvalidImages(image_path, validate_filename, rejected_filename):
	image_paths = readAllImages(image_path)
	with open(validate_filename,'a+') as vf:
		validated_images = vf.read().splitlines()
	with open(rejected_filename,'a+') as rf:
		rejected_images = rf.read().splitlines()

	validated_paths = [os.path.join(os.path.dirname(image_path),validated_image+'.jpg') for validated_image in validated_images] #TODO jpg or png
	rejected_paths = [os.path.join(os.path.dirname(image_path),rejected_image+'.jpg') for rejected_image in rejected_images] #TODO jpg or png

	image_paths = [img for img in image_paths if not img in validated_paths and not img in rejected_paths]
	print('%d images to be validated'%len(image_paths))
	return image_paths

if __name__ == "__main__":
	# parse arguments
	parser = argparse.ArgumentParser()
	parser.add_argument("--add", help="Usage: python main.py --add manual OR --add test (to validate rejected images manually OR to draw Ground Truth BBox for APC test set)")
	args = parser.parse_args()


	DEBUG = True
	image_path = jpegDir+'*.jpg' #TODO jpg or png
	annotation_path = xmlDir
	validate_filename = imageSetDir+'validated.txt'
	rejected_filename = imageSetDir+'rejected.txt'
	validate_all = True

	mode = 'train'
	if args.add == 'test':
		mode = 'test'
		image_path = testjpegDir+'*.jpg' #TODO jpg or png
		validate_filename = testimageSetDir+'validated.txt'
		annotation_path = testxmlDir
		print('Start to annotate test images')
	elif args.add == 'all':
		validate_all = True
		print('Start to validate all images')
	else:
		validate_all = False
		print('Start to validate invalid images (or images that are not yet validated at all)')

	# Read list of images to be validated
	if validate_all:
		# Empty validate_file
		with open(validate_filename,'w') as file: pass # open and close the file

	image_paths = readOnlyInvalidImages(image_path,validate_filename, rejected_filename)

	if DEBUG:
		print ('Validated image names are saved to: ' + validate_filename)

	validated_imageNames = []
	rejected_imageNames = []
	#lines = vf.read().splitlines()
	#validated_imageNames = lines

	i = 0
	while len(image_paths) > 0:
		print "Image path"
		print image_paths[0]
		img_Path = image_paths[0]
		imgName = os.path.splitext(os.path.basename(img_Path))[0]
		#print imgName
		# create a validation object
		val = dataValidation(img_Path,annotation_path,validated_imageNames)
		# load the bounding box from the annotations and store internally
		val.annotation_BBox()
		#print "Validated images:"
		#print validated_imageNames
		# perform validation
		command = val.manual_adjustment(mode)
		# If the user wants to go back
		if command == "previous":
			# 1. open file (and automatically close when 'with' exits)
			with open(validate_filename, 'r') as validate_file:
				data = validate_file.read().splitlines()
				print "Data:"
				print data
			# 2. remove validate image
			print "Remove"
			print validated_imageNames[-1]
			#data.replace(validated_imageNames[-1],"") # maybe the newline is left
			data.remove(str(validated_imageNames[-1]))
			with open(validate_filename, 'w') as validate_file:
				validate_file.write('\n'.join(data))
				# 3. close file and lock
			# Next image to be validated is the same
			prev_image_path = os.path.join(os.path.dirname(image_path),validated_imageNames[-1]+'.jpg')
			image_paths = [prev_image_path] # list with one element
			# 4. remove from list of validated images
			del validated_imageNames[-1]
			print "impath:"
			print prev_image_path
			#i += -1
		elif command == "accepted": # If the user accepted
			with open(validate_filename,'a') as validate_file:
					validate_file.write(imgName+'\n')
			# append
			validated_imageNames.append(imgName)
			# update list of images to be validated new image must be one of these
			image_paths = readOnlyInvalidImages(image_path,validate_filename, rejected_filename)
		elif command == "rejected":
			with open(rejected_filename,'a') as rejected_file:
					rejected_file.write(imgName+'\n')
			# append
			rejected_imageNames.append(imgName)
			# update list of images to be validated new image must be one of these
			image_paths = readOnlyInvalidImages(image_path,validate_filename, rejected_filename)



