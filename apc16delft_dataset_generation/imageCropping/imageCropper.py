import cv2
import argparse
import os
import glob
import numpy as np
import copy
from PATHS import originalDir
from classes import classes_short, inverse_classes

class CropImages():
	def __init__(self,images2crop):
		self.images2crop = images2crop
		self.imageIdx = 0
		self.imageNum = len(images2crop)
		self.img = None
		self.tmpImg = None
		self.xmin = 0
		self.ymin = 0
		self.xmax = 0
		self.ymax = 0
		self.ix = -1
		self.iy = -1
		self.iw = 0
		self.ih = 0
		self.drawing = False

	def cropRegionOfInterest(self):
		print "Showing {}-th image.".format(self.imageIdx)
		windowName = "Crop the region of interest. [(q)uit]"
		cv2.namedWindow(windowName,cv2.WINDOW_AUTOSIZE)
		cv2.setMouseCallback(windowName,self.draw_ROI)


		while(1):
			self.img = cv2.imread(self.images2crop[self.imageIdx])
			cv2.rectangle(self.img, (self.xmin,self.ymin),(self.xmax,self.ymax),(255,255,255),2)
			cv2.imshow(windowName,self.img)
			k = cv2.waitKey(1) & 0xFF
			if k == ord('q'):
				print 'Did nothing. How boring.'
				break
			elif k == ord('n'):
				self.imageIdx += 1
				if self.imageIdx > len(self.images2crop) - 1:
					self.imageIdx = 0
				self.cropRegionOfInterest()
				break
			elif k == ord('l'):
				self.imageIdx -= 1
				if self.imageIdx < 0:
					self.imageIdx = len(self.images2crop)
				self.cropRegionOfInterest()
			elif k == ord('y'):
				self.crop()
				break

	def draw_ROI(self, event, x, y, flags, params):
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
			print "Current choice of cropping area: "
			print (self.xmin,self.ymin,self.xmax, self.ymax)

	def crop(self):
		for image in self.images2crop:
			img = cv2.imread(image)
			imgWidth,imgHeight,imgDepth = img.shape
			img = img[self.ymin:self.ymax, self.xmin:self.xmax] # cv2 uses xidx and yidx reversely
			cv2.imwrite(image, img)
		print "Images in {} overwrited".format(os.path.dirname(images2crop[0]))







if __name__ == "__main__":
	parser = argparse.ArgumentParser(description='Crop the original images and keep the region of interest only.')
	parser.add_argument('--c', help='classname of the target images')
	parser.add_argument('--p', help='prefix of the target images')
	args = parser.parse_args()
        classid = classes_short[args.c]
	classname = inverse_classes[classid]
	prefix = args.p


	print 'Processing data from {}'.format(originalDir)
	print 'Processing {0} images from {1} degree'.format(classname, prefix)

	dir2class = os.path.join(originalDir,classname)
	foregroundDir = os.path.join(dir2class, 'foreground')
	backgroundDir = os.path.join(dir2class, 'background')
	images2crop = glob.glob(os.path.join(foregroundDir,prefix+'*.png'))
	#TODO should also crop background images
	images2crop = images2crop + glob.glob(os.path.join(backgroundDir,prefix+'*.png'))

	cropper = CropImages(images2crop)
	cropper.cropRegionOfInterest()

