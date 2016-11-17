import cv2
import os
import numpy as np
import glob
import copy

from classes import classes, classes_short, inverse_classes, inverse_classes_short, getShortName
from dataPreprocessing.processFunctions import calcSegmentation, storeSegmentation, loadSegmentation, getClass, getBoundingBox
from captcha import object_selection
from segmentationManager import segmentationManager
import time
from dataLib.writeAnnotations import toXML, get_annotation_BBox
from dataValidation.segmentationManager import segmentationManager
from dataLib.colors import bcolors

from PATHS import originalDir, segmentationDir, imageSetDir, xmlDir
class validator_train():

	def __init__(self):
		self.originalDir     = originalDir
		self.xmlDir          = xmlDir
		self.imageSetDir     = imageSetDir
		self.segmentationDir = segmentationDir

		# set parameters for drawing and display of the mask
		self.drawing       = False
		self.brushMode     = "normal"
		self.alpha         = 0.5
		self.gamma         = 0.8
		self.brushsize     = 11
		self.polygonPoints = np.zeros((0, 2)).astype(np.int32)

		# Create a window for viewing
		self.winName = 'Is the bounding box correct? [(a)accept/(r)eject/(p)revious/(c)lear/(q)uit]'
		cv2.namedWindow(self.winName,cv2.WINDOW_NORMAL)
		cv2.namedWindow("Zoom", cv2.WINDOW_NORMAL)

		## Link user input to the window
		cv2.setMouseCallback(self.winName,self.adjustMask)
		self.mouse =  {'x':0, 'y':0}

	def setImage(self,img_path):
		self.imageFileName = os.path.basename(img_path)
		self.imageName     = os.path.splitext(self.imageFileName)[0]
		self.imageClass    = getClass(img_path)

		# start the segmentation manager that allows for undo and redo actions
		self.segmentationMngr = segmentationManager()
		self.segmentationPath = os.path.join(self.segmentationDir,self.imageClass,self.imageFileName)

		self.img          = cv2.imread(img_path)
		self.zoom         = np.zeros((500, 500))
		self.tmpImg       = copy.deepcopy(self.img)
		self.segmentation = self.getSegmentation(img_path) # automatically updates the segmentationmanager

		# load the annotation and check for a bounding box, if not, then calculate the bounding box based on the segmentation
		annotation_path = os.path.join(self.xmlDir,self.imageName+'.xml')
		self.objs       = get_annotation_BBox(annotation_path)
		if not self.objs:
			xmin, ymin, width, height = getBoundingBox(self.segmentation)
			self.objs.append({"name":self.imageClass,"bbox":(xmin,ymin,width,height)})

		# Redraw screen with new image and bounding boxes
		self.updateWindow()

		# Set drawing mode to false
		self.drawing = False

	# mouse event handler for drawing the segmentation
	def adjustMask(self, event, x, y, flags, params):
		if self.brushMode == "normal":
			if event == cv2.EVENT_LBUTTONDOWN:
				self.drawing  = True
				self.drawMode = 'add'
				self.changeMask(x,y,self.brushsize,'add')

			if event == cv2.EVENT_RBUTTONDOWN:
				self.drawing  = True
				self.drawMode = 'remove'
				self.changeMask(x,y,self.brushsize,'remove')

			if event == cv2.EVENT_LBUTTONUP or event == cv2.EVENT_RBUTTONUP:
				self.drawing  = False
				self.drawMode = None
				# update segmentation manager
				self.segmentationMngr.addSegmentation(self.segmentation)

			if event == cv2.EVENT_MOUSEMOVE and self.drawing == True:
				self.changeMask(x,y,self.brushsize,self.drawMode)

		elif self.brushMode == "polygon":
			if (event == cv2.EVENT_LBUTTONDOWN or event == cv2.EVENT_RBUTTONDOWN) and \
				flags & (cv2.EVENT_FLAG_SHIFTKEY | cv2.EVENT_FLAG_CTRLKEY | cv2.EVENT_FLAG_ALTKEY):
				# also add current point
				self.polygonPoints = np.vstack([self.polygonPoints, [x, y]])

				# add or remove from segmentation
                                mode = ''
                                if (flags & cv2.EVENT_FLAG_SHIFTKEY):
                                    mode = 'polygon-add'
                                    self.changeMask(x, y, self.brushsize, mode)
                                elif(flags & cv2.EVENT_FLAG_CTRLKEY):
                                    mode = 'polygon-remove-inside'
                                    self.changeMask(x, y, self.brushsize, mode)
                                elif(flags & cv2.EVENT_FLAG_ALTKEY):
                                    print "ALTKEY"
                                    mode = 'polygon-remove-outside'
                                    self.changeMask(x, y, self.brushsize, mode)
                                else:
                                    pass

                                # reset polygon tool
				self.polygonPoints = np.zeros((0, 2)).astype(np.int32)
				self.segmentationMngr.addSegmentation(self.segmentation)
			elif event == cv2.EVENT_LBUTTONDOWN or event == cv2.EVENT_RBUTTONDOWN:
				self.polygonPoints = np.vstack([self.polygonPoints, [x, y]])

		# update mouse positions
		if event == cv2.EVENT_MOUSEMOVE:
			self.mouse["x"] = x
			self.mouse["y"] = y

		self.updateTmpImg()

		min_p = (max(0, x-50), max(0, y-50))
		max_p = (min(self.img.shape[1], x+50), min(self.img.shape[0], y+50))
		self.zoom = self.tmpImg[min_p[1]:max_p[1], min_p[0]:max_p[0]]
		if self.zoom.size:
			self.zoom = cv2.cvtColor(self.zoom, cv2.COLOR_BGR2GRAY)
			self.zoom = cv2.equalizeHist(self.zoom)
			self.zoom = cv2.resize(self.zoom, (500, 500))
			cv2.line(self.zoom, (250, 225), (250, 275), (255))
			cv2.line(self.zoom, (225, 250), (275, 250), (255))
			cv2.imshow("Zoom", self.zoom)
			# ugly hack to force draw
			cv2.waitKey(1)

	def changeMask(self, x0, y0, brushsize, mode):
		if mode == 'add':
			cv2.circle(self.segmentation,center=(x0,y0),radius=brushsize,color=(255,0,0),thickness=-1)
		elif mode == 'remove':
			cv2.circle(self.segmentation,center=(x0,y0),radius=brushsize,color=(0,0,0),thickness=-1)
		elif mode == 'polygon-add':
			cv2.fillPoly(self.segmentation, [self.polygonPoints], (255, 0, 0))
		elif mode == 'polygon-remove-inside':
			cv2.fillPoly(self.segmentation, [self.polygonPoints], (0, 0, 0))
                elif mode == 'polygon-remove-outside':
                        tmpSegmentation = copy.deepcopy(self.segmentation)
                        cv2.cvSet(tmpSegmentation, CV_RGB(0,0,0))
                        cv2.fillPoly(tmpSegmentation, [self.polygonPoints], (255, 0, 0))
                        cv2.bitwise_and(tmpSegmentation, self.segmentation, self.segmentation)
		else: print 'Error in changeMask, no correct mode detected'

		# update target object
		self.updateObjects()


	def drawState(self):
		# Draw bounding box + classname
		for obj in self.objs:
			print self.objs
			print obj["name"]
			name = self.getShortName(obj["name"])
			x,y,w,h = obj["bbox"]
			cv2.rectangle(self.tmpImg, (x,y), (x+w, y+h), (100,100,100), 2)
			cv2.putText(img=self.tmpImg,text=name,org=(x,y),fontFace=cv2.FONT_HERSHEY_SIMPLEX,fontScale=2,color=(255,255,255),thickness=2,lineType=cv2.CV_AA)

	def updateTmpImg(self):
		alpha       = self.alpha
		beta        = 1-alpha
		height      = self.segmentation.shape[0]
		width       = self.segmentation.shape[1]
		mask        = cv2.cvtColor(self.segmentation, cv2.COLOR_GRAY2BGR)
		self.tmpImg = cv2.addWeighted(self.img, 1, mask, 1-alpha,self.gamma)

		if self.brushMode == "normal":
			# draw mouse/brush position
			cv2.circle(self.tmpImg,center=(self.mouse["x"],self.mouse["y"]),radius=self.brushsize,color=(0,0,0),thickness=2,lineType=cv2.CV_AA)
			cv2.circle(self.tmpImg,center=(self.mouse["x"],self.mouse["y"]),radius=self.brushsize,color=(255,255,255),thickness=1,lineType=cv2.CV_AA)


	def updateWindow(self):
		# update tmpImage (segmentation, mouse position)
		self.updateTmpImg()
		# recalculate bounding box
		bbox = getBoundingBox(self.segmentation)

		xmin, ymin, width, height = bbox

		if width > 0 and height > 0:
			# Draw bounding box
			name =getShortName(self.imageClass)
			offset_y = +50
			offset_x = +5
			cv2.rectangle(self.tmpImg, (xmin,ymin), (xmin+width, ymin+height), (0,0,0), 2)
			cv2.rectangle(self.tmpImg, (xmin,ymin), (xmin+width, ymin+height), (255,255,255), 1)

			# Draw the className
			cv2.putText(img=self.tmpImg,text=name,org=(offset_x,offset_y),fontFace=cv2.FONT_HERSHEY_SIMPLEX,fontScale=2,color=(0,0,0),thickness=3,lineType=cv2.CV_AA)
			cv2.putText(img=self.tmpImg,text=name,org=(offset_x,offset_y),fontFace=cv2.FONT_HERSHEY_SIMPLEX,fontScale=2,color=(255,255,255),thickness=2,lineType=cv2.CV_AA)

		if self.polygonPoints.shape[0] > 0:
			# append current mouse pos
			polygonPoints = np.vstack([self.polygonPoints, [self.mouse["x"], self.mouse["y"]]])

			# draw polygon lines
			cv2.polylines(self.tmpImg, [polygonPoints], True, (0, 0, 0), 2, cv2.CV_AA)
			cv2.polylines(self.tmpImg, [polygonPoints], True, (255, 255, 255), 1, cv2.CV_AA)

		cv2.imshow(self.winName,self.tmpImg)
		# only wait 1 ms because also the drawing needs to be updated
		k = cv2.waitKey(1) & 0xFF
		return k

	def updateObjects(self):
		# Recalculate the bounding box and store it
		xmin, ymin, width, height = getBoundingBox(self.segmentation)

		# remove all other objects, since in this mode only one object per image is present
		self.objs = []

		self.objs.append({"name":self.imageClass, "bbox":(xmin,ymin,width,height)})

	def unredo(self,k):
		if k == ord('z'): # undo
			self.segmentationMngr.undo()
		else:
			self.segmentationMngr.redo()
		self.segmentation = self.segmentationMngr.getSegmentation()

	def adjustBrush(self,k):
		if k == 61:
			self.brushsize += 5
		else:
			self.brushsize = min(self.brushsize,abs(self.brushsize-5))
		# update image that is shown
		self.updateTmpImg()

	def selectName(self):
		return self.objectSelector.select()

	def showStatus(self):
		print "status:"
		if len(self.objs) == 0 :
			print "No object found"

		for obj in self.objs:
			print(obj["name"],obj["bbox"])

	def getSegmentation(self,img_path):
		# Check if the segmentation already exists
		if os.path.exists(self.segmentationPath):
			# Load the segmentation
			self.segmentationMngr.addSegmentation(loadSegmentation(self.segmentationPath))
			segmentation = self.segmentationMngr.getSegmentation()
		else:
			# Calculate the segmentation
			imgClassDir = os.path.join(self.originalDir,self.imageClass)
			self.segmentationMngr.addSegmentation(calcSegmentation(imgClassDir,imgFileName))
			segmentation = self.segmentationMngr.getSegmentation()
			# Store the (intermediate) segmentation
			storeSegmentation(self.segmentation, self.segmentationPath)
		return segmentation

	def validate(self):
		# continuously loop to update screen and also to poll for user keyboard input
		while True:
			k = self.updateWindow()
		#	if k == ord('c'):
		#		print "Reset state"
		#		self.reset()
		#		# update tmpImage
		#		self.updateTmpImg()
		#		print "Draw state"
		#		self.drawState()
			if k == ord('a') or k == ord('A'):  # sometimes CAPS-lock is accidentally pressed.
				# does not write correctly
				toXML(self.xmlDir,self.img.shape,self.imageFileName,self.objs)
				self.showStatus()
				# export accepted segmentation
				storeSegmentation(self.segmentation, self.segmentationPath)
				return 'accept'
			elif k == 9: #Tab key
				self.brushMode = "polygon" if self.brushMode == "normal" else "normal"
				self.polygonPoints = np.zeros((0, 2)).astype(np.int32)
			elif k == 45 or k == 61: # + and - keys
				self.adjustBrush(k)
			elif k == ord('p') or k == ord('P'):
				return 'previous'
			elif k == ord('r') or k == ord('R'):
				return 'reject'
			elif k == ord('z') or k == ord('Z') or k == ord('x') or k == ord('X'):
				self.unredo(k)
			elif k == ord('q') or k == ord('Q'):
				print "Exit validation"
				return 'quit'
