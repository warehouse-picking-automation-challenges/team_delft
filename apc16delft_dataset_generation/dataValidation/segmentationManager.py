# SegmentationManager
# This class manages a list of segmentations and alows the user (slave)
# to undo changes in segmentations
import cv2
import copy
class segmentationManager():

	segmentationList	= []
	segmentationCounter = 0  # tracks how many segmentations are stored internally
	segmentationPointer = -1 # track which segmentation is being used by the slave (zero based)

	def __init__(self):
		self.segmentationCounter		= 0
		self.segmentationPointer		= -1
		self.segmentationList			= []

	def undo(self):
		# stop for beginning of list
		if self.segmentationPointer - 1 >= 0:
			self.segmentationPointer -= 1

	def redo(self):
		# stop for end of list
		if self.segmentationPointer + 1 < self.segmentationCounter:
			self.segmentationPointer += 1

	def addSegmentation(self, segmentation):
		self.segmentationPointer += 1
		del self.segmentationList[self.segmentationPointer:]
		self.segmentationList.append(copy.deepcopy(segmentation))
		self.segmentationCounter = len(self.segmentationList)

	def getSegmentation(self):
		return copy.deepcopy(self.segmentationList[self.segmentationPointer])

	def reset(self):
		self.__init__()


