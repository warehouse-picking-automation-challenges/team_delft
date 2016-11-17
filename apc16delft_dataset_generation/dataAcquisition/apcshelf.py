#!/usr/bin/env python
import cv2
import numpy as np

class bin_selection():
	"""A pop-out bin selection method
	"""
	def __init__(self, bins_occupied):
		self.bins_occupied = bins_occupied
		self.winName = 'Select the bin and press space to confirm'
		self.font = cv2.FONT_HERSHEY_SIMPLEX
		self.bin_size = 200
		self.shape = np.array([4,3])
		self.size  = self.shape * self.bin_size
		self.img = np.ones(np.append(self.size,[3])) * 255
		self.interface = None
		self.bin_names = [['bin_A', 'bin_B', 'bin_C',],
							['bin_D', 'bin_E', 'bin_F'],
							['bin_G', 'bin_H', 'bin_I'],
							['bin_J', 'bin_K', 'bin_L']]
		self.bins = []
		self.build_shelf()
		self.bin_selected = None

	def build_shelf(self):
		# initialise bins coordinates
		inds = [(x, y, (x+1), (y+1)) for x in range(0,self.shape[0]) for y in range(0,self.shape[1])]
		self.bins = np.reshape(np.array(inds), [4, 3, -1] )* self.bin_size

		# build bins
		for x in range(0, self.shape[0]):
			for y in range(0, self.shape[1]):
				# render occupied bins
				if self.bin_names[x][y] in self.bins_occupied:
					cv2.rectangle(self.img, (self.bins[x][y][1], self.bins[x][y][0]), (self.bins[x][y][3], self.bins[x][y][2]), (0,0,255), -1)
				# render shelf structure
				cv2.rectangle(self.img, (self.bins[x][y][1], self.bins[x][y][0]), (self.bins[x][y][3], self.bins[x][y][2]), (0,0,0), 5)
				cv2.putText(self.img, self.bin_names[x][y], (self.bins[x][y][1]+50, self.bins[x][y][0]+100), self.font, 1, (0,0,0), 2)
				

		self.interface = self.img.copy()

	def bin_parser(self, x, y):
		i = y / self.bin_size
		j = x / self.bin_size
		if 0 <= i < self.shape[0] and 0 <= self.shape[1]:
			return self.bins[i][j], self.bin_names[i][j]

	def click(self, event, x, y, flags, param):
		if event == cv2.EVENT_LBUTTONDOWN:
			bin_reg, bin_name = self.bin_parser(x, y)

			# if the bin has not been occupied
			if bin_name not in self.bins_occupied:
				self.interface = self.img.copy()
				cv2.rectangle(self.interface, (bin_reg[1],bin_reg[0]), (bin_reg[3],bin_reg[2]), (0,0,0), -1)
				cv2.putText(self.interface, bin_name, (bin_reg[1]+50, bin_reg[0]+100), self.font, 1, (255,255,255), 2)
				self.bin_selected = bin_name


	def select(self):
		cv2.namedWindow(self.winName)
		cv2.setMouseCallback(self.winName, self.click)
		while True:
			cv2.imshow(self.winName, self.interface)
			key = cv2.waitKey(1) & 0xff
			if (key == 32 or key == 10) and self.bin_selected != None \
					and self.bin_selected not in self.bins_occupied:
				break
			elif key == 27 or key == ord('a'): #Esc
				self.bin_selected = None
				break
		cv2.destroyWindow(self.winName)
		return self.bin_selected


