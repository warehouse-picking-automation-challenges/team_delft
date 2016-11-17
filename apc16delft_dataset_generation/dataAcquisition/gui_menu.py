#!/usr/bin/env python
import cv2
import numpy as np
import matplotlib.pyplot as plt
import copy
import os
import math

class main_menu():
	"""A pop-out main menu method
	"""

	def __init__(self):
		self.winName = 'DelftRobotics APC2016 GUI'
		imgpath = self.parse_imgpath()
		self.img = cv2.imread(imgpath)
		self.font = cv2.FONT_HERSHEY_SIMPLEX
		self.window_size = np.array([self.img.shape[0],self.img.shape[1]])
		self.interact_size = np.array([self.window_size[0]/2, self.window_size[1]])
		self.header_size = self.window_size - self.interact_size

		self.button_shape = np.array([1, 2])
		self.button_size = None

		self.interface = self.img.copy()
		self.button_names = [
						"   Start bringup",
						" Start coordinator",
						"    Stop bringup",
						" Stop coordinator",
						"Create picking order",
						"Create stowing order"
						]
		self.buttons = None
		self.button_position = None
		self.button_selected = None
		self.create_menu()
	
	def parse_imgpath(self):
		cwdpath = os.getcwd()
		pathsplit = cwdpath.split('/')
		parentpaths = pathsplit[0:pathsplit.index('team_delft')+1]
		parentpath = ''
		for elem in parentpaths:
			parentpath = parentpath + elem + '/'
		imgpath = parentpath + 'apc16delft_dataset_generation/dataAcquisition/apc16_gui_menu.jpg'
		return imgpath


	
	def create_menu(self):
		num_buttons = len(self.button_names)
		num_row = int(np.sqrt(num_buttons)) + 1
		num_col = math.ceil(num_buttons / num_row)
		button_size = np.divide(self.interact_size, 1.5 * np.array([num_row, num_col]))
		self.button_size = button_size.astype(int)

		inds = [(x+0.5, y+0.5) for x in np.arange(num_row) for y in np.arange(num_col)]
		button_translation = np.reshape(np.array(inds), [num_row, num_col, -1])
		position_scale = [self.interact_size[0]/num_row, self.interact_size[1]/num_col]
		button_translation[:,:,0] = button_translation[:,:,0] * position_scale[0] + self.header_size[0]
		button_translation[:,:,1] = button_translation[:,:,1] * position_scale[1] + self.header_size[1]
		button_position = np.array([map(int, [button_translation[x][y][1]-self.button_size[1]/2, \
						button_translation[x][y][0]-self.button_size[0]/2, \
						button_translation[x][y][1]+self.button_size[1]/2, \
						button_translation[x][y][0]+self.button_size[0]/2]) \
						for x in np.arange(num_row) for y in np.arange(num_col)])
		button_position = np.reshape(button_position, [num_row, num_col, -1])
		self.button_position = button_position

		num_empty_button = num_row * num_col - num_buttons
		buttons = copy.deepcopy(self.button_names)
		for i in np.arange(num_empty_button):
			buttons.append(None)

		buttons = np.reshape(buttons, [num_row, -1])

		if buttons.shape == self.button_position.shape[0:-1]:
			self.buttons = buttons
		else:
			print "Error: number of buttons and button positions does not match"

		# create buttons
		for x in np.arange(num_row):
			for y in np.arange(num_col):
				if buttons[x][y] != None:
					fill_color = (0,0,0)
					if buttons[x][y] == "   Start bringup" or buttons[x][y] == " Start coordinator":
						fill_color = (0,255,0)
					elif buttons[x][y] == "    Stop bringup" or buttons[x][y] == " Stop coordinator":
						fill_color = (0,0,255)
					cv2.rectangle(self.img, (button_position[x][y][0], button_position[x][y][1]), \
							(button_position[x][y][2], button_position[x][y][3]), fill_color, -1)
					cv2.putText(self.img, buttons[x][y], (button_position[x][y][0]+self.button_size[0]/8, \
							button_position[x][y][1]+self.button_size[1]/4), self.font, 0.5, (255,255,255), 2)

		# render headers

		self.interface = self.img.copy()
	
	
	def button_parser(self, x, y):
		for i in np.arange(self.button_position.shape[0]):
			for j in np.arange(self.button_position.shape[1]):
				if self.button_position[i][j][0] <= x < self.button_position[i][j][2] \
						and self.button_position[i][j][1] <= y < self.button_position[i][j][3]:
							return self.button_position[i][j], self.buttons[i][j]
		return None, None

	def click(self, event, x, y, flags, param):
		if event == cv2.EVENT_LBUTTONDOWN:
			but_reg, but = self.button_parser(x, y)
			self.interface = self.img.copy()
			if but_reg != None and but != None:
				cv2.rectangle(self.interface, (but_reg[0], but_reg[1]), (but_reg[2], but_reg[3]), (255,255,255), -1)
				cv2.putText(self.interface, but, (but_reg[0]+self.button_size[0]/8, but_reg[1]+self.button_size[1]/4), \
						self.font, 0.5, (0,0,0), 2)
				self.button_selected = but

	def select(self):
		cv2.namedWindow(self.winName)
		down = cv2.setMouseCallback(self.winName, self.click)
		
		while True:
			cv2.imshow(self.winName, self.interface)
			key = cv2.waitKey(1) & 0xff
			if (key == 32 or key == 10) and self.button_selected != None:
				break
			elif key == 27: #Esc
				self.button_selected = None
				return None

		cv2.destroyWindow(self.winName)
		if self.button_selected != None:
			return self.button_names.index(self.button_selected)




