#!/usr/bin/env python

import os
import glob
import Pyro4

#import the path settings
from PATHS import originalDir, imageDir, xmlDir, imageSetDir, segmentationDir
from PATHS import testoriginalDir, testimageDir, testxmlDir, testimageSetDir, testsegmentationDir
from classes import classes, classes_short, inverse_classes, inverse_classes_short
import argparse
import time
import datetime
from dataLib.writeAnnotations import toXML
from dataPreprocessing.processFunctions import getClass
from dataLib.colors import bcolors
import sys
import signal
import getpass
import string

class masterValidator():

	def __init__(self, mode, daemon):
		self.daemon = daemon # needed to nicely shutdown
		self.mode = mode
		self.user = getpass.getuser()
		if self.mode == 'traindata':
			self.image_dir			= imageDir # not jpegDir anymore
			self.original_dir		= originalDir
			self.segmentation_dir	= segmentationDir
			self.xml_dir			= xmlDir
			self.imageSet_path		= imageSetDir
		elif self.mode == 'testdata':
			self.image_dir			= testimageDir
			self.original_dir		= testoriginalDir
			self.segmentation_dir	= None
			self.xml_dir			= testxmlDir
			self.imageSet_path		= testimageSetDir
		else:
			print bcolors.BOLD+ bcolors.FAIL + "mode must either be 'traindata' or 'testdata' (without apostrophes)" + bcolors.ENDC
			sys.exit()

		self.validation_file	= self.imageSet_path+"validated.txt"
		self.rejection_file		= self.imageSet_path+"rejected.txt"
		self.image_format		= '.png'

		print "PARAMETERS:"
		print "Mode:............ " + bcolors.WARNING+bcolors.BOLD+self.mode+bcolors.ENDC
		print "image_dir:....... " + self.image_dir
		print "original_dir: ... " + self.original_dir
		print "segmentation_dir: " + str(self.segmentation_dir)
		print "xml_dir: ........ " + self.xml_dir
		print "imageSet_path:... " + self.imageSet_path
		print "validation_file:. " + self.validation_file
		print "image_format:.... " + self.image_format
		print ''

		# construct empty lists for storing actions, images that are validated (accepted) and iages that are rejected
		self.actions			= []
		self.validated_images	= []
		self.rejected_images	= []
		# Dictionary of users/slaves to tasks
		self.userTasks			= {}

		# Construct a list of ALL images that NEED to be validated
		# This also initializes the self.validated_images and the self.rejected_images with the contents of those files
		self.image_paths = self.readOnlyInvalidImages()

		# create directory for logfiles if it does not exist
		if not os.path.exists('./dataValidation/logs'):
			os.mkdir('./dataValidation/logs')

		# create logfile if it does not exist yet
		timestamp = datetime.datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d_%H:%M:%S')
		self.logFile = os.path.join('./dataValidation','logs',timestamp+".log")
		with open(self.logFile,'w+') as lf: pass

## this line is useless

	def readAllImages(self):
		image_path_list = []
		if self.mode == 'traindata':
			#print self.original_dir
			for classDir in os.walk(self.original_dir).next()[1]: # for all directories of classes in the originalDir
				globPath	= os.path.join(self.original_dir,classDir,'foreground','*'+self.image_format)
				image_paths = glob.glob(globPath)
				image_path_list.extend(image_paths) # use extend instead of append for correct functionality
		else: # mode == 'testdata'
			for binDir in os.walk(self.original_dir).next()[1]: # for all directories of classes in the originalDir
				globPath    = os.path.join(self.original_dir, binDir, '*' + self.image_format)
				image_paths = glob.glob(globPath)
				image_path_list.extend(image_paths) # use extend instead of append for correct functionality
		return image_path_list #image_names


	def readOnlyInvalidImages(self):
		image_paths = self.readAllImages()

		# Check if validation file exists, if not: create
		if not os.path.exists(self.validation_file):
			with open(self.validation_file,'w+') as f: pass

		# Check if the rejection file exists, if not: create
		if not os.path.exists(self.rejection_file):
			with open(self.rejection_file,'w+') as f: pass

		# Open validation file and read contents
		with open(self.validation_file,'r+') as vf:
			validated_images = vf.read().splitlines()
			# initialize internal representation of the file
			self.validated_images = validated_images

		# Open rejection file and read contents
		with open(self.rejection_file,'r+') as rf:
			rejected_images = rf.read().splitlines()
			# initialize internal representation of the file
			self.rejected_images = rejected_images

		image_paths		= [img_path for img_path in image_paths if not os.path.splitext(os.path.basename(img_path))[0] in validated_images and not os.path.splitext(os.path.basename(img_path))[0] in rejected_images]

		if len(image_paths) > 0:
			print 'Images to be validated: ' + bcolors.BOLD + str(len(image_paths)) + bcolors.ENDC
		else:

			# No images to validate, exit
			print bcolors.WARNING+ bcolors.BOLD + "No images to validate. \nExit" + bcolors.ENDC
			#self.daemon.shutdown()
			sys.exit()

		return image_paths

	def updateRejectionFile(self):
		with open(self.rejection_file,'w') as fr:
			for rej in self.rejected_images:
				fr.write(rej+'\n')

	def updateValidationFile(self):
		with open(self.validation_file,'w') as vf:
			for val in self.validated_images:
				vf.write(val+'\n')

	def getImageName(self,img_path):
		return os.path.splitext(os.path.basename(img_path))[0]

	def getImagePath(self,img_name):
		if self.mode == 'traindata':
			return os.path.join(self.originalDir,img_class,'foreground',img_name + self.image_format)
		else: # mode == 'testdata'
			return os.path.join(self.originalDir,img_name+self.image_format)

	def checkConnection(self, user):
		print "User "+ user + " tries to connect"
		# check if user is not still has a task registerd (this happend when a slave incorrectly shuts down)
		print self.userTasks
		if user in self.userTasks.keys():
			print "User " + user + "has tasks standing"
			img_path = self.userTasks[user]
			# append task to the list of tasks
			if not task == '':
				self.image_paths.insert(0, img_path)

		self.userTasks[user] = ''

		logmessage = "User "+ user + " connected"
		self.writeLogMessage(logmessage)
		return True

	def getUserPath(self,generalPath, user):
		return string.replace(generalPath,self.user,user,1) # only replace first occurrence


	#def getGeneralPath(self,userPath,user):
	#return string.replace(userPath,user,self.user)

	def getTask(self,user):
		# only give tasks whene there are still tasks to give
		if len(self.image_paths) > 0:
			# image_paths is list of paths to images that need to be validated
			# Take one image out of this list
			img_path	= self.image_paths[0]
			usr_path	= self.getUserPath(img_path,user)
			# delete it from the list
			del self.image_paths[0]
			# add the as current task of the user
			self.userTasks[user]=img_path

			# construct the annotation path
			#annotation_path = self.getXMLPath(img_name)
			logmessage = user + " got task: " + os.path.basename(usr_path)
			self.writeLogMessage(logmessage)
			return usr_path
		else:
			return None

	def bounce(self, user):
		# task is returned, nothing is done by slave
		# re insert img_name in image_names
		img_path	= self.userTasks[user]
		self.userTasks[user] = ''

		self.image_paths.insert(0, img_path)
		logmessage = user + " bounced task: " + os.path.basename(usr_path)
		self.writeLogMessage(logmessage)

	def accept(self, user):
		img_path = self.userTasks[user]
		img_name = self.getImageName(img_path)
		self.userTasks[user] = ''

		self.validated_images.append(img_name)

		# image is validated/accepted by slave
		total_images = len(self.image_paths) + len(self.validated_images)
		logmessage = user + " verified: " + img_name + " images left: ({}/{})".format(len(self.image_paths), total_images)

		self.actions.append((user, img_path,'accept'))
		self.writeLogMessage(logmessage)
		self.updateValidationFile()

	def undo(self,user):
		# append to image_paths
		img_path = self.userTasks[user]
		self.image_paths.insert(0, img_path)
		img_name = self.getImageName(img_path)
		# find previous action for img_name (start searching from the end)
		action = ''
		img_name = ''
		for i, (usr, img_path, act) in reversed(list(enumerate(self.actions))):
			if usr == user:
				# remove actions and stop search
				action = act
				img_name = self.getImageName(img_path)
				del self.actions[i]
				self.userTasks[user] = img_path # new task of slave is his previous task
				break
		# depending on the action, remove it from the list of accepted or rejected images
		if action == 'accept':
			# remove from list of verified images
			self.validated_images.remove(img_name)
			self.updateValidationFile()
		elif action == 'reject':
			# remove from list of rejected images
			self.rejected_images.remove(img_name)
			self.updateRejectionFile()
		else:
			# give error
			print "ERROR in undo: Action: "+ action + " by user: " + user + " not recognized"

		logmessage = user + " undid: action = " + action + " for task: "+ img_name
		self.writeLogMessage(logmessage)

		return self.getUserPath(self.userTasks[user],user) # return the task of the slave

	def reject(self, user):
		img_path = self.userTasks[user]
		self.userTasks[user] = ''
		# image is rejected by slave (should not get into validated.txt)
		img_name = self.getImageName(img_path)
		self.rejected_images.append(img_name)
		self.actions.append((user,img_path,'reject'))
		print user + "rejected: " + img_name
		self.updateRejectionFile()

	def writeLogMessage(self,message):
		print message
		with open(self.logFile,'a') as lf:
			lf.write(message+'\n')

	def disconnect(self,user):
		img_path = self.userTasks[user]
		if not img_path == '':
			self.image_paths.insert(0, img_path)
		del self.userTasks[user]

		logmessage = "User: " + user + " disconnected"
		self.writeLogMessage(logmessage)
		# Clause for end of validation procedure
		if len(self.userTasks) == 0 and len(self.image_paths) == 0:
			# Validation process is completed. All users are already logged out, and there are no tasks left
			print bcolors.OKGREEN + bcolors.BOLD + "Validation procedure completed !!" + bcolors.ENDC
			self.daemon.shutdown()

	def getMode(self):
		return self.mode

def exit_sigint(signum,frame):
	print bcolors.BOLD + bcolors.FAIL + 'Abort' + bcolors.ENDC

	# shutdown the daemon (global)
	daemon.shutdown()

	# TODO: Check if exit is allowed
	sys.exit()

	# Restore original sigint handler, otherwise evil things happen
	signal.signal(signal.SIGINT,original_sigint)

def main():
	print bcolors.WARNING + """Don't forget to start the nameserver first!\n
Execute: 'pyro4-ns' in a seperate terminal to start the nameserver\n"""+bcolors.ENDC

	## Parse argument(s)
	parser = argparse.ArgumentParser(description='Get the mode of the validation process')
	parser.add_argument('-mode', type=str,
			default='traindata',
			dest='mode',
			help="The mode of the validation process 'traindata' (default) or 'testdata'")
	parser.add_argument('-local', action='store_true',
			default=False,
			help="Type of connection (local vs non-local)")
	args = parser.parse_args()
	mode = args.mode

	global daemon

	# check if dataset is mounted
	if not os.path.isdir(originalDir) and not os.path.isdir(testoriginalDir):
		sys.exit(bcolors.FAIL+"Dataset is not mounted: " + originalDir + " " + testoriginalDir + bcolors.ENDC)

	# initialize/create master validator
	if not args.local:
		hostAdr = Pyro4.socketutil.getIpAddress(None,workaround127=True)
		print "hostAdr: " + hostAdr
		daemon = Pyro4.Daemon(host=hostAdr)
		ns = Pyro4.locateNS(hostAdr)
	else:
		daemon = Pyro4.Daemon()
		ns = Pyro4.locateNS()

	master		= masterValidator(mode,daemon)
	master_uri	= daemon.register(master)
	print "Master URI: " + str(master_uri)

	ns.register("validator.master",master_uri)
	print bcolors.OKGREEN + "Master validator running and connected to nameserver" + bcolors.ENDC

	# create a signal handler for SIGINT
	global original_sigint
	original_sigint = signal.getsignal(signal.SIGINT)
	signal.signal(signal.SIGINT,exit_sigint)

	# Start the master and wait for calls from the slaves
	daemon.requestLoop()

if __name__ == "__main__":
	main()
