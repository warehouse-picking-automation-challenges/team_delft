#!/usr/bin/env python

import cv2
import os
import copy
import sys
import argparse
import Pyro4
import getpass
import signal

from dataValidation.validator_test import validator_test
from dataValidation.validator_train import validator_train
from dataLib.colors import bcolors

from PATHS import originalDir, testoriginalDir

def connect2Master(user,ns_ip):
	ns = Pyro4.locateNS(ns_ip)
	master_uri = ns.lookup("validator.master")
	## Connect to validation master
	master = Pyro4.Proxy(master_uri)
# TODO: somewhere here is an error.
	try:
		master.checkConnection(user)
		print bcolors.OKGREEN+"Connection working"+bcolors.ENDC
	except:
		print bcolors.FAIL + "No connection established"+bcolors.ENDC
# TODO: break or exit here
	return master


def exit_sigint(signum,frame):
	# Restore original sigint handler, otherwise evil things happen
	signal.signal(signal.SIGINT,original_sigint)

	# bounce current task
	# master.bounce(user,img_path) # is handeled internally in hte master
	master.disconnect(user)

	sys.exit(bcolors.OKGREEN + bcolors.BOLD + "Quit succesfully" + bcolors.ENDC)

def main():
	# create a signal handler for SIGINT, set some variables to global because the signal handler needs access
	global original_sigint, master, user, img_path
	original_sigint = signal.getsignal(signal.SIGINT)
	signal.signal(signal.SIGINT,exit_sigint)

	## Parse argument(s)
	parser = argparse.ArgumentParser(description='Get the ip adress of the nameserver to be able to find the master')
	parser.add_argument('-ip', type=str,
			default='',
			dest='ns_ip',
			help="The ip of adress of the nameserver (find with ifconfig)")
	args = parser.parse_args()

	# check if dataset is mounted
	if not os.path.isdir(originalDir) and not os.path.isdir(testoriginalDir):
		sys.exit(bcolors.FAIL+"Dataset is not mounted"+bcolors.ENDC)

        user = getpass.getuser()
	print "User:\t"+user
	master = connect2Master(user,args.ns_ip)

	# Initialize the slave validator and get the validation mode from the master ('traindata' or 'testdata')
	mode = master.getMode()

	processed_imagePaths	= []
	processed_xmlPaths		= []

	# construct a new validator object
	if mode == 'traindata':
		validator = validator_train()
	else:
		validator = validator_test()

	action = ''
	while True:
		if action != 'previous': # only get a new task when previous is not pressed
			# get a task from the master
			img_path = master.getTask(user)
			if img_path == None:
				master.disconnect(user)
				sys.exit(bcolors.OKGREEN + "Validation completed. " + bcolors.ENDC+'Thank you, ' + user + '!')
			print bcolors.BOLD + "Received task:" + bcolors.ENDC
		else:
			print bcolors.BOLD + "Redo task:" + bcolors.ENDC
		print img_path
		validator.setImage(img_path)

		# perform validation
		action = validator.validate()

		print "Action: " + bcolors.BOLD + action + bcolors.ENDC
		if action == 'accept':
			master.accept(user)
			processed_imagePaths.append(img_path)
		elif action == 'previous':
			if not len(processed_imagePaths) == 0:
				# notify master and get the previous task
				previous_img_path = master.undo(user)
				# only remove last image if there is more than one image
				del processed_imagePaths[-1]
			else:
				# for the case no images are processed yet
				previous_img_path = img_path
			img_path = previous_img_path
		elif action == 'reject':
			master.reject(user)
			processed_imagePaths.append(img_path)
		elif action == 'quit':
			master.disconnect(user)
			sys.exit(bcolors.OKGREEN + bcolors.BOLD + "Quit succesfully" + bcolors.ENDC)
		else:
			master.disconnect(user)
			sys.exit(bcolors.BOLD + bcolors.FAIL+ "Action: " + str(action) + ", is invalid." + bcolors.ENDC)

if __name__ == "__main__":
	main()
