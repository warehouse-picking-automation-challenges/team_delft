import os
import errno
import rospy
import genpy
import yaml

def __makedirs(path):
	try:
		os.makedirs(path)
	except OSError as exception:
		if exception.errno != errno.EEXIST: raise

def dumpMessage(filename, message, create_directories = True):
	data = str(message).split('\n')
	stripped = []
	for line in data:
		if line and line[-1] == ' ': line = line[:-1]
		stripped.append(line)
	__makedirs(os.path.dirname(filename))

	with open(filename, 'w') as file:
		file.write('\n'.join(stripped))
		file.write('\n')

def loadMessage(type, filename):
	with open(filename, 'r') as file:
		data = yaml.load(file.read())
	message = type()
	genpy.message.fill_message_args(message, [data])
	return message
