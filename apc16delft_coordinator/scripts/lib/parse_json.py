import json

import job as Job
from bins import binIndexToString, binIndexFromString
from apc16delft_msgs.objects import objectTypeFromString

def parsePickOrders(node):
	for order in node:
		object_type  = objectTypeFromString[order['item']]
		bin_index    = binIndexFromString[order['bin']]
		yield Job.PickOrder(object_type, bin_index)

def parseObjectList(node):
	return [objectTypeFromString[x] for x in node]

def parseBinContents(node):
	for i in range(12):
		bin_name = binIndexToString[i]
		try:
			yield parseObjectList(node[bin_name])
		except KeyError:
			yield []

def parsePickDescription(node):
	bin_contents  = list(parseBinContents(node['bin_contents']))
	orders        = list(parsePickOrders(node['work_order']))
	tote_contents = list(parseObjectList(node['tote_contents']))
	return bin_contents, orders, tote_contents

def parseStowDescription(node):
	bin_contents  = list(parseBinContents(node['bin_contents']))
	tote_contents = list(parseObjectList(node['tote_contents']))
	return bin_contents, tote_contents

def loadJsonFile(filename):
	with open(filename, 'r') as json_file:
		return json.load(json_file)

def loadPickDescription(filename):
	return parsePickDescription(loadJsonFile(filename))

def loadStowDescription(filename):
	return parseStowDescription(loadJsonFile(filename))
