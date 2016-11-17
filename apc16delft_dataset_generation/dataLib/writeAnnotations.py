import os
import numpy as np
import xml.etree.ElementTree as ET
from xml.dom import minidom

from colors import bcolors

def toXML(xmlDir, imageShape, imageFilename, objects):
	# xmlDir = directory to xml
	# imageShape = image shape
	# imageFilename = name+extension
	# objects = {"name":obj,"bbox":[1,1,20,20]}

	#print xmlDir
	#print imageShape
	#print imageFilename
	#print objects
	(fname, extension) = os.path.splitext(imageFilename)


	xml_file = os.path.join(xmlDir,fname+".xml")
	#print "Write to xml file: %s" % xml_file

	#if extension in ['jpg','.png','.ppm','.pbm']:
	#	xml_file = xmlDir+fname+".xml"
	#	print "Write to xml file: {%s}".format(xml_file)

	#else:
	#	xml_file = xmlDir + os.path.join(fname) + '.xml'

	assert len(imageShape) == 3, bcolors.FAIL + "toXML: Image shape needs to be of length 3" + bcolors.ENDC
	height = imageShape[0]
	width = imageShape[1]
	depth = imageShape[2]


	annotation = ET.Element("annotation")
	ET.SubElement(annotation, "folder").text = "DRapc2016"
	ET.SubElement(annotation, "filename").text = imageFilename
	source = ET.SubElement(annotation, "source")
	ET.SubElement(source, "database").text = "Delft Robotics Amazon Pickup Challenge"
	ET.SubElement(source, "annotation").text = "DRapc2016"
	ET.SubElement(source, "image").text = "DelftRobotics"
	owner = ET.SubElement(annotation, "owner")
	ET.SubElement(owner, "name").text = "Delft Robotics"
	size = ET.SubElement(annotation, "size")
	ET.SubElement(size, "width").text = str(width)
	ET.SubElement(size, "height").text = str(height)
	ET.SubElement(size, "depth").text = str(depth)
	ET.SubElement(annotation, "segmented").text = "0"

	if len(objects) > 0:
		for obj in objects:
			name = obj["name"]
			pose = "above"
			truncated = "0"
			difficult = "0"
			#obtainable = "1" if obj["obtainable"] else "0"
			obtainable = "1"
			(x,y,w,h) = obj["bbox"]
			ymin = y
			ymax = y+h
			xmin = x
			xmax = x+w
			if xmin < 1:
				xmin = 1
			if ymin < 1:
				ymin = 1
			_object = ET.SubElement(annotation, "object")
			ET.SubElement(_object, "name").text = name
			ET.SubElement(_object, "pose").text = pose
			ET.SubElement(_object, "truncated").text = truncated
			ET.SubElement(_object, "difficult").text = difficult
			ET.SubElement(_object, "obtainable").text = obtainable
			bndbox = ET.SubElement(_object, "bndbox")
			ET.SubElement(bndbox, "xmin").text = str(xmin)
			ET.SubElement(bndbox, "ymin").text = str(ymin)
			ET.SubElement(bndbox, "xmax").text = str(xmax)
			ET.SubElement(bndbox, "ymax").text = str(ymax)
	else:
		print "toXML: no objects received."

	annotation = prettify(annotation)
	with open(xml_file, 'w+') as xml:
		xml.write(annotation)
	# chage the permissions of the file, other users also need to be able to write
	os.chmod(xml_file,0664)
	#print "Write to xml file: "+xml_file
	return

def prettify(elem):
	"""Return a pretty-printed XML string for the Element.
	"""
	rough_string = ET.tostring(elem, 'utf-8')
	reparsed = minidom.parseString(rough_string)
	return reparsed.toprettyxml(indent="\t")

def get_annotation_BBox(annotation_file):
# Read read the bounding boxes from the annotations file
# and store these as objects in objects
	objects = []
	if os.path.exists(annotation_file):
		tree = ET.parse(annotation_file)
		root = tree.getroot()
		for obj in root.iter("object"):
			name       = obj.find('name').text
			bbox       = obj.find('bndbox')
			xmin       = int(bbox.find('xmin').text)
			ymin       = int(bbox.find('ymin').text)
			xmax       = int(bbox.find('xmax').text)
			ymax       = int(bbox.find('ymax').text)
			obtainable = int(bbox.findtext('obtainable', "0"))
			objects.append({
				"name"      : name,
				"bbox"      : (xmin,ymin,xmax-xmin,ymax-ymin),
				"obtainable" : obtainable == 1
			})
	else:
		print "Annotation file: " + os.path.basename(annotation_file) + " does not yet exist "
	
	return objects
