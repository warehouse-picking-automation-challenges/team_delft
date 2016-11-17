# PATHS TO caffe-fast-rcnn/python AND caffe-fast-rcnn/lib must be added to $PYTHONPATHS

import os
os.environ['GLOG_minloglevel'] = '2' 

from fast_rcnn.config import cfg
from fast_rcnn.test import im_detect
from fast_rcnn.nms_wrapper import nms
import numpy as np
import caffe

import apc16delft_msgs.objects
from apc16delft_object_detection.apc_thresholds import score_thresholds

class BoundingBoxDetection():
	NETS = {'vgg16': ('apc.caffemodel')}

	DATA_DIR = os.path.join(os.environ['HOME'], 'classifier_test_data')
	
	def __init__(self, caffemodel, prototxt):
		cfg.TEST.HAS_RPN = True  # Use RPN for proposals

		if not os.path.isfile(caffemodel):
			raise IOError(('{:s} not found.').format(caffemodel))

		caffe.set_mode_gpu()
		#caffe.set_device(args.gpu_id)
		caffe.set_device(0)
		#cfg.GPU_ID = args.gpu_id
		cfg.GPU_ID = 0
		self.net = caffe.Net(prototxt, caffemodel, caffe.TEST)

		#print '\n\nLoaded network {:s}'.format(caffemodel)

		# Warmup on a dummy image
		im = 128 * np.ones((300, 500, 3), dtype=np.uint8)
		for i in xrange(2):
			_, _= im_detect(self.net, im)

		#print "##################\n Initialization complete \n#################" 

	def getObjectBoundingBox(self, im, filter_score=True, nms_threshold=0.3):
		# Detect all object classes and regress object bounds
		scores_, boxes_ = im_detect(self.net, im)

		# convert scores and boxes to dicts for ease of use
		scores = dict(enumerate(scores_.T))
		boxes  = np.reshape(boxes_, (scores_.shape[0], scores_.shape[1], 4))
		boxes  = dict(enumerate(np.transpose(boxes, (1, 0, 2)))) # define axes for transposing, otherwise it transposes wrong

		if nms_threshold > 0.0:
			for index, cls_scores, cls_boxes in zip(scores.keys(), scores.values(), boxes.values()):
				# skip background
				if index == 0:
					continue

				detections = np.hstack((cls_boxes, cls_scores[:, np.newaxis])).astype(np.float32)
				keep = nms(detections, nms_threshold)
				detections = detections[keep, :]

				scores.update({ index: detections[:, 4] })
				boxes.update({ index: detections[:, :4] })

		if filter_score:
			for index, cls_scores, cls_boxes in zip(scores.keys(), scores.values(), boxes.values()):
				# skip background
				if index == 0:
					continue

				# skip empty detections
				if len(scores) == 0 or len(boxes) == 0:
					continue

				name = apc16delft_msgs.objects.objectTypeToString[index]
				keep = np.where(cls_scores > score_thresholds[name])

				scores.update({ index: cls_scores[keep[0]] })
				boxes.update({ index: cls_boxes[keep[0], :] })

		return scores, boxes
