#!/usr/bin/env python

import cv2
import os

from apc16delft_object_detection.DetectObjectBoundingBox import BoundingBoxDetection

# initialization
tester = BoundingBoxDetection('/home/xgerrmann/DRapc2016/models')

# testing
tester.test()

# #
# image = os.path.join(os.environ['HOME'], 'classifier_test_data/test/000456.jpg')
# print image
# im = cv2.imread(image)
# scores, boxes = tester.getObjectBoundingBoxes(im)

# print '### Scores:'
# print scores
# print '### Boxes:'
# print boxes
