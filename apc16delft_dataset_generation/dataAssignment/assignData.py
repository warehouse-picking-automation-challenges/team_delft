# Assign validated images to training set and test set
# Images can be read from: imageSetDir/validated.txt

import os
import random
import itertools
from PATH import imageSetDir
from classes import classes

def split_image_set(image_set, num_class):
    """split an image set based on classes given the whole mixed image_set and number of class"""
    data_set = []
    for i in xrange(1,num_class+1):
        cls = "{0:0=2d}".format(i)
        ims = [im for im in image_set if im[:2]==str(cls)]
        data_set.append(ims)
    return data_set

def assign_data_set(data_set, train_pct):
    """assign data_set to training and test set given the whole data_set and percentage for training"""
    train_set = []
    test_set = []
    for cls_set in data_set:
        # get size of training set for the current class
        N = len(cls_set)
        ind = int(train_pct * N)
        # shuffle the current set
        random.shuffle(cls_set)
        # assign to training set and test set
        cls_trn_set = cls_set[:ind]
        cls_tst_set = cls_set[ind:]
        # append 
        train_set.append(cls_trn_set)
        test_set.append(cls_tst_set)
    # concatenate different classes
    train_set = list(itertools.chain(*train_set))
    test_set = list(itertools.chain(*test_set))
    return train_set, test_set

def readValImg(valtxt):
    with open(valtxt) as f:
    	vals = f.read().splitlines()
	return vals


def writeImgSet(image_set, set_txt):
    with open(set_txt, 'w+') as f:
        for im in image_set:
            f.write(im+'\n')

if __name__ == "__main__":
	numCls = len(classes)
	train_pct = 0.95

	valtxt = os.path.join(imageSetDir,'validated.txt')
	trainvaltxt = os.path.join(imageSetDir, 'trainval.txt')
	testtxt = os.path.join(imageSetDir, 'test.txt')

	images = readValImg(valtxt)

	data_set = split_image_set(images, numCls)
	train_set, test_set = assign_data_set(data_set, train_pct)

	writeImgSet(train_set, trainvaltxt)
	print 'Write {} successfully'.format(trainvaltxt)
	writeImgSet(test_set, testtxt)
	print 'Write {} successfully'.format(testtxt)
