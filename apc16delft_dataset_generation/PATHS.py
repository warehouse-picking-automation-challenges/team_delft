# This is a configuration file that sets the paths
# this file must be linked to symbolically in each directory that uses this settings
# this is done by executing: 'ln -s ../PATHS.py PATHS.py' in the corresponding directory
# the symbolic link must then be imported in each script that uses these settings to get acces 
# to the variables that are set in the PATHS.py file
# 'import PATHS varname1, varname2, varname3'

# This file is located at:
# EWI/xander/gitFolder/

# The file that use these settings are located at:
# EWI/xander/gitFolder/subfolder

# The folders of interest are located in:
# EWI/py-faster-rcnn/data.....

# get username, this is part of the path
import getpass
user = getpass.getuser()

# Directory with the originally recorded *.ppm images
jpegOriginalDir = '/home/%s/DRapc2016/APC2016/Originals/'%user
# use the one below now!
originalDir = '/home/%s/DRapc2016/APC2016/Originals/'%user

# Directory that contains the preprocessed images, the *ppm image are converted to JPEG
jpegDir = '/home/%s/DRapc2016/APC2016/Images/'%user
# use this one now for the jpegDir:
imageDir = '/home/%s/DRapc2016/APC2016/Images/'%user

# Directory that contains the segmented images
segmentationDir = '/home/%s/DRapc2016/APC2016/Segmentations/'%user

# Directory with the annotations that result from the preprocessing. The annotations contain information about the data present in the images (widht, height, bounding box, ..)
xmlDir = '/home/%s/DRapc2016/APC2016/Annotations/'%user

# Directory with the annotations that result from the preprocessing. The annotations contain information about the data present in the images (widht, height, bounding box, ..)
imageSetDir ='/home/%s/DRapc2016/APC2016/ImageSets/Main/'%user

# Directory with the originally recorded *.ppm images for test set
testoriginalDir = '/home/%s/DRapc2016/APCtest/Originals/'%user

# Directory to the test interface files
testinterfaceDir = '/home/%s/DRapc2016/APCtest/Interface'%user

# Directory that contains the preprocessed images for test set, the *ppm image are converted to JPEG
testjpegDir = '/home/%s/DRapc2016/APCtest/Images/'%user
# use the one below now!
testimageDir = '/home/%s/DRapc2016/APCtest/Images/'%user

# Directory that contains the segmented images
testsegmentationDir = '/home/%s/DRapc2016/APCtest/Segmentations/'%user

# Directory with the annotations that result from the preprocessing for test set. The annotations contain information about the data present in the images (widht, height, bounding box, ..)
testxmlDir = '/home/%s/DRapc2016/APCtest/Annotations/'%user

# Directory with the annotations that result from the preprocessing for test set. The annotations contain information about the data present in the images (widht, height, bounding box, ..)
testimageSetDir ='/home/%s/DRapc2016/APCtest/ImageSets/Main/'%user

# Directory with random background images
augmentationDir = '/home/%s/VOCdevkit2007/VOC2007/JPEGImages/'%user
