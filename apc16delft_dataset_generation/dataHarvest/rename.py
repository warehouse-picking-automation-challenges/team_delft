# !/usr/bin/python

import os
import argparse
from classes import inverse_classes

# parse directory
parser = argparse.ArgumentParser(description='Rename source image directories.')
parser.add_argument('srcDir', help='Path to source directories')
args = parser.parse_args()
# list directories to rename
dirs = os.listdir(args.srcDir)
print 'Directories to rename: {}'.format(dirs)

# renameing
for d in dirs:
	if d != 'background':
		d = int(d)
	if d in inverse_classes.keys():
		srcName = os.path.join(args.srcDir, str(d))
		print srcName
		tgtName = os.path.join(args.srcDir, inverse_classes[d])
		os.rename(srcName, tgtName)

# list directories after rename
print 'Directories after renaming is: {}'.format(os.listdir(args.srcDir))
