#!/usr/bin/env python
import copy
import json
import random
import os
import sys
import cv2
import numpy as np
from classes import classes, classes_short, inverse_classes_short, getShortName
from PATHS import testinterfaceDir
from dataValidation.captcha import object_selection

'''
    This script generates two Task Input Files [1] designed to be used for
    testing against the pick and stow tasks defined in the Amazon Picking
    Challenge 2016 rules.

    Note: The script distributes items randomly, which is not be a perfect
    representation of an actual pick/stow task. During the actual competition,
    the item distribution will be chosen to present an array of difficulties
    amongst the bins. Considerations will be made (i.e. for object size,
    duplicates, point value) that are not reflected in this generator.


    The 'apc_pick_task.json' file will contain the following json objects:
        - 'bin_contents' will contain the mapping of each bin to its contents,
          as they exist prior to a task attempt.
        - 'tote_contents' will be an empty list, to properly reflect that the
          tote is empty at the beginning of the task.
        - 'work_order' will contain a mapping of each bin to the item that is
          expected to be picked from that bin.

        After a pick task attempt, it is expected that the same file format
        will be returned to the judges. The 'bin_contents' object should
        appropriately reflect the new status of the bins' contents. The
        'tote_contents' list should contain the objects that were correctly
        placed in the tote. The 'work_order' object is ignored.

    The 'apc_stow_task.json' file will contain the following json objects:
        - 'bin_contents' will contain the mapping of each bin to its contents,
          as they exist prior to a task attempt.
        - 'tote_contents' will contain 12 items that are expected to be stowed
          into the bins.

        After a stow task attempt, it is expected that the same file format
        will be returned to the judges. The 'bin_contents' object should
        appropriately reflect the new status of the bins' contents. The
        'tote_contents' list should also appropriately reflect the new status
        of the bins' contents. Items that were not removed from the tote during
        the attempt should be remain in this list.

    [1] http://amazonpickingchallenge.org/APC_2016_Official_Rules.pdf,
        section 'Task Attempt Rules', paragraph 2.
'''

#-------------------------------------------------------------------------------
class interface_generator():
    def __init__(self):
        # define our bin and item names to use
        self.CONST_BIN_NAMES = ["bin_A", "bin_B", "bin_C", "bin_D", "bin_E", "bin_F", "bin_G", "bin_H", "bin_I", "bin_J", "bin_K", "bin_L"]
        self.CONST_ITEM_NAMES = ["i_am_a_bunny_book",
                                "laugh_out_loud_joke_book",
                                "scotch_bubble_mailer",
                                "scotch_bubble_mailer",
                                "up_glucose_bottle",
                                "dasani_water_bottle",
                                "dasani_water_bottle",
                                "rawlings_baseball",
                                "folgers_classic_roast_coffee",
                                "elmers_washable_no_run_school_glue",
                                "elmers_washable_no_run_school_glue",
                                "hanes_tube_socks",
                                "womens_knit_gloves",
                                "cherokee_easy_tee_shirt",
                                "peva_shower_curtain_liner",
                                "cloud_b_plush_bear",
                                "barkely_hide_bones",
                                "kyjen_squeakin_eggs_plush_puppies",
                                "cool_shot_glue_sticks",
                                "creativity_chenille_stems",
                                "creativity_chenille_stems",
                                "soft_white_lightbulb",
                                "safety_first_outlet_plugs",
                                "oral_b_toothbrush_green",
                                "oral_b_toothbrush_red",
                                "dr_browns_bottle_brush",
                                "command_hooks",
                                "easter_turtle_sippy_cup",
                                "fiskars_scissors_red",
                                "scotch_duct_tape",
                                "scotch_duct_tape",
                                "woods_extension_cord",
                                "platinum_pets_dog_bowl",
                                "fitness_gear_3lb_dumbbell",
                                "rolodex_jumbo_pencil_cup",
                                "clorox_utility_brush",
                                "kleenex_paper_towels",
                                "expo_dry_erase_board_eraser",
                                "expo_dry_erase_board_eraser",
                                "kleenex_tissue_box",
                                "ticonderoga_12_pencils",
                                "crayola_24_ct",
                                "jane_eyre_dvd",
                                "dove_beauty_bar",
                                "staples_index_cards",
                                "staples_index_cards"]


    # This function uses the bin_size_count map to randomly arrange items into bins.
    def generateBinContents(self, bin_size_count):
	bin_contents = {bin_name:[] for bin_name in self.CONST_BIN_NAMES}

	# Create destroyable copies of items and bins
	items = copy.deepcopy(self.CONST_ITEM_NAMES)
	abins = copy.deepcopy(self.CONST_BIN_NAMES)

	# Generate all bins into bin_contents variable
	for bin_size in bin_size_count:
		bin_count = bin_size_count[bin_size]
		for ii in range(0, bin_count):
			bin_name = random.choice(abins)
			abins.remove(bin_name)
			for jj in range(0, bin_size):
				item_name = random.choice(items)
				items.remove(item_name)
				bin_contents[bin_name].append(item_name)
	return bin_contents


    # generate the bin contents data structure
    def draw_pick_task(self):
		# bin_size_count is a mapping of bin "sizes" to number of bins of that size.
		# key is size of bin contents, value is number of bins.
		# i.e. {2: 3} represents that there are 3 bins that hold 2 items each.
		bin_size_count = {}

		# bin_contents describes the contents of each bin on a pod.
		# key is bin name (from self.CONST_BIN_NAMES), value is a list of items (from CONST_ITEM_NAMES)
		bin_contents = {}

		#-------------------------------------------------------------------------------
		# Generate bin contents for the pick pod
		bin_size_count[1] = 1
		bin_size_count[2] = 3
		bin_size_count[3] = 2
		bin_size_count[4] = 3
		bin_size_count[5] = 1
		bin_size_count[7] = 1
		bin_size_count[9] = 1

                bin_contents = self.generateBinContents(bin_size_count)

		# Generate the work order data structure
		work_order = [{'bin':bin_name,'item':item_name} for bin_name in self.CONST_BIN_NAMES
	                      for item_name in (bin_contents[bin_name][0:1])]

		# Picking task begins with an empty tote
		tote_contents = []

		# Write data to appropriately-named json file
		data = {'bin_contents': bin_contents, 'work_order': work_order, 'tote_contents': tote_contents}
                return data

#-------------------------------------------------------------------------------
# Generate bin contents for the stow pod
    def draw_stow_task(self):
        bin_size_count = {}
        bin_size_count[2] = 3
        bin_size_count[3] = 4
        bin_size_count[4] = 2
        bin_size_count[6] = 2
        bin_size_count[8] = 1

        bin_contents = self.generateBinContents(bin_size_count)

        # Stowing task begins with 12 items in tote
        tote_contents = []
        for bin_name in self.CONST_BIN_NAMES:
            tote_contents.append(bin_contents[bin_name].pop())

        # Write data to appropriately-named json file
        data = {'bin_contents': bin_contents, 'tote_contents': tote_contents}

        return data


#-------------------------------------------------------------------------------
if __name__ == "__main__":
	interface = interface_generator()
	pick_tasks = []
	stow_tasks = []
	num_tasks = 100
	DRAW = False
	for task_id in range(0, num_tasks):
		if DRAW:
			# Do the work for the picking task
			data = interface.draw_pick_task()
			pick_tasks.append(data)
#			with open(os.path.join(testinterfaceDir,'apc_pick_task_{}.json'.format(task_id)), 'w') as outfile:
#				json.dump(data, outfile, sort_keys=True, indent=4, separators=(',',': '))

			data = interface.draw_stow_task()

			stow_tasks.append(data)
#			with open(os.path.join(testinterfaceDir,'apc_stow_task_{}.json'.format(task_id)), 'w') as outfile:
#				json.dump(data, outfile, sort_keys=True, indent=4, separators=(',',': '))

		else:
			# Read pick json file
			json_data = open(os.path.join(testinterfaceDir,'apc_pick_task_{}.json'.format(task_id)))
			pick_data = json.load(json_data)
			pick_tasks.append(pick_data)
			# Read stow json file
			json_data = open(os.path.join(testinterfaceDir,'apc_stow_task_{}.json'.format(task_id)))
			stow_data = json.load(json_data)
			stow_tasks.append(stow_data)


	# Visualize the interfaces
	# show the bin contents
	task_name  = int(sys.argv[1])
	task_start = int(sys.argv[2])
	if task_name == 1:
		tasks = stow_tasks
		for task_id in range(task_start, len(tasks)):
			objects = tasks[task_id]['tote_contents']
			objects = [inverse_classes_short[classes[obj]] for obj in objects]
			#objects = sorted(objects)
			randind = np.random.randint(1,13)
			os.system('clear')
			print '\n\n\n'
			print '===============task ID: {} tote contents==============='.format(task_id)
			print '||'
			for elem in objects[:randind]:
				print '||',elem
			#print '||   {}'.format(objects[0:randind])
			print '||'
			print 'Totoal: {} objects'.format(randind)
			print '||'
			print '======================================================='
			raw_input("Press Enter to continue...")
	else:
		tasks = pick_tasks
		bin_id = int(sys.argv[3])
		#-------------------------------------------------------------------------------
		for bin_name in interface.CONST_BIN_NAMES[bin_id:]:
			for task_id in range(task_start, len(tasks)):
				objects = tasks[task_id]['bin_contents'][bin_name]
				objects = [inverse_classes_short[classes[obj]] for obj in objects]
				os.system('clear')
				print '\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n'
				print '===============task ID: {} bin: {}==============='.format(task_id, bin_name)
				print '||'
#				print '||   {}'.format(objects)
				for elem in sorted(objects):
					print '||',elem
				print '|| Objects in total: {}'.format(len(objects))
				print '||'
				print '================================================='
				raw_input("Press Enter to continue...")
#				captcha = object_selection()
#				grids = captcha.get_grids()
#				object_grids = captcha.get_objects()
#				items = [[getShortName(object_grids[i][j]) if object_grids[i][j] != None else object_grids[i][j] for j in range(0,6)] for i in range(0,7)]
#				for i, item in enumerate(objects):
#					cls_id = classes_short[item]
#					i, j = [(i, x.index(item)) for i, x in enumerate(items) if item in x][0]
#					grid = grids[i][j]
#					im = captcha.img
#					im_cls = im[grid[1]:grid[3],grid[0]:grid[2]]
#					cv2.namedWindow(item, cv2.WINDOW_NORMAL)
#					cv2.imshow(item, im_cls)
#					cv2.waitKey(50)
#				k = cv2.waitKey(0)
#				# quit
#				if k == ord('q'):
#					sys.exit()
#				# next item in the bin
#				elif k == ord(' '):
#					cv2.destroyAllWindows()
#					continue
	
