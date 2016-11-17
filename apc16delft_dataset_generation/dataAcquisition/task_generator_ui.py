#!/usr/bin/env python
from gui_menu import main_menu
from dataValidation.captcha import object_selection
from apcshelf import bin_selection
import json

if __name__ == '__main__':
	menu = main_menu()
	command = menu.select()
	"""command code:
		0: start bringup
		1: start coordinator
		2: stop bringup
		3: stop coordinator
		4: generate picking order
		5: generate stowing order"""
	# print "command selected is {}".format(command)

	if command == 4:
		"""Create picking order"""

		terminated = False
		bin_contents = {}
		work_order   = []

		while not terminated:
			# select bin_name
			bins_occupied = bin_contents.keys()
			bin_menu = bin_selection(bins_occupied)
			bin_name = bin_menu.select()

			if bin_name != None:
				print "Bin selected is {}".format(bin_name)
			else:
				break
		
			# select contents in the bin
			content_menu = object_selection(multi=True)
			contents = content_menu.select()

			if contents != None and contents != []:
				print "Contents in {} is {}".format(bin_name, contents)
				bin_contents[bin_name] = contents
			else:
				break
		
			# select target menu from the bin
			target_menu = object_selection(multi=False)
			target_menu.read(contents)
			target = target_menu.select()

			if target != None and target != []:
				print "Target in {} is {}".format(bin_name, target)
				work_order.append({'bin':bin_name,'item':target[0]})
			else:
				break

			# terminate if all the bins are determined
			if len(bin_contents.keys()) >= 12:
				terminated = True

		# write task to files
		data = {'bin_contents': bin_contents, 'work_order': work_order, 'tote_contents': []}
		with open('../apc16delft_data/coordinator/apc_pick_task.json', 'w') as outfile:
			json.dump(data, outfile, sort_keys=True, indent=4, separators=(',',': '))
		print "Wrote to file {} successfully".format('apc_pick_task.json')
	
	
	if command == 5:
		"""Create Stowing task"""

		# select contents in the tote
		content_menu = object_selection(multi=True)
		tote_contents = content_menu.select()

		# write task to files
		data = {'bin_contents': {}, 'tote_contents': tote_contents}
		with open('../apc16delft_data/coordinator/apc_stow_task.json', 'w') as outfile:
			json.dump(data, outfile, sort_keys=True, indent=4, separators=(',',': '))
		print "Wrote to file {} successfully".format('apc_stow_task.json')
	
