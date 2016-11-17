import time
import rospy
import job as Job
import traceback
from objects import loadItemBonuses, loadItemSize, loadItemFragility, loadItemWeight
from parse_json import loadPickDescription, loadStowDescription
from apc16delft_msgs.objects import objectTypeToString
from apc16delft_msgs.msg import Object
from print_json import saveStateAsJson

def stowPoints(items_in_bin):
	if items_in_bin > 4:
		return 20
	elif items_in_bin > 3:
		return 15
	else:
		return 10

def lastOwnResult(history):
	for result in reversed(history):
		if not isinstance(result, Job.DependencyFailed):
			return result
	return None

def try_remove(collection, value):
	try:
		collection.remove(value)
	except ValueError:
		pass

tote_x_count = 2
tote_y_count = 3
tote_cell_count = tote_x_count * tote_y_count

items = rospy.get_param("/item/")

MOVE_BLACKLIST = [Object.FITNESS_GEAR_3LB_DUMBBELL, Object.ROLODEX_JUMBO_PENCIL_CUP, Object.KLEENEX_PAPER_TOWELS, Object.HANES_TUBE_SOCKS]
UNOCCLUDABLE   = [Object.KLEENEX_PAPER_TOWELS, Object.HANES_TUBE_SOCKS]

def getItemProperties(object_type):
	return items[objectTypeToString[object_type]]

def isGraspOcclusion(result):
	return (isinstance(result, Job.NoPlan) or isinstance(result, Job.NoGrasp)) and result.grasp_occluders is not None

def cellFull(contents):
	sizes = [getItemProperties(item)['size'] for item in contents]
	if 'large' in sizes:
		return True
	elif sizes.count('medium') > 1:
		return True
	elif sizes.count('small') > 3:
		return True
	elif sizes.count('medium') == 1 and sizes.count('small') > 2:
		return True
	else:
		return False


def cellFragility(contents):
	fragilities = [ getItemProperties(x)['fragility'] for x in contents]
	if 'yes' in fragilities :
		return 'yes'
	elif 'medium' in fragilities :
		return 'medium'
	else:
		return 'no'


def binOccupancy(contents):
# based on discrete sizes: beware to update in stow_params
# a better version would use item bounding box volume (check Wilson code)
	sizes = [getItemProperties(x)['size'] for x in contents]
	return sizes.count('small') + sizes.count('medium')*2 + sizes.count('large')*5


def hasLarge2Stow(contents):
	items = [objectTypeToString[i] for i in contents]
	if 'kleenex_paper_towels' in items :
		return True
	if 'hanes_tube_socks' in items :
		return True
	else:
		return False

def hasLarge(contents):
	sizes = [ getItemProperties(x)['size'] for x in contents]
	if 'large' in sizes :
		return True
	else:
		return False


def binsForPinched():
	bins_stow_pinch = range(12) # TODO change to those actually possible
	return bins_stow_pinch



class PickTaskPlanner():
	def __init__(self, json_file):
		bin_contents, pending_orders, tote_contents = loadPickDescription(json_file)
		self.current_order      = None
		self.completed_orders   = []
		self.failed_orders      = []
		self.tote_contents      = tote_contents
		self.start_bin_contents = bin_contents
		self.emptied_bins       = set()
		self.bin_contents       = list(bin_contents)
		self.item_bonuses       = loadItemBonuses()
		self.pending_orders     = sorted(pending_orders, key = lambda order: (len(self.bin_contents[order.source_bin]), -1 * self.item_bonuses[order.object_type]))
		self.output_file_no     = 1

		self.tote_cells = [[] for i in range(tote_cell_count)]

		for order in self.pending_orders:
			order.grasp_occlusions = 0
			order.next_grasp_occluder = 0

		self.free_cells			= range(tote_cell_count)

		targets = [ x.object_type for x in pending_orders]

		# CASE PICH GRASP: blocks up to 2 tote cells
		if Object.FITNESS_GEAR_3LB_DUMBBELL in targets:
			try_remove(self.free_cells, 2)
			rospy.loginfo('DUMBBELL -- booked cell 2')
		if Object.ROLODEX_JUMBO_PENCIL_CUP in targets:
			try_remove(self.free_cells, 1)
			rospy.loginfo('PENCIL CUP -- booked cell 1')

	def taskDone(self, result):
		order = self.current_order
		job   = self.current_job
		order.history.append(result)

		if isinstance(job, Job.Move) and order.grasp_occlusions >= 2:
			order.grasp_occlusions = 1
			rospy.loginfo('Grasp occlusion count reset {}'.format(order.grasp_occlusions))

		if isinstance(result, Job.Success):
			self.bin_contents[job.source_bin].remove(job.object_type)
			if isinstance(job, Job.Pick):
				self.completed_orders.append(order)
				self.tote_contents.append(job.object_type)
				self.tote_cells[job.target_tote_cell].append(job.object_type)
				self.emptied_bins.add(job.source_bin)
			elif isinstance(job, Job.Move):
				self.bin_contents[job.target_bin].append(job.object_type)
				self.pending_orders.append(order)
			else:
				raise Exception("Unexpected job type.")
		else:

			add_to_failed = True
			if isinstance(job, Job.Move):
				order.history.append(Job.DependencyFailed(result))
			elif job.object_type in UNOCCLUDABLE and isinstance(result, Job.NoDetection):
				self.completed_orders.append(job)
				self.emptied_bins.add(job.source_bin)
				add_to_failed = False
			elif isGraspOcclusion(result):
				order.grasp_occlusions += 1
				rospy.loginfo('Grasp occlusion count now at {}'.format(order.grasp_occlusions))
				order.grasp_occluders = result.grasp_occluders

			if add_to_failed:
				# Never give up! Never surrender!
				self.failed_orders.append(order)

		self.saveState()
		self.current_order = None

	def saveState(self):
		remaining_orders = self.pending_orders + self.failed_orders
		saveStateAsJson('output_task_pick_{}_{}.json'.format(time.strftime('%Y-%m-%d-%H-%M-%S'), self.output_file_no), self.bin_contents, self.tote_contents, remaining_orders)
		self.output_file_no += 1

	def nextJob(self):
		order = None
		job   = None

		if self.pending_orders:
			order = self.pending_orders.pop(0)
			job   = Job.Pick(order.object_type, order.source_bin, self.selectToteCell(order.object_type))

		elif self.failed_orders:
			order, job = self.selectFailed()
			# Replace move jobs for blacklisted items with pick jobs for their target.
			if isinstance(job, Job.Move) and job.object_type in MOVE_BLACKLIST:
				job = Job.Pick(order.object_type, order.source_bin, self.selectToteCell(order.object_type))

		self.current_order = order
		self.current_job   = job
		return self.current_job

	def selectFailed(self):
		front  = self.failed_orders.pop(0)
		result = lastOwnResult(front.history)
		if (isinstance(result, Job.Occluded) or isinstance(result, Job.NoDetection)) and result.occluder is not None:
			return front, Job.Move(result.occluder, front.source_bin, self.selectPlacementBin())
		elif front.grasp_occlusions >= 2 and front.grasp_occluders:
			occluder = self.selectGraspOccluder(front)
			return front, Job.Move(occluder, front.source_bin, self.selectPlacementBin())
		else:
			return front, Job.Pick(front.object_type, front.source_bin, self.selectToteCell(front.object_type))

	def selectGraspOccluder(self, order):
		if not order.grasp_occluders:
			return None
		index = order.next_grasp_occluder
		if index >= len(order.grasp_occluders):
			index = 0
		selection = sorted(order.grasp_occluders, key = lambda x: x.centroid.z)[index]
		order.next_grasp_occluder = index + 1
		return selection.object.type

	def selectPlacementBin(self):
		key = lambda x: len(self.bin_contents[x])
		if not self.emptied_bins:
			return min(range(12), key = key)
		return min(self.emptied_bins, key = key)


	def selectToteCell(self, object_type):
		try:
			tote_cell = self.__selectToteCell(object_type)
		except Exception as e:
			rospy.logerr('Error selecting target bin: {}'.format(str(e)))
			traceback.print_exc()
			try:
				tote_cell = min( range(6), key = lambda x: len(self.tote_cells[x]) )
			except Exception as e:
				rospy.logerr('Error selecting fallback tote cell: {}'.format(str(e.trace)))
				traceback.print_exc()
				tote_cell = 1
		if tote_cell is None:
			rospy.logerr('No cell selected, falling back to 1.')
			tote_cell = 1
		return tote_cell



	def __selectToteCell(self, object_type):

		# SPECIAL CASES ################################################
		if object_type == Object.FITNESS_GEAR_3LB_DUMBBELL :
			self.free_cells.append(2)
			rospy.loginfo('DUMBBELL -- in 2')
			return 2
		if object_type == Object.ROLODEX_JUMBO_PENCIL_CUP :
			rospy.loginfo('PENCIL CUP')
			if 2 in self.free_cells:
				self.free_cells.append(1)
				return 2
			else:
				self.free_cells.append(1)
				return 1


		# DEFAULT #######################################################
		fragility 	= loadItemFragility(object_type)
		weight 		= loadItemWeight(object_type)

		empty_cells = filter(lambda x: len(self.tote_cells[x]) == 0, self.free_cells)

		if not empty_cells: # no empty cells
			rospy.loginfo('NO EMPTY cells')
			if weight == 'high':
				no_fragile_places = filter(lambda x: cellFragility(self.tote_cells[x]) is 'no', self.free_cells)
				if not no_fragile_places:
					medium_frag = filter(lambda x: cellFragility(self.tote_cells[x]) is 'medium', self.free_cells)
					if not medium_frag :
						return min(available_cells, key = lambda x: len(self.tote_cells[x]))
					else:
						return min(medium_frag, key = lambda x: len(self.tote_cells[x]))

				else:
					return min(no_fragile_places, key = lambda x: len(self.tote_cells[x]))

			else:
				# Filter out full cells.
				available_cells = filter(lambda x: not cellFull(self.tote_cells[x]), self.free_cells)

				if not available_cells:
					places = filter(lambda x: not hasLarge(self.tote_cells[x]), self.free_cells)
					if not places:
						return min(self.free_cells, key = lambda x: len(self.tote_cells[x]))
					else:
						return min(places, key = lambda x: len(self.tote_cells[x]))
						

				else: # AVAILABLE cells
					if fragility == 'yes':
						return max(available_cells, key = lambda x: len(self.tote_cells[x]))
					else:
						return min(available_cells, key = lambda x: len(self.tote_cells[x]))


		else: # EMPTY cells  format(result.object_type, job.target_bin)
			
			# heavy or no fragility staff on one side
			if weight == 'high' or fragility == 'no' :
				result = max(empty_cells, key = lambda x: x % 3)
				rospy.loginfo('EMPTY cell {} for {}'.format(result, object_type))
				return result

			else: # other staff on the other side
				result = min(empty_cells, key = lambda x: x % 3)
				rospy.loginfo('EMPTY cell {} for {}'.format(result, object_type))
				return result




class StowTaskPlanner():
	def __init__(self, json_file):
		bin_contents, tote_contents = loadStowDescription(json_file)
		self.initial_bin_contents   = list(bin_contents)
		self.bin_contents           = bin_contents
		self.tote_contents          = tote_contents
		self.initial_tote_contents	= list(tote_contents)
		self.output_file_no         = 1
		self.blacklist              = []
		self.free_bins				= range(12)
		self.dumbbell_bin			= None
		self.big_items_bins			= []
		self.t_shirt_bin			= None
		self.mailer_bin				= None


		try:
			# Eliminate bins with SOCKS and PAPER TOWELS
			self.free_bins = filter(lambda x: not Object.HANES_TUBE_SOCKS in self.initial_bin_contents[x], self.free_bins)
			self.free_bins = filter(lambda x: not Object.KLEENEX_PAPER_TOWELS in self.initial_bin_contents[x], self.free_bins)

			if Object.CHEROKEE_EASY_TEE_SHIRT in self.initial_tote_contents :
				t = [9,10,11]
				t = filter(lambda x: not Object.KLEENEX_PAPER_TOWELS in self.initial_bin_contents[x], t)
				self.t_shirt_bin = max(t, key = lambda x: stowPoints( len(self.bin_contents[x]) ))

			for_bigs = [1,4,7,10]

			if Object.HANES_TUBE_SOCKS in self.initial_tote_contents :
				n = filter(lambda x: binOccupancy(self.bin_contents[x]) < 3, for_bigs)
				if not n:
					b = min(self.free_bins, key = lambda x: len(self.bin_contents[x]))
				else:
					b = min(n, key = lambda x: len(self.bin_contents[x]))

				self.big_items_bins.append(b)
				try_remove(self.free_bins, b)
				try_remove(for_bigs, b)

			if Object.KLEENEX_PAPER_TOWELS in self.initial_tote_contents :
				n = filter(lambda x: binOccupancy(self.bin_contents[x]) < 3, for_bigs)
				if not n:
					b = min(self.free_bins, key = lambda x: len(self.bin_contents[x]))
				else:
					b = min(n, key = lambda x: len(self.bin_contents[x]))

				self.big_items_bins.append(b)
				try_remove(self.free_bins, b)


			if Object.SCOTCH_BUBBLE_MAILER in self.initial_tote_contents :
				self.mailer_bin = max([0,1,2,9,10,11], key = lambda x: stowPoints(len(self.initial_bin_contents[x])))
				# self.free_bins.remove(self.mailer_bin)

			#if Object.FITNESS_GEAR_3LB_DUMBBELL in self.initial_tote_contents :
			#	n = filter(lambda x: fragility(self.bin_contents[x]) =='no', self.free_bins)
			#	if not n:
			#		m = filter(lambda x: fragility(self.bin_contents[x]) =='medium', self.free_bins)
			#		if not m :
			#			self.dumbbell_bin = min(self.free_bins, key = lambda x: len(self.initial_bin_contents[x]))
			#		else:
			#			self.dumbbell_bin = min(m, key = lambda x: len(self.initial_bin_contents[x]))
			#	else:
			#		self.dumbbell_bin = max(n, key = lambda x: stowPoints(len(self.initial_bin_contents[x])))

			#	self.free_bins.remove(self.dumbbell_bin)
			self.dumbbell_bin = 7


			rospy.loginfo( '\nFREE BINS: %s \n', self.free_bins )

			# prepare lists of not-blocked bins:
			self.bins_10 = filter(lambda x: stowPoints(len(self.initial_bin_contents[x])) == 10, self.free_bins)
			self.bins_15 = filter(lambda x: stowPoints(len(self.initial_bin_contents[x])) == 15, self.free_bins)
			self.bins_20 = filter(lambda x: stowPoints(len(self.initial_bin_contents[x])) == 20, self.free_bins)
		except Exception as e:
			rospy.logerr('Error during task planner initialization: {}'.format(str(e)))
			traceback.print_exc()


	def taskDone(self, result):
		job = self.current_job
		self.current_job = None

		if isinstance(result, Job.Success):
			raise Exception("Stow jobs should never report regular success. They must report StowSuccess with an object type.")

		if isinstance(result, Job.StowSuccess):
			rospy.loginfo("Successfully stowed item {} in bin {}".format(result.object_type, job.target_bin))
			try_remove(self.tote_contents, result.object_type)
			self.bin_contents[job.target_bin].append(result.object_type)

		elif isinstance(result, Job.NoPlan) or isinstance(result, Job.NoGrasp) or isinstance(result, Job.NoObjectPose):
			self.blacklist.append(result.object_type)

		elif isinstance(result, Job.NoDetection):
			self.blacklist = []

		if len(self.filteredToteContents()) == 0:
			self.blacklist = []

		self.saveState()

	def filteredToteContents(self):
		return filter(lambda x: x not in self.blacklist, self.tote_contents)

	def nextJob(self):
		# If the tote is empty we're done.
		if not self.tote_contents:
			return None

		self.current_job = Job.Stow(None, None)
		return self.current_job


	def __selectTargetBin(self, item):
		#specific items
		if item == Object.FITNESS_GEAR_3LB_DUMBBELL:
			return self.dumbbell_bin

		if item == Object.CHEROKEE_EASY_TEE_SHIRT :
			return self.t_shirt_bin

		if item == Object.HANES_TUBE_SOCKS:
			return self.big_items_bins.pop()

		if item == Object.KLEENEX_PAPER_TOWELS:
			return self.big_items_bins.pop()

		if item == Object.SCOTCH_BUBBLE_MAILER:
			return self.mailer_bin

		# general rules
		limit_occupancy_medium	= 15
		limit_occupancy_large	=  7

		# be safe
		rospy.loginfo("bins_20: {}, bins_15: {}, bins_10: {}".format(len(self.bins_20), len(self.bins_15), len(self.bins_10)))
		if not self.bins_20 and not self.bins_15:
			rospy.loginfo('bins20 and bins15 empty')
			if not self.bins_10:
				return min(range(12), key = lambda x: binOccupancy(self.bin_contents[x]))
			else:
				return min(self.bins_10, key = lambda x: binOccupancy(self.bin_contents[x]))

		# DEFAULT
		if loadItemSize(item) == 'small' :
			if not self.bins_20:
				b = min(self.bins_15, key = lambda x: binOccupancy(self.bin_contents[x]))
				if binOccupancy(self.bin_contents[b]) < limit_occupancy_medium:
					return b
				elif self.bins_15:
					return min(self.bins_15, key = lambda x: binOccupancy(self.bin_contents[x]))
				else:
					return min(self.bins_10, key = lambda x: binOccupancy(self.bin_contents[x]))

			return min(self.bins_20, key = lambda x: len(self.bin_contents[x]))

		if loadItemSize(item) == 'medium' :
			if not self.bins_20:
				return min(self.bins_15, key = lambda x: binOccupancy(self.bin_contents[x]))

			b = min(self.bins_20, key = lambda x: binOccupancy(self.bin_contents[x]))
			if binOccupancy(self.bin_contents[b]) < limit_occupancy_medium:
				return b
			elif self.bins_15:
				return min(self.bins_15, key = lambda x: binOccupancy(self.bin_contents[x]))
			else:
				return min(self.bins_10, key = lambda x: binOccupancy(self.bin_contents[x]))

		if loadItemSize(item) == 'large' :
			if not self.bins_20:
				return min(self.bins_15, key = lambda x: binOccupancy(self.bin_contents[x]))

			b = min(self.bins_20, key = lambda x: binOccupancy(self.bin_contents[x]))
			if binOccupancy(self.bin_contents[b]) < limit_occupancy_large:
				return b
			else:
				if self.bins_15:
					return min(self.bins_15, key = lambda x: binOccupancy(self.bin_contents[x]))
				else:
					return min(self.bins_10, key = lambda x: binOccupancy(self.bin_contents[x]))


	def selectTargetBin(self, item):
		self.current_job.object_type = item
		try:
			self.current_job.target_bin = self.__selectTargetBin(item)
		except Exception as e:
			rospy.logerr('Error selecting target bin: {}'.format(str(e)))
			traceback.print_exc()
			try:
				self.current_job.target_bin = min(range(12), key = lambda x: min(self.bin_contents[x]))
			except Exception as e:
				rospy.logerr('Error selecting fallback target bin: {}'.format(str(e.trace)))
				traceback.print_exc()
				self.current_job.target_bin = 1
		if self.current_job.target_bin is None:
			rospy.logerr('No bin selected, falling back to 1.')
			self.current_job.target_bin = 1
		return self.current_job.target_bin



	def saveState(self):
		tote_contents = self.tote_contents
		saveStateAsJson('output_task_stow_{}_{}.json'.format(time.strftime('%Y-%m-%d-%H-%M-%S'), self.output_file_no), self.bin_contents, tote_contents, None)
		self.output_file_no += 1
