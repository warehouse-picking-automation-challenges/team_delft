import json
from bins import binIndexToString, binIndexFromString
from apc16delft_msgs.objects import objectTypeToString

def saveStateAsJson(filename, bin_contents, tote_contents, remaining_jobs):

	bin_contents_named = {}

	for index in range(12):
		bin_contents_named[binIndexToString[index]] = [objectTypeToString[x] for x in bin_contents[index]]

	tote_contents = [objectTypeToString[x] for x in tote_contents]

	if remaining_jobs is not None:
		remaining_jobs = [{'item': objectTypeToString[job.object_type], 'bin': binIndexToString[job.source_bin]} for job in remaining_jobs]
		data = {'bin_contents' : bin_contents_named, 'tote_contents' : tote_contents, 'work_order': remaining_jobs}
	else:
		data = {'bin_contents' : bin_contents_named, 'tote_contents' : tote_contents}

	with open(filename, 'w') as outfile:
		json.dump(data, outfile, sort_keys=True, indent=4, separators=(',',': '))
