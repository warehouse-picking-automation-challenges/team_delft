#!/bin/bash
for i in {0..499}; do
	roslaunch apc16delft_coordinator pick_test.launch task_file:="$PWD/apc_pick_task_$i.json" || exit
done
