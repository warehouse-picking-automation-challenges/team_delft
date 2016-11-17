int32 STRATEGY_VACUUM_H   = 0
int32 STRATEGY_VACUUM_V   = 1
int32 VACUUM_H_BIG        = 2
int32 VACUUM_V_BIG        = 3
# BEWARE, LEAVE THE NUMBERS UNTOUCHED FROM HERE UNTIL THE NEXT COMMENT!
int32 DUMBBELL_INWARDS    = 4
int32 DUMBBELL_SIDEWARDS  = 5
int32 PENCIL_CUP_STANDING = 6
int32 PENCIL_CUP_SIDEWAYS = 7
int32 PENCIL_CUP_INWARDS  = 8
# You may add more strategies from here, but don't touch the ones between this and the previous comment.

float64 score                           # The score of the candiate. Higher is better (0 to 1).
int32 strategy                          # The grasp strategy for the candidate.
geometry_msgs/PoseStamped stamped_pose  # The candidate pose.
float64 pre_grasp_offset                # The pre-grasp-offset.
bool angled_approach                    # Determines whether we do an angled approach or not.
bool side_angled_approach               # Determines whether we do a side-angled approach or not.
