# Primary states
int8 UNCONFIGURED = 0
int8 INACTIVE     = 1
int8 ACTIVE       = 2
int8 FINALIZED    = 3

# Transition states
int8 CONFIGURING      = 4
int8 CLEANING_UP      = 5
int8 SHUTTING_DOWN    = 6
int8 ACTIVATING       = 7
int8 DEACTIVATING     = 8
int8 ERROR_PROCESSING = 9

int8 state          # The new state of the lifecycle node.
int8 old_state      # The previous state of the lifecycle node.
int32 result_code   # The result code of the transition.
