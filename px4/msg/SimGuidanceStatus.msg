# Off-board control mode

uint64 timestamp		# time since system start (microseconds)

bool started 	# received request to engage guidance
bool loaded 	# finished loading trajectory
bool executing 	# started trajectory execution / evaluation
bool finished 	# completed trajectory
bool trajectory_valid

