# Off-board control mode

uint64 timestamp		# time since system start (microseconds)

bool start 			# engage guidance module
bool reset			# request reset
bool start_execution		# request to start a trajectory execution
bool set_home			# request set_home at current state
bool stop			# stop execution



# Commands are consumed in order; results are published on sim_guidance_status.
uint8 ORB_QUEUE_LENGTH = 8
