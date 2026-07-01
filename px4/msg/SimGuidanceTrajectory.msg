# Off-board control mode

uint64 timestamp		# time since system start (microseconds)

float32 time_s
uint8 n_dofs #n_dofs < 4, stands for valid entries in vectors below:

#order: [x y z yaw]
float32[4] position
float32[4] velocity
float32[4] acceleration
float32[4] jerk
float32[4] snap
