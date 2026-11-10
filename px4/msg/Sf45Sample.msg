# Internal transport from the DroneCAN subscriber to the standalone SF45 module.
uint64 timestamp # Local reception time; not a synchronized acquisition timestamp
float32 yaw # Degrees in the unrotated sensor frame
float32 distance # Metres, first unfiltered return; >50 is no return
uint8 node_id
uint8 sensor_id
uint8 ORB_QUEUE_LENGTH = 32
