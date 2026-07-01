uint8 SIMULINK_INBOUND_ID = 200
uint8 SIMULINK_INBOUND_CTRL_ID = 250
uint8 SIMULINK_GUIDANCE_ID = 150
uint8 SIMULINK_OUTBOUND_ID = 100
uint8 SIMULINK_OUTBOUND_1_ID = 101
uint8 SIMULINK_OUTBOUND_2_ID = 102
uint8 SIMULINK_OUTBOUND_3_ID = 103
uint8 COMPANION_GUIDANCE_OUTBOUND_ID = 120
uint8 COMPANION_GUIDANCE_INBOUND_ID = 121

uint8 ARRAY_SIZE = 58
uint64 timestamp            # time since system start (microseconds)
uint16 id                   # unique ID of debug array, used to discriminate between arrays
char[10] name               # name of the debug array (max. 10 characters)
float32[58] data            # data

# TOPICS debug_array simulink_inbound simulink_outbound simulink_outbound_1 simulink_outbound_2 simulink_outbound_3 simulink_inbound_ctrl simulink_guidance companion_guidance_inbound companion_guidance_outbound
