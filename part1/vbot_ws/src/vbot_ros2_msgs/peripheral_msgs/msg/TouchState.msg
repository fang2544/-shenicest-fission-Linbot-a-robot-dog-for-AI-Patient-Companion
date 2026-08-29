uint8 source # 0: s100, 1: x5
uint64 timestamp # ms 
# Endpoint-specific code:
# /touch_node/touch_state: 0=released, 1=pressed
# /touch_node/touch_event: 10=single_tap, 11=double_tap, 12=triple_tap
uint8 state
uint8 idx
