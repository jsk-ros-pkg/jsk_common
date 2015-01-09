# A message from OSC to FC.
# There is are several limitations
# 1. no nested fields is allowed.
# 2. no variable length array is allowed.
# 3. string is not supported.
# 4. duration and time are not supported.

uint8[32] joint_angles
bool start_impedance
bool stop
