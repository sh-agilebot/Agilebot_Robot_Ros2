# Definition of a message type for the servo controller's operational status
# Controller Operational Status Enum List:
# CTRL_INIT = (0, "Controller Initialization")
# CTRL_ENGAGED = (1, "Controller Engaged")
# CTRL_ESTOP = (2, "Controller Emergency Stop")
# CTRL_TERMINATED = (3, "Controller Terminated")
# CTRL_ANY_TO_ESTOP = (101, "Controller Intermediate State: Any to Emergency Stop")
# CTRL_ESTOP_TO_ENGAGED = (102, "Controller Intermediate State: Emergency Stop to Engaged")
# CTRL_ESTOP_TO_TERMINATED = (103, "Controller Intermediate State: Emergency Stop to Terminated")
# CTRL_UNKNOWN = (-1, "Unknown Controller Status")

int32 status_code       # Status code
string status_description  # Description of the status

