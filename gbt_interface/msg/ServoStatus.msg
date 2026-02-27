# Definition of a message type for the servo controller's operational status

# Status Code Descriptions:
# SERVO_IDLE = (1, "Servo Controller Idle")
# SERVO_RUNNING = (2, "Servo Controller Running")
# SERVO_DISABLE = (3, "Servo Controller Disabled")
# SERVO_WAIT_READY = (4, "Servo Controller Waiting for Ready")
# SERVO_WAIT_DOWN = (5, "Servo Controller Waiting for Shutdown")
# SERVO_INIT = (10, "Servo Controller Initializing")
# SERVO_UNKNOWN = (-1, "Unknown Servo Controller Status")

int32 status_code       # Status code
string status_description  # Description of the status

