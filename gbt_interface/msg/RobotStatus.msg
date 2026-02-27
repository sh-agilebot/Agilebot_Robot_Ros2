
# Definition of a message type for the robot's operational status

# Status Code Descriptions:
# ROBOT_IDLE = (0, "Robot Idle")
# ROBOT_RUNNING = (1, "Robot Running")
# ROBOT_TEACHING = (2, "Robot Teaching")
# ROBOT_IDLE_TO_RUNNING = (101, "Robot Intermediate State: Idle to Running")
# ROBOT_IDLE_TO_TEACHING = (102, "Robot Intermediate State: Idle to Teaching")
# ROBOT_RUNNING_TO_IDLE = (103, "Robot Intermediate State: Running to Idle")
# ROBOT_TEACHING_TO_IDLE = (104, "Robot Intermediate State: Teaching to Idle")
# ROBOT_UNKNOWN = (-1, "Unknown Robot Status")

int32 status_code       # Status code
string status_description  # Description of the status

