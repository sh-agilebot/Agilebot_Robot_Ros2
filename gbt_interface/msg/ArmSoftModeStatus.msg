# Definition of a message type for the servo controller's operational status
# Status Code Descriptions:
# UNKNOWN = 0
# AUTO = 1      # Automatic mode
# MANUAL_LIMIT = 2  # Manual speed-limited mode
# MANUAL = 3    # Manual mode

int32 status_code       # Status code
string status_description  # Description of the status

