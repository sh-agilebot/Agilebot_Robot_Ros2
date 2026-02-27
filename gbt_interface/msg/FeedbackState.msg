std_msgs/Header header

# Connection Status
bool is_connected                            # Whether connected to the robot

# Robot Status
string robot_type                            # Robot Type
RobotStatus robot_status                     # Robot Status
ControllerStatus controller_status           # Controller Status
ServoStatus servo_status                     # Servo Status
ArmSoftModeStatus arm_soft_mode_status       # Arm Soft Mode Status

# Robot joint 
sensor_msgs/JointState joint_states          # Robot joint states
geometry_msgs/PoseStamped flange_pose        # Robot flange pose
geometry_msgs/PoseStamped tool_pose          # Robot tool pose

# alarm code 
AlarmCode[]   alarm_code_list                # Robot alarm code

float32 overall_speed                        # Overall speed (%)


# IO
IOType[] io                                   # TODO: IO status

bool is_error                                 # TODO: implement error detection
uint8 error_code                              # TODO: assign proper error codes on failures


# coordinate system
int32 actived_uf_id                           # Activated user coordinate system ID
int32 actived_tf_id                           # Activated tool coordinate system ID

