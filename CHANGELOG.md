# Changelog
All significant changes will be documented in this file.
## [0.2.0] - February 27, 2026
### Added
- Completed Chinese documentation structure: Installation, Quick Start, Examples, FAQ, Roadmap, License
- Added environment and build guide: Ubuntu 22.04 + ROS2 Humble + Python 3.10 and dependency setup
- Clarified supported robot models: C5A, C7A, C12A, C16A (URDF, Gazebo, MoveIt2)
- Added real robot state synchronization guide (publish `/joint_states` and visualize in RViz)
- Added robot status topic documentation: `/gbt_driver/feedback_states`
- Added service and action interface docs: `move_to_pose`, IO read/write, program control, servo control, emergency stop, script sending
- Added documentation for MoveIt2 real robot control (Experimental)
- Added offline trajectory CSV control guide (format and examples)
- Added vision and application examples: AgileGaze communication and palletizing demo
- Added robot connection mode documentation: physical robot, virtual controller, AirBot cloud robot

### Changed
- Unified documentation structure under `ros2_docs/zh` with topic-based sections
- Refined quick-start command and parameter descriptions (`robot_type`, `controller_name`, `enable_rviz`)

### Known Limitations
- `io` field in `/gbt_driver/feedback_states` is currently not published by default; use `/gbt_driver/service_server/io` for per-port reads
- MoveIt2 real robot control is still in Experimental stage

## [0.1.0] - August 4, 2025
### Added
- URDF model (including textures) import
- RViz visualization of URDF model
- Gazebo import of URDF model
- MoveIt trajectory planning
- MoveIt + Gazebo co-simulation
- Publish robot information
  - Publish robot model
  - Publish rotation angles of each joint
  - Publish end flange pose
  - Publish tool coordinate system pose
  - Publish error codes
  - Publish controller status
  - Publish servo controller status
  - Publish robot connection status
  - Publish soft mode status
  - Publish global velocity
  - Publish currently activated UF, TF
- Synchronize physical robot pose to ROS
- Read and write IO through services
- Turn on and off LED through services
- Emergency stop through services
- Control program start, pause, resume, stop through services
- Control servo on and off through services
- By sending scripts
- Move to a point through action
- Offline trajectory
  - Execute offline trajectory file through action
  - MoveIt2 plan and generate offline trajectory file for execution
- Vision integration
  - ROS2 node calls AgileGaze flowchart
- Palletizing
  - Simulated data vision palletizing demo
