# Robot Motion Planning with MoveIt2

This section describes how to use MoveIt 2 for motion planning of Agilebot robots. MoveIt 2 is a ROS 2–based robot motion planning framework that integrates the following core features:

* **Collision Detection**: Avoids collisions with the environment and the robot itself in real time during planning
* **Kinematics Solvers**: Provides efficient forward and inverse kinematics solvers
* **Path Planning**: Supports various planar and spatial planning algorithms (e.g., OMPL, CHOMP, STOMP)
* **Trajectory Optimization**: Smooths generated paths and enforces velocity/acceleration limits
* **Trajectory Execution**: Seamlessly interfaces with hardware controllers for real-time motion control
* **Real-Time Replanning**: Automatically updates plans when the environment or goal changes
* **Visualization & Debugging**: Displays the planning pipeline in RViz plugins and supports interactive goal setting and parameter tuning

With MoveIt 2, you can rapidly generate and execute high-quality arm trajectories in both simulation and on real robots, streamlining your development and debugging workflows.


![](/assets/moveit.png)

---

To perform **path planning and motion control** for Shanghai Agilebot Robotics products using MoveIt2, run:

```bash
ros2 launch <robot type>_moveit_config demo.launch.py
```

For example, to launch MoveIt2 for the C5A model:

```bash
ros2 launch c5a_moveit_config demo.launch.py
```

Other robot models can be launched similarly:

```bash
# C7A robot
ros2 launch c7a_moveit_config demo.launch.py

# C12A robot
ros2 launch c12a_moveit_config demo.launch.py

# C16A robot
ros2 launch c16a_moveit_config demo.launch.py
```

This will start the MoveIt2 demonstration interface, enabling path planning and other operations.

> Ref: [MoveIt2 official documentation](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html#step-1-start).
