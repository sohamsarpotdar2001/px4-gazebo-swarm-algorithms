# PX4 Gazebo Swarm Algorithms
A ROS 2 package for experimenting with swarm-control algorithms in a multi-vehicle PX4 SITL simulation in Gazebo.

The repository contains ROS 2 controller code, launch files, and headers for building swarm controllers. The controller framework is designed to support different algorithms and vehicle-agent types.

## Requirements
Install and configure the versions of these tools that are compatible with your PX4 and Gazebo setup:
* [ROS 2](https://docs.px4.io/main/en/ros2/user_guide#install-ros-2)
* Colcon and other ROS 2 build tools
* [PX4 Autopilot](https://docs.px4.io/main/en/ros2/user_guide#install-px4)
* [Gazebo](https://gazebosim.org/docs/harmonic/ros_installation/)
* [QGroundControl](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html)
* [uXRCE-DDS](https://docs.px4.io/main/en/middleware/uxrce_dds)
* [px4_msgs](https://github.com/PX4/px4_msgs)
* Gnuplot - `sudo apt install gnuplot`

## Features
* Multi-vehicle PX4 Software-in-the-Loop (SITL) simulation
* Swarm logging and performance plots
* Comparison of mission completion time across algorithms and varying swarm sizes
* Extensible controller base class
* Available algorithms:
  * Boids
  * Consensus
  * Artificial Potential Fields
  * Formation Control
  * Particle Swarm Optimization
  * Artificial Bee Colony
  * Ant Colony Optimization

## Build and Test
From your ROS 2 workspace
```
cd ~/ros2_ws/src
git clone https://github.com/sohamsarpotdar2001/px4-gazebo-swarm-algorithms.git
```

Build the package
```
cd ~/ros2_ws
colcon build --packages-select drone_swarm
```

Start QGroundControl (Terminal 1)
```
./QGroundControl-x86_64.AppImage
```

Run the simulation (Terminal 2)
```
ros2 launch drone_swarm swarm_launch.py drone_count:=5
```

Test a controller (Terminal 3)
```
ros2 launch drone_swarm swarm_logger.launch.py algo:=consensus drone_count:=5
```

<img width="800" height="600" alt="swarm_simulation_gazebo" src="https://github.com/user-attachments/assets/f8fcac61-0eb6-4c4d-a095-2f883675310f" />

After the waypoint mission ends, Press `Ctrl + C` to terminate the nodes in Terminal 3 and save the plot in the present directory.
The `swarm_logger` node prints out the calculated metrics in the terminal and also mentions the graph file
```
[swarm_logger-3] [INFO] [1795612768.089351765] [swarm_logger]: ================ SWARM METRICS SUMMARY ================
[swarm_logger-3] [INFO] [1795612768.089380885] [swarm_logger]: Samples Recorded           : 383 (38.2 s)
[swarm_logger-3] [INFO] [1795612768.089393356] [swarm_logger]: Mean Tracking Error        : 2.074 m
[swarm_logger-3] [INFO] [1795612768.089399873] [swarm_logger]: Total Separation Violations: 13
[swarm_logger-3] [INFO] [1795612768.089405484] [swarm_logger]: Total Collisions (<0.65m)  : 0
[swarm_logger-3] [INFO] [1795612768.089412137] [swarm_logger]: Goals Reached              : 5 (mean settling: 5.51 s)
[swarm_logger-3] [INFO] [1795612768.089418403] [swarm_logger]: Gnuplot graph saved to     : swarm_consensus_report.png
```

<img width="800" height="500" alt="swarm_consensus_report" src="https://github.com/user-attachments/assets/7326e74d-f406-4ae5-86f1-ba4ef640cfb6" /> 

## Scaling up
The repository also includes code for comparing different algorithms across varying numbers of drones. It plots mission completion time for the algorithms in the same graph, making it easier to compare performance as swarm size changes.

First run QGC (Terminal 1)
```
./QGroundControl-x86_64.AppImage
```

Then swarm launch file (Terminal 2)
```
ros2 launch drone_swarm swarm_launch.py drone_count:=5
```

Start the mission publisher and logger nodes (Terminal 3)
```
ros2 launch drone_swarm scaling_swarm.launch.py algo:=boids
```

Run the algorithm node (Terminal 4)
```
ros2 run drone_swarm boids --drone_count=5
```

When all the waypoints are covered, terminate gazebo and algorithm node in Terminals 2 and 4. Then relaunch gazebo with a different `drone_count` value and start the same algoritihm node or any other of your choice. After every mission completion, the logger node prints out the performance metrics across all tested algorithms and drone sizes.

<img width="800" height="500" alt="scalability" src="https://github.com/user-attachments/assets/819bf80e-3d27-462c-9bbc-77a736b78986" />

## Swarm logging and evaluation

The swarm logger generates four plots for evaluating an algorithm:

1. **Centroid tracking error**
```math
\epsilon = P_{centroid} - P_{goal}
```

2. **Goal Reaching time**
```math
T_{goal} = T_{goal\ reached} - T_{goal\ publshed}
```

3. **Minimum inter-drone separation**
```math
\theta = min(P_n - P_{n-1})
```
7. **Cumulative separation violations**
```math
\lambda = (P_n - P_{n-1}) < d_{safe}
```

## Development and extensions

The project is designed to be extended with custom swarm algorithms, Gazebo worlds, and performance metrics.

### Implement a custom algorithm
To create a new controller, derive a class from the `SwarmBaseController` base class. This ensures compatibility with the launch system and the swarm logger.

1. **Inherit**: Create a class that inherits from `SwarmBaseController<BaseDroneAgent>`
2. **Expand**: Add more variables to your DroneAgent by inheriting from BaseDroneAgent
    ```
    struct MyDroneAgent : public BaseDroneAgent {
        double obstacle_dist;
        double battery_health;
    };
    
    class Px4CustomController : public SwarmBaseController<MyDroneAgnet> {
    public:
        explicit Px4CustomController() : SwarmBaseController<MyDroneAgent>(node_name) { }
    };
    ```

3. **Override**: Update the `compute_velocities` function for your custom algorithm
    ```
    class MySwarmController : public SwarmBaseController<BaseDroneAgent> {
    public:
        void compute_velocities() override {
            // custom algorithm implementation
        }
    
        // Add more functions or control logic required by your implementation.
    };
    ```

### Adding Custom Gazebo Worlds

You can test your algorithms in custom environments to evaluate robustness against different terrain or obstacles.

* **Base on Defaults**: Start with a default PX4 world SDF to ensure all necessary simulation plugins (Physics, ground_plane, sun) are initialized correctly.
* **Modify Terrain**: Add your custom models or terrains to the `<model>` section. Also put your custom model at `PX4-Autopilot/Tools/simulation/gz/models`
  ```
  <model name="grasspatch">
    <static>true</static>
    <include>
      <uri>https://fuel.ignitionrobotics.org/1.0/hexarotor/models/grasspatch</uri>
      <name>grasspatch</name>
      <pose>2.71574 -22.80395 0 0 -0 0.006057</pose>
    </include>
  </model>
  <model name="my_model">
    <static>true</static>
    <include>
      <uri>model://my_model</uri>
    </include>
  </model>
  ```
* **Save and Apply**: Save the custom world file to `PX4-Autopilot/Tools/simulation/gz/worlds`. Edit the `swarm_launch.py` file from repo and add your world name for the `PX4_GZ_WORLD` variable

  <img width="800" height="600" alt="custom_world_sim" src="https://github.com/user-attachments/assets/5f485773-3692-4576-ba96-0934d97a8e39" />


### Custom Performance Metrics
The swarm logger can be extended to track new KPIs beyond the default tracking error and separation violations.
* **Log Data**: Add the required telemetry or state data to the logger's collection loop.
* **Update Plotter**: Modify the plotting script to process this new data stream.
* **Compare**: Use gnuplot to plot these new metrics across different algorithms or varying numbers of drones to analyze scalability.
