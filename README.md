# PX4 Gazebo Swarm Algorithms
A ROS 2 package for experimenting with swarm-control algorithms in a multi-vehicle PX4 SITL simulation in Gazebo.

The repository contains ROS 2 controller code, launch files, and headers for building swarm controllers. The controller framework is designed to support different algorithms and vehicle-agent types.

## Requirements
Install and configure the versions of these tools that are compatible with your PX4 and Gazebo setup:
* ROS 2 [Install](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
* PX4 Autopilot [Setup](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu)
* Gazebo [Install](https://gazebosim.org/docs/harmonic/ros_installation/)
* `colcon` and the ROS 2 build tools
* Gnuplot

## Build and Simulate
From your ROS 2 workspace
```
cd ~/ros2_ws/src
git clone https://github.com/sohamsarpotdar2001/px4-gazebo-swarm-algorithms.git
```

Build the package
```
cd ~/ros2_ws
colcon build --packages-select px4-gazebo-swarm-algorithms
```

Run the simulation
```
ros2 launch px4-gazebo-swarm-algorithms swarm_launch.py drone_count:=5
```

Test a controller
```
ros2 launch px4-gazebo-swarm-algorithms swarm_logger.launch.py algo:=consensus
```

Available algorithms:
* Boids
* Consensus
* Artificial Potential Fields
* Formation Control
* Particle Swarm Optimization
* Artificial Bee Colony
* Ant Colony Optimization

After the waypoint mission ends, Press `Ctrl + C` to terminate the nodes and save the plot in the present directory
