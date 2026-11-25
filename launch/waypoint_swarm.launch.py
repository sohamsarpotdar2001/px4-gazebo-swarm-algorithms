from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='drone_swarm',
            executable='waypoint_follower',
            namespace='uav_1',
            name='waypoint_follower',
            output='screen',
            parameters=[{
                'waypoints': [0.0, 0.0, -3.0, 5.0, 0.0, -3.0, 5.0, 5.0, -3.0],
                'acceptance_radius': 0.5,
            }],
        ),
   ])
