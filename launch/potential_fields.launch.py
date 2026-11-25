from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='drone_swarm',
            executable='waypoint_mission',
            name='waypoint_mission',
            output='screen',
        ),

        Node(
            package='drone_swarm',
            executable='potential_fields',
            name='potential_fields',
            output='screen',
        )
   ])
