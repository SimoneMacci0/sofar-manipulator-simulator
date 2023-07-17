from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sofar_manipulator_simulator',
            executable='manipulator_sim_node',
            name='manipulator_sim_node'
        ),
        Node(
            package='sofar_manipulator_simulator',
            executable='controller_node',
            name='controller_node'
        ),
        Node(
            package='sofar_manipulator_simulator',
            executable='logic_node',
            name='logic_node',
        )
    ])
