from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pubsubint',
            #namespace='turtlesim1',
            executable='pubsubint_node',
            name='pubsubint_node',
            #output="screen",
            #emulate_tty=True
        )
    ])