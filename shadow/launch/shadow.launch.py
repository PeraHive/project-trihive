from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Assumes MAVROS already launched for uav1 elsewhere; if not, add it here.
        Node(
            package='shadow',
            executable='shadow_leader',
            namespace='uav1',
            output='screen',
        ),
    ])