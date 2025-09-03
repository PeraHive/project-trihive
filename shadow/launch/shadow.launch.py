from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Leader (uav1)
        Node(
            package='shadow',
            executable='shadow_leader',
            namespace='leader',
            output='screen',
            parameters=[{
                'rel_dy': 0.0,
                'target_alt':5.0,
            }]
        ),

        # Follower uav2 (right side)
        Node(
            package='shadow',
            executable='shadow_follower',
            namespace='follower_right',
            output='screen',
            parameters=[{
                'follower_ns': '/uav2',
                'leader_ns': '/uav1',
                'side': 'right',
                'slot_offset_m': 5.0,
                'target_alt': 5.0,
            }],
        ),

        # Follower uav3 (left side)
        Node(
            package='shadow',
            executable='shadow_follower',
            namespace='follower_left',
            output='screen',
            parameters=[{
                'follower_ns': '/uav3',
                'leader_ns': '/uav1',
                'side': 'left',
                'slot_offset_m': 5.0,
                'target_alt': 5.0,
            }],
        ),
    ])
