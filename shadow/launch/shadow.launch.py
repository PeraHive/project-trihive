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
                'target_alt': 8.0,
            }]
        ),

        # Follower uav2 (right side, +Y offset in ENU)
        Node(
            package='shadow',
            executable='shadow_follower',
            namespace='follower_right',
            output='screen',
            parameters=[{
                'follower_ns': '/uav2',
                'leader_ns': '/uav1',
                'target_alt': 5.0,
                'offset_e': 4.0,   # East (+x)
                'offset_n': 0.0,  # North (+y); adjust sign for your formation
                'offset_u': 0.0,   # Up (+z)
                'sp_rate_hz': 12.0,
            }],
        ),

        # Follower uav3 (left side, -Y offset in ENU)
        Node(
            package='shadow',
            executable='shadow_follower',
            namespace='follower_left',
            output='screen',
            parameters=[{
                'follower_ns': '/uav3',
                'leader_ns': '/uav1',
                'target_alt': 5.0,
                'offset_e': -4.0,
                'offset_n': 0.0,
                'offset_u': 0.0,
                'sp_rate_hz': 12.0,
            }],
        ),
    ])
