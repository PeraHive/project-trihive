from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Leader (uav1)
        Node(
            package='shadow',
            executable='shadow_leader',
            namespace='uav1',
            output='screen',
        ),

        # Follower uav2 (right side)
        Node(
            package='shadow',
            executable='shadow_follower',
            namespace='uav2',
            output='screen',
            parameters=[{
                'follower_ns': '/uav2',
                'leader_ns': '/uav1',
                'side': 'right',
                'slot_offset_m': 5.0,
                'target_alt': 10.0,
            }],
        ),

        # Follower uav3 (left side)
        # Node(
        #     package='shadow',
        #     executable='shadow_follower.py',
        #     namespace='uav3',
        #     output='screen',
        #     parameters=[{
        #         'follower_ns': '/uav3',
        #         'leader_ns': '/uav1',
        #         'side': 'left',
        #         'slot_offset_m': 5.0,
        #         'target_alt': 10.0,
        #     }],
        # ),
    ])
