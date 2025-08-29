from launch import LaunchDescription
from launch.actions import GroupAction, TimerAction, ExecuteProcess
from launch_ros.actions import Node, PushRosNamespace

def make_mavros_node(ns, fcu_url, gcs_url, sys_id, tgt_sys_id):
    return GroupAction([
        PushRosNamespace(ns),
        Node(
            package='mavros',
            executable='mavros_node',
            output='screen',
            parameters=[{
                'fcu_url': fcu_url,
                'gcs_url': gcs_url,
                'system_id': sys_id,
                'target_system_id': tgt_sys_id,
                # optional mavros params here…
            }],
        ),
    ])

def set_ext_state_cmd(ns, rate_hz=1.0):
    # Default MAVROS topic is /<ns>/mavros/set_message_interval
    service = f'/{ns}/mavros/set_message_interval'
    return ExecuteProcess(
        cmd=[
            'ros2','service','call', service, 'mavros_msgs/srv/MessageInterval',
            f'{{message_id: 245, message_rate: {rate_hz}}}'
        ],
        shell=False
    )

def generate_launch_description():
    # --- edit these for your setup ---
    uav1 = make_mavros_node('uav1', 'serial:///dev/ttyACM0:57600', 'udp://@0.0.0.0:14550', 255, 1)
    uav2 = make_mavros_node('uav2', 'serial:///dev/ttyACM1:57600', 'udp://@0.0.0.0:14551', 255, 2)
    uav3 = make_mavros_node('uav3', 'serial:///dev/ttyACM2:57600', 'udp://@0.0.0.0:14552', 255, 3)

    # After MAVROS starts, request EXTENDED_SYS_STATE (245) @ 1 Hz on each
    kick_uav1 = set_ext_state_cmd('uav1', 1.0)
    kick_uav2 = set_ext_state_cmd('uav2', 1.0)
    kick_uav3 = set_ext_state_cmd('uav3', 1.0)

    # Give MAVROS a few seconds to advertise services before calling them
    delayed_kicks = TimerAction(period=3.0, actions=[kick_uav1, kick_uav2, kick_uav3])

    return LaunchDescription([
        uav1, uav2, uav3,
        delayed_kicks
    ])
