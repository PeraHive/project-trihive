# selective_mavros.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction, GroupAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
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
            }],
        ),
    ])

def set_ext_state_cmd(ns, rate_hz=1.0):
    service = f'/{ns}/mavros/set_message_interval'
    return ExecuteProcess(
        cmd=[
            'ros2','service','call', service, 'mavros_msgs/srv/MessageInterval',
            f'{{message_id: 245, message_rate: {rate_hz}}}',
        ],
        shell=False
    )

# Edit this mapping for your ports/IDs
UAV_CFG = {
    1: dict(ns='uav1', fcu='udp://0.0.0.0:14551@', gcs='udp://@0.0.0.0:14550', sys_id=255, tgt_id=1),
    2: dict(ns='uav2', fcu='udp://0.0.0.0:14552@', gcs='udp://@0.0.0.0:14550', sys_id=255, tgt_id=2),
    3: dict(ns='uav3', fcu='udp://0.0.0.0:14553@', gcs='udp://@0.0.0.0:14550', sys_id=255, tgt_id=3),
}

def _expand_uavs(s: str):
    s = s.strip()
    if not s:
        return []
    parts = []
    for tok in s.split(','):
        tok = tok.strip()
        if '-' in tok:  # support ranges like "1-3"
            a, b = tok.split('-', 1)
            parts.extend(range(int(a), int(b) + 1))
        else:
            parts.append(int(tok))
    return parts

def _setup(context, *args, **kwargs):
    uavs_str = LaunchConfiguration('uavs').perform(context)  # e.g., "1,2" or "1-3"
    rate_str = LaunchConfiguration('ext_state_rate').perform(context)
    rate = float(rate_str)

    selected = _expand_uavs(uavs_str)
    actions = []

    nodes = []
    kicks = []
    for i in selected:
        cfg = UAV_CFG.get(i)
        if not cfg:
            continue
        nodes.append(
            make_mavros_node(cfg['ns'], cfg['fcu'], cfg['gcs'], cfg['sys_id'], cfg['tgt_id'])
        )
        kicks.append(set_ext_state_cmd(cfg['ns'], rate))

    if nodes:
        actions.extend(nodes)
        actions.append(TimerAction(period=15.0, actions=kicks))  # wait for services to appear

    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'uavs',
            default_value='1,2,3',
            description='Comma list or ranges of UAV indices to launch (e.g., "1", "2", "1,3", "1-2")'
        ),
        DeclareLaunchArgument(
            'ext_state_rate',
            default_value='1.0',
            description='EXTENDED_SYS_STATE (245) request rate in Hz'
        ),
        OpaqueFunction(function=_setup),
    ])

