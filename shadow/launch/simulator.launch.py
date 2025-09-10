# selective_mavros.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction, GroupAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def make_mavros_node(ns, fcu_url, sys_id, tgt_sys_id, gcs_url=None):
    params = {
        'fcu_url': fcu_url,
        'system_id': sys_id,
        'target_system_id': tgt_sys_id,
    }
    if gcs_url:
        params['gcs_url'] = gcs_url
    return GroupAction([
        PushRosNamespace(ns),
        Node(
            package='mavros',
            executable='mavros_node',
            output='screen',
            parameters=[params],
        ),
    ])

def set_ext_state_cmd(ns, rate_hz=1.0):
    # MAVLink 2: EXTENDED_SYS_STATE = 245
    service = f'/{ns}/mavros/set_message_interval'
    return ExecuteProcess(
        cmd=[
            'ros2','service','call', service, 'mavros_msgs/srv/MessageInterval',
            f'{{message_id: 245, message_rate: {rate_hz}}}',
        ],
        shell=False
    )

def _expand_uavs(s: str):
    s = s.strip()
    if not s:
        return []
    parts = []
    for tok in s.split(','):
        tok = tok.strip()
        if '-' in tok:
            a, b = tok.split('-', 1)
            parts.extend(range(int(a), int(b) + 1))
        else:
            parts.append(int(tok))
    return parts

def _setup(context, *args, **kwargs):
    uavs_str = LaunchConfiguration('uavs').perform(context)          # e.g. "1,2" or "1-3"
    base_port = int(LaunchConfiguration('base_port').perform(context))  # default 14550
    bind_ip   = LaunchConfiguration('bind_ip').perform(context)      # default "0.0.0.0"
    rate      = float(LaunchConfiguration('ext_state_rate').perform(context))
    gcs_url   = LaunchConfiguration('gcs_url').perform(context)      # optional, can be empty

    selected = _expand_uavs(uavs_str)
    actions = []
    nodes, kicks = [], []

    for i in selected:
        ns = f'uav{i}'
        port = base_port + (i) # each UAV gets its own port
        fcu = f'udp://{bind_ip}:{port}@'   # listen (server) on this machine
        node = make_mavros_node(
            ns=ns,
            fcu_url=fcu,
            sys_id=255,           # ground system id
            tgt_sys_id=i,         # we expect this UAV SYSID from SITL
            gcs_url=gcs_url if gcs_url else None
        )
        nodes.append(node)
        kicks.append(set_ext_state_cmd(ns, rate))

    if nodes:
        actions.extend(nodes)
        # give MAVROS time to bring up services before calling MessageInterval
        actions.append(TimerAction(period=15.0, actions=kicks))

    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'uavs',
            default_value='1,2',
            description='Comma list or ranges of UAV indices (e.g., "1", "2", "1,3", "1-3")'
        ),
        DeclareLaunchArgument(
            'base_port',
            default_value='14550',
            description='First UDP port to bind; subsequent UAVs use base_port+(index-1)'
        ),
        DeclareLaunchArgument(
            'bind_ip',
            default_value='0.0.0.0',
            description='IP to bind MAVROS UDP server (use 0.0.0.0 to accept from any)'
        ),
        DeclareLaunchArgument(
            'gcs_url',
            default_value='',
            description='Optional GCS endpoint (e.g., "udp://@192.168.184.158:14650"). Leave empty to disable.'
        ),
        DeclareLaunchArgument(
            'ext_state_rate',
            default_value='1.0',
            description='EXTENDED_SYS_STATE (245) request rate in Hz'
        ),
        OpaqueFunction(function=_setup),
    ])
