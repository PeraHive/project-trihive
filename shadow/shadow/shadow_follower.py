#!/usr/bin/env python3
"""
Shadow Follower (track XYZ + yaw) with optional GPS frame bridge

- Tracks the LEADER's pose (x,y,z,yaw) while holding a LEFT/RIGHT slot offset in XY.
- Altitude is always taken from the leader (no modes).
- Optional Fix #2: translate leader's local pose into the follower's local frame using
  an ENU translation between home GPS positions. Toggle with 'use_frame_bridge'.

Params:
  follower_ns:   '/uav2'
  leader_ns:     '/uav1'
  side:          'left' | 'right'   (slot direction)
  slot_offset_m: 5.0                (lateral spacing)
  target_alt:    10.0               (takeoff altitude for follower)
  guided_mode:   'GUIDED' (APM) | 'OFFBOARD' (PX4)
  poshold_mode:  'POSHOLD' (APM) | 'POSCTL' (PX4)
  sp_rate_hz:    12.0
  use_frame_bridge: False in sim, True in field (requires NavSatFix from both)
"""

import math
from typing import Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode


class ShadowFollower(Node):
    def __init__(self) -> None:
        super().__init__('shadow_follower')

        # QoS similar to MAVROS sensor topics
        qos_sensor = QoSProfile(depth=10)
        qos_sensor.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_sensor.durability  = QoSDurabilityPolicy.VOLATILE

        # ---------------- Parameters ----------------
        self.follower_ns = self.declare_parameter('follower_ns', '/uav2').get_parameter_value().string_value
        self.leader_ns   = self.declare_parameter('leader_ns',   '/uav1').get_parameter_value().string_value

        side = self.declare_parameter('side', 'left').get_parameter_value().string_value.lower()
        self.side = side if side in ('left', 'right') else 'left'

        self.slot_offset_m = float(self.declare_parameter('slot_offset_m', 5.0).get_parameter_value().double_value)
        self.target_alt    = float(self.declare_parameter('target_alt', 5.0).get_parameter_value().double_value)

        self.guided_mode   = self.declare_parameter('guided_mode', 'GUIDED').get_parameter_value().string_value
        self.poshold_mode  = self.declare_parameter('poshold_mode', 'POSHOLD').get_parameter_value().string_value
        self.sp_rate_hz    = float(self.declare_parameter('sp_rate_hz', 12.0).get_parameter_value().double_value)

        # Field/sim toggle for Fix #2
        self.use_frame_bridge = bool(self.declare_parameter('use_frame_bridge', False).get_parameter_value().bool_value)

        # Tolerances / checks
        self.alt_air_thresh = float(self.declare_parameter('alt_air_thresh', 0.5).get_parameter_value().double_value)

        # ---------------- Internal State ----------------
        self.my_state = State()
        self.my_rel_alt = 0.0
        self.my_pose = PoseStamped()

        self.leader_state = State()
        self.leader_pose = PoseStamped()

        # GPS homes for frame bridge
        self.leader_home: Optional[tuple] = None  # (lat, lon, alt)
        self.follower_home: Optional[tuple] = None

        # ---------------- Subscriptions ----------------
        # Follower
        self.create_subscription(State, f'{self.follower_ns}/state', self._state_cb, qos_sensor)
        self.create_subscription(Float64, f'{self.follower_ns}/global_position/rel_alt', self._alt_cb, qos_sensor)
        self.create_subscription(PoseStamped, f'{self.follower_ns}/local_position/pose', self._pose_cb, qos_sensor)
        if self.use_frame_bridge:
            self.create_subscription(NavSatFix, f'{self.follower_ns}/global_position/global', self._follower_gps_cb, qos_sensor)

        # Leader
        self.create_subscription(State, f'{self.leader_ns}/state', self._leader_state_cb, qos_sensor)
        self.create_subscription(PoseStamped, f'{self.leader_ns}/local_position/pose', self._leader_pose_cb, qos_sensor)
        if self.use_frame_bridge:
            self.create_subscription(NavSatFix, f'{self.leader_ns}/global_position/global', self._leader_gps_cb, qos_sensor)

        # ---------------- Publisher ----------------
        self.sp_pub = self.create_publisher(PositionTarget, f'{self.follower_ns}/setpoint_raw/local', 10)

        # ---------------- Services ----------------
        self.arm_cli     = self.create_client(CommandBool, f'{self.follower_ns}/cmd/arming')
        self.tko_cli     = self.create_client(CommandTOL,  f'{self.follower_ns}/cmd/takeoff')
        self.setmode_cli = self.create_client(SetMode,     f'{self.follower_ns}/set_mode')

        self.get_logger().info('Waiting for follower MAVROS services...')
        for cli in (self.arm_cli, self.tko_cli, self.setmode_cli):
            cli.wait_for_service()
        self.get_logger().info('Follower MAVROS services are available.')

    # ---------------- Callbacks ----------------
    def _state_cb(self, msg: State): self.my_state = msg
    def _alt_cb(self, msg: Float64): self.my_rel_alt = float(msg.data)
    def _pose_cb(self, msg: PoseStamped): self.my_pose = msg

    def _leader_state_cb(self, msg: State): self.leader_state = msg
    def _leader_pose_cb(self, msg: PoseStamped): self.leader_pose = msg

    def _leader_gps_cb(self, msg: NavSatFix):
        if self.leader_home is None and msg.status.status != -1:
            self.leader_home = (msg.latitude, msg.longitude, msg.altitude)
            self.get_logger().info(f'Leader home set: {self.leader_home}')

    def _follower_gps_cb(self, msg: NavSatFix):
        if self.follower_home is None and msg.status.status != -1:
            self.follower_home = (msg.latitude, msg.longitude, msg.altitude)
            self.get_logger().info(f'Follower home set: {self.follower_home}')

    # ---------------- Helpers ----------------
    def _set_mode(self, mode: str, timeout: float = 10.0) -> bool:
        req = SetMode.Request(); req.base_mode = 0; req.custom_mode = mode
        fut = self.setmode_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return bool(fut.done() and fut.result() and getattr(fut.result(), 'mode_sent', False))

    def _arm_once(self, timeout: float = 10.0) -> bool:
        req = CommandBool.Request(); req.value = True
        fut = self.arm_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return bool(fut.done() and fut.result() and getattr(fut.result(), 'success', False))

    def _takeoff(self, alt: float = 10.0, timeout: float = 10.0) -> bool:
        req = CommandTOL.Request()
        req.altitude = float(alt); req.latitude = 0.0; req.longitude = 0.0
        req.min_pitch = 0.0; req.yaw = float('nan')
        fut = self.tko_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return bool(fut.done() and fut.result() and getattr(fut.result(), 'success', False))

    @staticmethod
    def _pose_xyz_yaw(p: PoseStamped) -> Tuple[float, float, float, float]:
        q = p.pose.orientation
        siny_cosp = 2.0 * (q.w*q.z + q.x*q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return p.pose.position.x, p.pose.position.y, p.pose.position.z, yaw

    def _slot_offset_vector(self, yaw: float, d: float) -> Tuple[float, float]:
        # ENU: left = (-sinψ, +cosψ)*d ; right = ( +sinψ, -cosψ)*d
        return ((-math.sin(yaw)*d, math.cos(yaw)*d) if self.side == 'left'
                else ( math.sin(yaw)*d,-math.cos(yaw)*d))

    @staticmethod
    def _enu_offset(lat0, lon0, alt0, lat1, lon1, alt1):
        """Approx ENU translation from (lat0,lon0,alt0)->(lat1,lon1,alt1) around (lat0,lon0)."""
        R = 6378137.0
        lat0r = math.radians(lat0)
        dlat = math.radians(lat1 - lat0)
        dlon = math.radians(lon1 - lon0)
        dE = dlon * math.cos(lat0r) * R
        dN = dlat * R
        dU = (alt1 - alt0)
        return dE, dN, dU

    def _bridge_leader_pose_if_needed(self, lx, ly, lz):
        if not self.use_frame_bridge: return lx, ly, lz
        if self.leader_home is None or self.follower_home is None: return lx, ly, lz
        dE, dN, dU = self._enu_offset(*self.follower_home, *self.leader_home)
        return lx + dE, ly + dN, lz + dU

    def _publish_position_target(self, x: float, y: float, z: float, yaw: float) -> None:
        sp = PositionTarget()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED  # MAVROS handles ENU<->NED
        sp.type_mask = (
            PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        sp.position.x = x; sp.position.y = y; sp.position.z = z
        sp.yaw = yaw
        self.sp_pub.publish(sp)

    # ---------------- Mission ----------------
    def run(self) -> None:
        # Wait FCU
        while rclpy.ok() and not self.my_state.connected:
            self.get_logger().info('Follower waiting for FCU connection...')
            rclpy.spin_once(self, timeout_sec=0.2)

        # Enter GUIDED/OFFBOARD
        self._set_mode(self.guided_mode)

        # Arm
        while rclpy.ok() and not self.my_state.armed:
            if not self._arm_once():
                self.get_logger().warn('Follower arming failed, retrying...')
            rclpy.spin_once(self, timeout_sec=0.1)

        # Takeoff to target_alt (initial climb only)
        if not self._takeoff(self.target_alt):
            self.get_logger().error('Follower takeoff failed; aborting.')
            return
        while rclpy.ok() and self.my_rel_alt < (self.target_alt - self.alt_air_thresh):
            rclpy.spin_once(self, timeout_sec=0.2)
        self.get_logger().info(f'Follower airborne: rel_alt={self.my_rel_alt:.1f} m')

        # Formation tracking loop: track leader XYZ + yaw, with lateral slot
        dt = 1.0 / max(1e-3, self.sp_rate_hz)
        self.get_logger().info(f'Holding {self.side.upper()} slot at {self.slot_offset_m:.1f} m from leader...')
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            # Leader pose in its local frame
            lx, ly, lz, lyaw = self._pose_xyz_yaw(self.leader_pose)
            # Optionally translate leader pose into follower's local frame
            # lx, ly, lz = self._bridge_leader_pose_if_needed(lx, ly, lz)
            # Slot offset (XY only); altitude follows leader exactly
            ox, oy = self._slot_offset_vector(lyaw, self.slot_offset_m)
            gx, gy, gz = lx + ox, ly + oy, lz
            # Same heading as leader
            gyaw = lyaw
            # Stream raw setpoint
            self._publish_position_target(gx, gy, gz, gyaw)
            # pace
            # self.get_clock().sleep_for(rclpy.duration.Duration(seconds=dt))
            # time.sleep(dt)


def main() -> None:
    rclpy.init()
    node = ShadowFollower()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
