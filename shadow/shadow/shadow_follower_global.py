
#!/usr/bin/env python3
import time
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State, GlobalPositionTarget, HomePosition
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode


class GPSShadowFollower(Node):
    """
    GPS-based shadow follower.

    Idea:
      - Read leader & follower *home* GPS.
      - Compute offset dLat/dLon = follower_home - leader_home.
      - Track the leader's current GPS by publishing leader_gps + offset as a
        SET_POSITION_TARGET_GLOBAL_INT (REL_ALT frame).
      - Copy leader heading (optional) and relative altitude.
      - If leader switches to RTL, follower also switches to RTL.

    Works with ArduPilot via MAVROS on ROS 2.
    """
    def __init__(self) -> None:
        super().__init__('gps_shadow_follower')

        qos_sensor = QoSProfile(depth=10)
        qos_sensor.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_sensor.durability  = QoSDurabilityPolicy.VOLATILE

        # ---------------- Parameters ----------------
        self.follower_ns = self.declare_parameter('follower_ns', '/uav3').get_parameter_value().string_value
        self.leader_ns   = self.declare_parameter('leader_ns',   '/uav1').get_parameter_value().string_value
        self.target_alt  = float(self.declare_parameter('target_alt', 3.0).get_parameter_value().double_value)
        self.alt_thresh  = float(self.declare_parameter('alt_air_thresh', 0.5).get_parameter_value().double_value)
        self.sp_rate_hz  = float(self.declare_parameter('sp_rate_hz',  10.0).get_parameter_value().double_value)
        self.guided_mode = self.declare_parameter('guided_mode', 'GUIDED').get_parameter_value().string_value
        self.copy_yaw    = self.declare_parameter('copy_yaw', True).get_parameter_value().bool_value

        # ---------------- Internal state ----------------
        self.my_state = State()
        self.my_rel_alt = 0.0

        self.leader_state = State()
        self.leader_gps = NavSatFix()
        self.leader_rel_alt = 0.0
        self.leader_heading_rad = 0.0

        self.home_leader: Optional[HomePosition] = None
        self.home_follower: Optional[HomePosition] = None
        self.offset_lat = 0.0
        self.offset_lon = 0.0
        self.offset_ready = False

        # ---------------- Subscriptions ----------------
        # Follower own state / altitude / home
        self.create_subscription(State,    f'{self.follower_ns}/state',                   self._state_cb, qos_sensor)
        self.create_subscription(Float64,  f'{self.follower_ns}/global_position/rel_alt', self._alt_cb,   qos_sensor)
        self.create_subscription(HomePosition, f'{self.follower_ns}/home_position/home',  self._home_follower_cb, qos_sensor)

        # Leader state / gps / rel alt / heading / home
        self.create_subscription(State,    f'{self.leader_ns}/state',                     self._leader_state_cb,   qos_sensor)
        self.create_subscription(NavSatFix,f'{self.leader_ns}/global_position/global',    self._leader_gps_cb,     qos_sensor)
        self.create_subscription(Float64,  f'{self.leader_ns}/global_position/rel_alt',   self._leader_rel_alt_cb, qos_sensor)
        self.create_subscription(Float64,  f'{self.leader_ns}/global_position/compass_hdg', self._leader_heading_cb, qos_sensor)
        self.create_subscription(HomePosition, f'{self.leader_ns}/home_position/home',    self._home_leader_cb,    qos_sensor)

        # ---------------- Publisher ----------------
        self.sp_pub = self.create_publisher(GlobalPositionTarget, f'{self.follower_ns}/setpoint_raw/global', 10)

        # ---------------- Services ----------------
        self.arm_cli     = self.create_client(CommandBool, f'{self.follower_ns}/cmd/arming')
        self.tko_cli     = self.create_client(CommandTOL,  f'{self.follower_ns}/cmd/takeoff')
        self.setmode_cli = self.create_client(SetMode,     f'{self.follower_ns}/set_mode')

        self.get_logger().info('Waiting for MAVROS services...')
        for cli in (self.arm_cli, self.tko_cli, self.setmode_cli):
            cli.wait_for_service()
        self.get_logger().info('MAVROS services ready.')

    # ---------------- Callbacks ----------------
    def _state_cb(self, msg: State): self.my_state = msg
    def _alt_cb(self, msg: Float64): self.my_rel_alt = float(msg.data)

    def _leader_state_cb(self, msg: State):
        prev = self.leader_state.mode if hasattr(self.leader_state, 'mode') else ''
        self.leader_state = msg
        if msg.mode == 'RTL' and prev != 'RTL':
            self.get_logger().warn('Leader entered RTL → switching follower to RTL.')
            self._set_mode('RTL')

    def _leader_gps_cb(self, msg: NavSatFix): self.leader_gps = msg
    def _leader_rel_alt_cb(self, msg: Float64): self.leader_rel_alt = float(msg.data)
    def _leader_heading_cb(self, msg: Float64):
        # Input in degrees, convert to radians
        self.leader_heading_rad = math.radians(float(msg.data))

    def _home_leader_cb(self, msg: HomePosition):
        self.home_leader = msg
        self._compute_offset_if_ready()

    def _home_follower_cb(self, msg: HomePosition):
        self.home_follower = msg
        self._compute_offset_if_ready()

    # ---------------- Helpers ----------------
    def _compute_offset_if_ready(self):
        if self.home_leader is None or self.home_follower is None:
            return
        # dLat/dLon in *degrees* (small delta is fine within local area)
        self.offset_lat = float(self.home_follower.geo.latitude  - self.home_leader.geo.latitude)
        self.offset_lon = float(self.home_follower.geo.longitude - self.home_leader.geo.longitude)
        self.offset_ready = True
        self.get_logger().info(f'Home offset ready: dLat={self.offset_lat:.7f}, dLon={self.offset_lon:.7f}')

    def _set_mode(self, mode: str) -> bool:
        req = SetMode.Request(); req.base_mode = 0; req.custom_mode = mode
        fut = self.setmode_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        ok = bool(fut.done() and fut.result() and getattr(fut.result(), 'mode_sent', False))
        if ok:
            self.get_logger().info(f"SetMode('{mode}') OK")
        else:
            self.get_logger().error(f"SetMode('{mode}') FAILED")
        return ok

    def _arm(self) -> bool:
        req = CommandBool.Request(); req.value = True
        fut = self.arm_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        ok = bool(fut.done() and fut.result() and getattr(fut.result(), 'success', False))
        self.get_logger().info(f'Arm -> {ok}')
        return ok

    def _takeoff(self, alt: float) -> bool:
        req = CommandTOL.Request()
        req.altitude = float(alt); req.latitude=0.0; req.longitude=0.0; req.min_pitch=0.0; req.yaw=float('nan')
        fut = self.tko_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        ok = bool(fut.done() and fut.result() and getattr(fut.result(), 'success', False))
        self.get_logger().info(f'Takeoff({alt}) -> {ok}')
        return ok

    def _publish_global_target(self, lat_deg: float, lon_deg: float, rel_alt_m: float, yaw_rad: float):
        sp = GlobalPositionTarget()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        sp.type_mask = (
            GlobalPositionTarget.IGNORE_VX | GlobalPositionTarget.IGNORE_VY | GlobalPositionTarget.IGNORE_VZ |
            GlobalPositionTarget.IGNORE_AFX | GlobalPositionTarget.IGNORE_AFY | GlobalPositionTarget.IGNORE_AFZ |
            GlobalPositionTarget.IGNORE_YAW_RATE
        )
        sp.latitude = float(lat_deg)
        sp.longitude = float(lon_deg)
        sp.altitude = float(rel_alt_m)  # REL_ALT frame → meters above home
        if self.copy_yaw:
            sp.yaw = float(yaw_rad)
        self.sp_pub.publish(sp)

    # ---------------- Mission ----------------
    def run(self):
        # Wait FCU
        while rclpy.ok() and not self.my_state.connected:
            self.get_logger().info('Waiting FCU…')
            rclpy.spin_once(self, timeout_sec=0.2)

        # Mode + arm + takeoff
        self._set_mode(self.guided_mode)
        while rclpy.ok() and not self.my_state.armed:
            if not self._arm():
                self.get_logger().warn('Arming failed, retrying…')
            rclpy.spin_once(self, timeout_sec=0.2)

        if not self._takeoff(self.target_alt):
            self.get_logger().error('Takeoff failed.')
            return
        while rclpy.ok() and self.my_rel_alt < (self.target_alt - self.alt_thresh):
            rclpy.spin_once(self, timeout_sec=0.2)
        self.get_logger().info(f'Follower airborne at {self.my_rel_alt:.1f} m')

        # Wait for home offsets and valid leader GPS
        self.get_logger().info('Waiting for home positions and leader GPS…')
        while rclpy.ok() and not self.offset_ready:
            rclpy.spin_once(self, timeout_sec=0.2)
        while rclpy.ok() and (math.isnan(self.leader_gps.latitude) or math.isnan(self.leader_gps.longitude)):
            rclpy.spin_once(self, timeout_sec=0.2)

        self.get_logger().info('GPS shadow engaged (global).')
        dt = 1.0 / max(1e-3, self.sp_rate_hz)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)

            # Check for leader RTL (handled in callback too, but keep a guard)
            if self.leader_state.mode == 'RTL':
                self._set_mode('RTL')
                break

            # Compose target from leader GPS + home-offset
            tgt_lat = float(self.leader_gps.latitude)  + self.offset_lat
            tgt_lon = float(self.leader_gps.longitude) + self.offset_lon
            tgt_alt = float(self.leader_rel_alt)       # relative altitude to follower's home (REL_ALT frame)

            self._publish_global_target(tgt_lat, tgt_lon, tgt_alt, self.leader_heading_rad)
            time.sleep(dt)


def main():
    rclpy.init()
    node = GPSShadowFollower()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
