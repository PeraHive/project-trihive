#!/usr/bin/env python3
import time
import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode


def deg2rad(d: float) -> float:
    return d * math.pi / 180.0


def enu_offset_between(lat_ref: float, lon_ref: float, alt_ref: float,
                       lat: float, lon: float, alt: float) -> Tuple[float, float, float]:
    """
    Return (dE, dN, dU) that takes a point at (lat, lon, alt) expressed relative to
    (lat_ref, lon_ref, alt_ref) using a small-angle ENU approximation.
    """
    R = 6378137.0  # WGS-84 equatorial radius (meters)
    lat_ref_rad = deg2rad(lat_ref)
    dlat = deg2rad(lat - lat_ref)
    dlon = deg2rad(lon - lon_ref)

    dN = R * dlat
    dE = R * math.cos(lat_ref_rad) * dlon
    dU = alt - alt_ref
    return (dE, dN, dU)


class PointShadowFollower(Node):
    def __init__(self):
        super().__init__('point_shadow_follower')

        # --- QoS for sensor-ish topics ---
        qos_sensor = QoSProfile(depth=10)
        qos_sensor.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_sensor.durability  = QoSDurabilityPolicy.VOLATILE

        # --- Parameters ---
        self.follower_ns = self.declare_parameter('follower_ns', '/uav3').get_parameter_value().string_value
        self.leader_ns   = self.declare_parameter('leader_ns',   '/uav1').get_parameter_value().string_value
        self.target_alt  = float(self.declare_parameter('target_alt', 3.0).get_parameter_value().double_value)
        self.alt_thresh  = float(self.declare_parameter('alt_air_thresh', 0.5).get_parameter_value().double_value)
        self.sp_rate_hz  = float(self.declare_parameter('sp_rate_hz', 12.0).get_parameter_value().double_value)
        self.guided_mode = self.declare_parameter('guided_mode', 'GUIDED').get_parameter_value().string_value
        # Optional spacing (ENU, meters). Default 0s → sit exactly on leader.
        self.offset_e    = float(self.declare_parameter('offset_e', 0.0).get_parameter_value().double_value)
        self.offset_n    = float(self.declare_parameter('offset_n', 0.0).get_parameter_value().double_value)
        self.offset_u    = float(self.declare_parameter('offset_u', 0.0).get_parameter_value().double_value)  # +up

        # --- Failsafe parameter (NEW) ---
        self.leader_land_thresh = float(self.declare_parameter('leader_land_thresh', 3.0).get_parameter_value().double_value)

        # --- State ---
        self.my_state: State = State()
        self.my_rel_alt: float = 0.0

        self.leader_pose: PoseStamped = PoseStamped()
        self.leader_rel_alt: float = 0.0

        # First GPS fixes we treat as "homes" (lat, lon, alt)
        self.follower_home_gps: Optional[Tuple[float, float, float]] = None
        self.leader_home_gps:   Optional[Tuple[float, float, float]] = None

        # Precomputed home offset ENU (leader-home in follower-frame)
        self.home_offset_enu: Optional[Tuple[float, float, float]] = None

        # --- Subscriptions (Follower) ---
        self.create_subscription(State,   f'{self.follower_ns}/state',                        self._state_cb,     qos_sensor)
        self.create_subscription(Float64, f'{self.follower_ns}/global_position/rel_alt',      self._alt_cb,       qos_sensor)
        self.create_subscription(NavSatFix, f'{self.follower_ns}/global_position/global',     self._follower_gps_cb, qos_sensor)

        # --- Subscriptions (Leader) ---
        self.create_subscription(PoseStamped, f'{self.leader_ns}/local_position/pose',        self._leader_pose_cb, qos_sensor)
        self.create_subscription(Float64,     f'{self.leader_ns}/global_position/rel_alt',    self._leader_alt_cb,  qos_sensor)
        self.create_subscription(NavSatFix,   f'{self.leader_ns}/global_position/global',     self._leader_gps_cb,  qos_sensor)

        # --- Publisher ---
        self.sp_pub = self.create_publisher(PositionTarget, f'{self.follower_ns}/setpoint_raw/local', 10)

        # --- Services ---
        self.arm_cli     = self.create_client(CommandBool, f'{self.follower_ns}/cmd/arming')
        self.tko_cli     = self.create_client(CommandTOL,  f'{self.follower_ns}/cmd/takeoff')
        self.setmode_cli = self.create_client(SetMode,     f'{self.follower_ns}/set_mode')

        self.get_logger().info('Waiting for MAVROS services...')
        for cli in (self.arm_cli, self.tko_cli, self.setmode_cli):
            cli.wait_for_service()
        self.get_logger().info('MAVROS services ready.')

    # ---------- Callbacks ----------
    def _state_cb(self, msg: State):
        self.my_state = msg

    def _alt_cb(self, msg: Float64):
        self.my_rel_alt = float(msg.data)

    def _leader_pose_cb(self, msg: PoseStamped):
        # Leader local pose is ENU in *leader’s* local frame
        self.leader_pose = msg

    def _leader_alt_cb(self, msg: Float64):
        self.leader_rel_alt = float(msg.data)

    def _follower_gps_cb(self, msg: NavSatFix):
        if self.follower_home_gps is None and self._gps_valid(msg):
            self.follower_home_gps = (float(msg.latitude), float(msg.longitude), float(msg.altitude))
            self.get_logger().info(f'Follower home GPS latched: {self.follower_home_gps}')
            self._maybe_compute_home_offset()

    def _leader_gps_cb(self, msg: NavSatFix):
        if self.leader_home_gps is None and self._gps_valid(msg):
            self.leader_home_gps = (float(msg.latitude), float(msg.longitude), float(msg.altitude))
            self.get_logger().info(f'Leader home GPS latched: {self.leader_home_gps}')
            self._maybe_compute_home_offset()

    # ---------- Helpers ----------
    def _gps_valid(self, fix: NavSatFix) -> bool:
        if math.isnan(fix.latitude) or math.isnan(fix.longitude):
            return False
        # status.status >= 0 is a good default (0=UNAVAILABLE in some stacks; adjust if needed)
        return True

    def _maybe_compute_home_offset(self):
        if self.leader_home_gps and self.follower_home_gps and self.home_offset_enu is None:
            latL, lonL, altL = self.leader_home_gps
            latF, lonF, altF = self.follower_home_gps
            # Offset that expresses the leader-home origin in the follower ENU frame:
            dE, dN, dU = enu_offset_between(latF, lonF, altF, latL, lonL, altL)
            self.home_offset_enu = (dE, dN, dU)
            self.get_logger().info(f'Computed home ENU offset (leader->follower frame): dE={dE:.2f}, dN={dN:.2f}, dU={dU:.2f}')

    def _set_mode(self, mode: str) -> bool:
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = mode
        fut = self.setmode_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        return bool(fut.done() and fut.result() and getattr(fut.result(), 'mode_sent', False))

    def _arm(self) -> bool:
        req = CommandBool.Request()
        req.value = True
        fut = self.arm_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        return bool(fut.done() and fut.result() and getattr(fut.result(), 'success', False))

    def _takeoff(self, alt: float) -> bool:
        req = CommandTOL.Request()
        req.altitude = float(alt)
        req.latitude = 0.0
        req.longitude = 0.0
        req.min_pitch = 0.0
        req.yaw = float('nan')
        fut = self.tko_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        return bool(fut.done() and fut.result() and getattr(fut.result(), 'success', False))

    def _publish_target_from_leader(self):
        """
        Take the leader’s ENU local pose (in leader frame), optionally add desired spacing,
        then rebase into the follower’s local ENU frame using the precomputed home offset.
        Publish as a local setpoint (no axis flips; MAVROS handles FCU conversion).
        """
        if self.home_offset_enu is None:
            self.get_logger().warn_throttle(2000, 'Home ENU offset not yet known; skipping setpoint.')
            return

        # Leader ENU pose (leader frame)
        xE = float(self.leader_pose.pose.position.x)
        yN = float(self.leader_pose.pose.position.y)
        zU = float(self.leader_pose.pose.position.z)

        # Optional desired spacing relative to leader (in ENU, meters)
        xE += self.offset_e
        yN += self.offset_n
        zU += self.offset_u

        dE_home, dN_home, dU_home = self.home_offset_enu

        # Rebase into follower frame
        xE_f = xE + dE_home
        yN_f = yN + dN_home
        zU_f = zU + dU_home

        sp = PositionTarget()
        sp.header.stamp = self.get_clock().now().to_msg()
        # NOTE: MAVROS expects ENU on ROS side; keep LOCAL_NED constant for MAVLink framing
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        sp.type_mask = (
            PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
        )
        sp.position.x = xE_f
        sp.position.y = yN_f
        sp.position.z = zU_f

        self.sp_pub.publish(sp)

    # ---------- Mission ----------
    def run(self):
        # Wait FCU
        while rclpy.ok() and not self.my_state.connected:
            self.get_logger().info('Waiting FCU…')
            rclpy.spin_once(self, timeout_sec=0.2)

        # Wait until we have latched both "homes"
        self.get_logger().info('Waiting for leader & follower GPS homes…')
        while rclpy.ok() and (self.follower_home_gps is None or self.leader_home_gps is None):
            rclpy.spin_once(self, timeout_sec=0.2)

        # Compute offset (if not already)
        self._maybe_compute_home_offset()

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

        # Wait for leader airborne (optional but helps)
        while rclpy.ok() and self.leader_rel_alt < (self.target_alt - self.alt_thresh):
            self.get_logger().info('Waiting for leader airborne…')
            rclpy.spin_once(self, timeout_sec=0.3)

        # Loop: mirror leader position with proper frame rebasing
        self.get_logger().info('Point shadow engaged (rebased to follower ENU).')
        dt = 1.0 / max(1e-3, self.sp_rate_hz)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)

            # --- FAILSAFE: leader low altitude -> LAND (NEW) ---
            if self.leader_rel_alt < self.leader_land_thresh:
                self.get_logger().warn(
                    f'Failsafe: leader altitude {self.leader_rel_alt:.2f} m < {self.leader_land_thresh:.2f} m. '
                    'Switching to LAND.'
                )
                if not self._set_mode('LAND'):
                    self.get_logger().error('Failed to switch to LAND mode.')
                # Stop sending setpoints once landing is initiated
                return

            self._publish_target_from_leader()
            time.sleep(dt)


def main():
    rclpy.init()
    node = PointShadowFollower()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
