#!/usr/bin/env python3
import time
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State, GlobalPositionTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode


EARTH_RADIUS_M = 6378137.0  # WGS84

class GPSShadowFollower(Node):
    """
    GPS-based shadow follower (fixed lateral offset).

    Idea:
      - Track leader's current GPS.
      - Compute a fixed lateral offset (left/right of leader heading) in meters.
      - Publish a SET_POSITION_TARGET_GLOBAL_INT (REL_ALT frame) at leader_gps + offset.
      - Copy leader heading (optional) and fly at target_alt.
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

        # New: fixed lateral slot
        self.side = self.declare_parameter('side', 'right').get_parameter_value().string_value.lower()
        self.slot_offset_m = float(self.declare_parameter('slot_offset_m', 3.0).get_parameter_value().double_value)

        # ---------------- Internal state ----------------
        self.my_state = State()
        self.my_rel_alt = 0.0

        self.leader_state = State()
        self.leader_gps = NavSatFix()
        self.leader_rel_alt = 0.0
        self.leader_heading_rad = 0.0

        # ---------------- Subscriptions ----------------
        # Follower own state / altitude
        self.create_subscription(State,    f'{self.follower_ns}/state',                   self._state_cb, qos_sensor)
        self.create_subscription(Float64,  f'{self.follower_ns}/global_position/rel_alt', self._alt_cb,   qos_sensor)

        # Leader state / gps / rel alt / heading
        self.create_subscription(State,    f'{self.leader_ns}/state',                     self._leader_state_cb,   qos_sensor)
        self.create_subscription(NavSatFix,f'{self.leader_ns}/global_position/global',    self._leader_gps_cb,     qos_sensor)
        self.create_subscription(Float64,  f'{self.leader_ns}/global_position/rel_alt',   self._leader_rel_alt_cb, qos_sensor)
        self.create_subscription(Float64,  f'{self.leader_ns}/global_position/compass_hdg', self._leader_heading_cb, qos_sensor)

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

        # Validate side
        if self.side not in ('left', 'right'):
            self.get_logger().warn(f"Unknown side='{self.side}', defaulting to 'right'")
            self.side = 'right'

    # ---------------- Callbacks ----------------
    def _state_cb(self, msg: State): self.my_state = msg
    def _alt_cb(self, msg: Float64): self.my_rel_alt = float(msg.data)

    def _leader_state_cb(self, msg: State):
        prev = getattr(self.leader_state, 'mode', '')
        self.leader_state = msg
        if msg.mode == 'RTL' and prev != 'RTL':
            self.get_logger().warn('Leader entered RTL → switching follower to RTL.')
            self._set_mode('RTL')

    def _leader_gps_cb(self, msg: NavSatFix): self.leader_gps = msg
    def _leader_rel_alt_cb(self, msg: Float64): self.leader_rel_alt = float(msg.data)
    def _leader_heading_cb(self, msg: Float64):
        # Input in degrees, convert to radians
        self.leader_heading_rad = math.radians(float(msg.data))

    # ---------------- Helpers ----------------
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

    @staticmethod
    def _meters_to_deg(dn_m: float, de_m: float, ref_lat_deg: float):
        """Convert local N/E meters to dLat/dLon degrees at reference latitude."""
        dlat = (dn_m / EARTH_RADIUS_M) * (180.0 / math.pi)
        # guard cos(lat) near poles
        clat = max(1e-6, math.cos(math.radians(ref_lat_deg)))
        dlon = (de_m / (EARTH_RADIUS_M * clat)) * (180.0 / math.pi)
        return dlat, dlon

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

        # Wait for valid leader GPS
        self.get_logger().info('Waiting for leader GPS…')
        # Some sources publish 0.0 initially; wait until we see a plausible lat/lon
        def _gps_valid(g):
            return abs(g.latitude) > 1e-6 or abs(g.longitude) > 1e-6
        while rclpy.ok() and not _gps_valid(self.leader_gps):
            rclpy.spin_once(self, timeout_sec=0.2)

        self.get_logger().info('GPS shadow engaged (fixed lateral offset).')
        dt = 1.0 / max(1e-3, self.sp_rate_hz)

        # Lateral sign: +left, -right relative to leader heading
        lateral_m = self.slot_offset_m if self.side == 'left' else -self.slot_offset_m

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)

            # Check for leader RTL (also handled in callback)
            if self.leader_state.mode == 'RTL':
                self._set_mode('RTL')
                break

            # Compute lateral offset in local N/E meters from leader heading
            psi = self.leader_heading_rad  # radians, 0 = North, increases Eastward
            dn = -lateral_m * math.sin(psi)  # north component
            de =  lateral_m * math.cos(psi)  # east component

            # Convert to lat/lon degrees at leader latitude
            dlat_deg, dlon_deg = self._meters_to_deg(dn, de, self.leader_gps.latitude)

            tgt_lat = float(self.leader_gps.latitude)  + dlat_deg
            tgt_lon = float(self.leader_gps.longitude) + dlon_deg
            tgt_alt = float(self.target_alt)  # hold configured altitude

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
