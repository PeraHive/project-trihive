
#!/usr/bin/env python3
import time, math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, GlobalPositionTarget, HomePosition, PositionTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode


def meters_to_latlon(m_north: float, m_east: float, ref_lat_deg: float):
    """Approx convert meters in N/E to degrees lat/lon near ref latitude."""
    lat_rad = math.radians(ref_lat_deg)
    dlat = m_north / 111320.0
    dlon = m_east / (111320.0 * math.cos(lat_rad) + 1e-9)
    return dlat, dlon


class SimpleLeaderGPS(Node):
    def __init__(self) -> None:
        super().__init__('shadow_leader_gps')

        qos_sensor = QoSProfile(depth=10)
        qos_sensor.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_sensor.durability  = QoSDurabilityPolicy.VOLATILE

        # ---------------- Params (match your style) ----------------
        self.uav1_ns = self.declare_parameter('uav1_ns', '/uav1').get_parameter_value().string_value
        self.guided_mode  = self.declare_parameter('guided_mode', 'GUIDED').get_parameter_value().string_value
        self.poshold_mode = self.declare_parameter('poshold_mode', 'POSHOLD').get_parameter_value().string_value
        self.target_alt   = float(self.declare_parameter('target_alt', 3.0).get_parameter_value().double_value)
        self.alt_thresh   = float(self.declare_parameter('alt_air_thresh', 0.5).get_parameter_value().double_value)
        self.sp_rate_hz   = float(self.declare_parameter('sp_rate_hz', 10.0).get_parameter_value().double_value)

        # Trial manoeuvre params (keep similar semantics to your leader)
        self.forward_m = float(self.declare_parameter('trial_forward_m', 5.0).get_parameter_value().double_value)
        self.climb_m   = float(self.declare_parameter('trial_climb_m',   2.0).get_parameter_value().double_value)
        self.hold_after_trial = self.declare_parameter('hold_after_trial', True).get_parameter_value().bool_value

        # ---------------- State ----------------
        self.state = State()
        self.rel_alt = 0.0
        self.pose = PoseStamped()
        self.home: Optional[HomePosition] = None
        self.gps = NavSatFix()
        self.heading_deg = 0.0

        # ---------------- I/O ----------------
        self.create_subscription(State,        f'{self.uav1_ns}/state',                    self._state_cb, qos_sensor)
        self.create_subscription(Float64,      f'{self.uav1_ns}/global_position/rel_alt',  self._alt_cb,   qos_sensor)
        self.create_subscription(PoseStamped,  f'{self.uav1_ns}/local_position/pose',      self._pose_cb,  qos_sensor)
        self.create_subscription(NavSatFix,    f'{self.uav1_ns}/global_position/global',   self._gps_cb,   qos_sensor)
        self.create_subscription(Float64,      f'{self.uav1_ns}/global_position/compass_hdg', self._hdg_cb, qos_sensor)
        self.create_subscription(HomePosition,  f'{self.uav1_ns}/home_position/home',       self._home_cb,  qos_sensor)

        self.sp_pub_global = self.create_publisher(GlobalPositionTarget, f'{self.uav1_ns}/setpoint_raw/global', 10)
        self.sp_pub_local  = self.create_publisher(PositionTarget,       f'{self.uav1_ns}/setpoint_raw/local',  10)

        self.arm_cli     = self.create_client(CommandBool, f'{self.uav1_ns}/cmd/arming')
        self.tko_cli     = self.create_client(CommandTOL,  f'{self.uav1_ns}/cmd/takeoff')
        self.setmode_cli = self.create_client(SetMode,     f'{self.uav1_ns}/set_mode')

        self.get_logger().info('Waiting for MAVROS services...')
        for cli in (self.arm_cli, self.tko_cli, self.setmode_cli):
            cli.wait_for_service()
        self.get_logger().info('MAVROS services are available.')

    # ---------------- Callbacks ----------------
    def _state_cb(self, msg: State): self.state = msg
    def _alt_cb(self, msg: Float64): self.rel_alt = float(msg.data)
    def _pose_cb(self, msg: PoseStamped): self.pose = msg
    def _gps_cb(self, msg: NavSatFix): self.gps = msg
    def _hdg_cb(self, msg: Float64): self.heading_deg = float(msg.data)
    def _home_cb(self, msg: HomePosition): self.home = msg

    # ---------------- Helpers ----------------
    def _set_mode(self, mode: str) -> bool:
        req = SetMode.Request(); req.base_mode = 0; req.custom_mode = mode
        fut = self.setmode_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        ok = bool(fut.done() and fut.result() and getattr(fut.result(), 'mode_sent', False))
        self.get_logger().info(f"SetMode('{mode}') -> {ok}")
        return ok

    def _arm(self) -> bool:
        req = CommandBool.Request(); req.value = True
        fut = self.arm_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        ok = bool(fut.done() and fut.result() and getattr(fut.result(), 'success', False))
        self.get_logger().info(f'Arming -> {ok}')
        return ok

    def _takeoff(self, alt: float) -> bool:
        req = CommandTOL.Request()
        req.altitude = float(alt); req.latitude = 0.0; req.longitude = 0.0; req.min_pitch = 0.0; req.yaw = float('nan')
        fut = self.tko_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        ok = bool(fut.done() and fut.result() and getattr(fut.result(), 'success', False))
        self.get_logger().info(f'Takeoff({alt}) -> {ok}')
        return ok

    def _publish_global(self, lat_deg: float, lon_deg: float, rel_alt_m: float, yaw_rad: float):
        sp = GlobalPositionTarget()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        sp.type_mask = (GlobalPositionTarget.IGNORE_VX | GlobalPositionTarget.IGNORE_VY | GlobalPositionTarget.IGNORE_VZ |
                        GlobalPositionTarget.IGNORE_AFX | GlobalPositionTarget.IGNORE_AFY | GlobalPositionTarget.IGNORE_AFZ |
                        GlobalPositionTarget.IGNORE_YAW_RATE)
        sp.latitude = float(lat_deg)
        sp.longitude = float(lon_deg)
        sp.altitude = float(rel_alt_m)
        sp.yaw = float(yaw_rad)
        self.sp_pub_global.publish(sp)

    def _hold_local_here(self):
        sp = PositionTarget()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        sp.type_mask = (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
                        PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                        PositionTarget.IGNORE_YAW_RATE)
        sp.position.x = float(self.pose.pose.position.x)
        sp.position.y = float(self.pose.pose.position.y)
        sp.position.z = float(self.pose.pose.position.z)
        self.sp_pub_local.publish(sp)

    # ---------------- Mission ----------------
    def run(self) -> None:
        # Wait FCU
        while rclpy.ok() and not self.state.connected:
            self.get_logger().info('Leader waiting for FCU...')
            rclpy.spin_once(self, timeout_sec=0.2)

        # GUIDED
        self._set_mode(self.guided_mode)

        # Arm
        while rclpy.ok() and not self.state.armed:
            self._arm()
            rclpy.spin_once(self, timeout_sec=0.2)

        # Takeoff
        if not self._takeoff(self.target_alt):
            self.get_logger().error('Leader takeoff failed.')
            return
        while rclpy.ok() and self.rel_alt < (self.target_alt - self.alt_thresh):
            rclpy.spin_once(self, timeout_sec=0.2)
        self.get_logger().info(f'Leader airborne: rel_alt={self.rel_alt:.1f} m')

        # Wait for home + GPS lock
        self.get_logger().info('Waiting for home position and GPS fix...')
        while rclpy.ok() and self.home is None:
            rclpy.spin_once(self, timeout_sec=0.2)
        while rclpy.ok() and (math.isnan(self.gps.latitude) or math.isnan(self.gps.longitude)):
            rclpy.spin_once(self, timeout_sec=0.2)

        # ---------- GPS manoeuvre: forward then climb ----------
        # lat0 = float(self.gps.latitude)
        # lon0 = float(self.gps.longitude)
        # hdg_rad = math.radians(self.heading_deg)

        # # a) forward along heading (meters → degrees)
        # dn = math.cos(hdg_rad) * self.forward_m   # north
        # de = math.sin(hdg_rad) * self.forward_m   # east
        # dlat, dlon = meters_to_latlon(dn, de, lat0)
        # tgt1_lat = lat0 + dlat
        # tgt1_lon = lon0 + dlon
        # tgt1_alt = self.target_alt

        # self.get_logger().info(f'Move forward {self.forward_m:.1f} m in global frame')
        dt = 1.0 / max(1e-3, self.sp_rate_hz)
        # for _ in range(int(3 * self.sp_rate_hz)):
        #     rclpy.spin_once(self, timeout_sec=0.0)
        #     self._publish_global(tgt1_lat, tgt1_lon, tgt1_alt, hdg_rad)
        #     time.sleep(dt)

        # # b) climb
        # tgt2_lat, tgt2_lon = tgt1_lat, tgt1_lon
        # tgt2_alt = self.target_alt + self.climb_m
        # self.get_logger().info(f'Climb +{self.climb_m:.1f} m in place (global REL_ALT)')
        # for _ in range(int(3 * self.sp_rate_hz)):
        #     rclpy.spin_once(self, timeout_sec=0.0)
        #     self._publish_global(tgt2_lat, tgt2_lon, tgt2_alt, hdg_rad)
        #     time.sleep(dt)

        # Hold (POSHOLD) if requested
        if self.hold_after_trial:
            self._set_mode("AUTO")
            # self._set_mode(self.poshold_mode)

            self.get_logger().info('Trial done → switched to position hold.')
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)
            #     self._hold_local_here()
                time.sleep(dt)


def main():
    rclpy.init()
    node = SimpleLeaderGPS()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
