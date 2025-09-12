#!/usr/bin/env python3
import time
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

import csv
import os
from datetime import datetime


class SimpleFollower(Node):
    def __init__(self) -> None:
        super().__init__('shadow_follower_simple')

        qos_sensor = QoSProfile(depth=10)
        qos_sensor.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_sensor.durability  = QoSDurabilityPolicy.VOLATILE

        # ---------------- Params ----------------
        self.follower_ns = self.declare_parameter('follower_ns', '/uav2').get_parameter_value().string_value
        self.leader_ns   = self.declare_parameter('leader_ns',   '/uav1').get_parameter_value().string_value
        self.side        = self.declare_parameter('side', 'right').get_parameter_value().string_value.lower()  # 'left' or 'right'
        self.slot_d      = float(self.declare_parameter('slot_offset_m', 3.0).get_parameter_value().double_value)
        self.target_alt  = float(self.declare_parameter('target_alt', 3.0).get_parameter_value().double_value)
        self.alt_thresh  = float(self.declare_parameter('alt_air_thresh', 0.5).get_parameter_value().double_value)
        self.sp_rate_hz  = float(self.declare_parameter('sp_rate_hz', 12.0).get_parameter_value().double_value)
        self.guided_mode = self.declare_parameter('guided_mode', 'GUIDED').get_parameter_value().string_value

        # Convert leader pose into follower's frame? (True for field; False for single-frame sims)
        self.use_frame_bridge = bool(self.declare_parameter('use_frame_bridge', True).get_parameter_value().bool_value)

        # ---------------- State ----------------
        self.my_state = State()
        self.my_rel_alt = 0.0
        self.my_pose = PoseStamped()
        self.leader_pose = PoseStamped()
        self.leader_rel_alt = 0.0

        # GPS home snapshots (for frame bridging)
        self.leader_home: Optional[Tuple[float, float, float]] = None  # (lat, lon, alt)
        self.follower_home: Optional[Tuple[float, float, float]] = None

        # ---------------- I/O ----------------
        # Follower topics
        self.create_subscription(State, f'{self.follower_ns}/state', self._state_cb, qos_sensor)
        self.create_subscription(Float64, f'{self.follower_ns}/global_position/rel_alt', self._alt_cb, qos_sensor)
        self.create_subscription(PoseStamped, f'{self.follower_ns}/local_position/pose', self._pose_cb, qos_sensor)
        if self.use_frame_bridge:
            self.create_subscription(NavSatFix, f'{self.follower_ns}/global_position/global', self._follower_gps_cb, qos_sensor)

        # Leader topics
        self.create_subscription(PoseStamped, f'{self.leader_ns}/local_position/pose', self._leader_pose_cb, qos_sensor)
        self.create_subscription(Float64, f'{self.leader_ns}/global_position/rel_alt', self._leader_alt_cb, qos_sensor)
        if self.use_frame_bridge:
            self.create_subscription(NavSatFix, f'{self.leader_ns}/global_position/global', self._leader_gps_cb, qos_sensor)

        # Setpoint publisher
        self.sp_pub = self.create_publisher(PositionTarget, f'{self.follower_ns}/setpoint_raw/local', 10)

        # Services
        self.arm_cli     = self.create_client(CommandBool, f'{self.follower_ns}/cmd/arming')
        self.tko_cli     = self.create_client(CommandTOL,  f'{self.follower_ns}/cmd/takeoff')
        self.setmode_cli = self.create_client(SetMode,     f'{self.follower_ns}/set_mode')

        # ---------------- CSV Logging ----------------
        log_dir = os.path.expanduser("~/shadow_logs")
        os.makedirs(log_dir, exist_ok=True)
        log_name = datetime.now().strftime("follower_log_%Y%m%d_%H%M%S.csv")
        self.log_path = os.path.join(log_dir, log_name)
        self.log_file = open(self.log_path, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        # Header (includes raw leader pose and bridged leader pose)
        self.csv_writer.writerow([
            "time_sec",
            "lx","ly","lz","lyaw_deg",          # leader raw (leader frame)
            "blx","bly","blz",                  # leader bridged (in follower frame)
            "gx","gy","gz","gyaw_deg",          # target in follower frame
            "fx","fy","fz","fyaw_deg"           # follower pose
        ])
        self.get_logger().info(f"Logging follower data to {self.log_path}")

        self.get_logger().info('Waiting for follower MAVROS services...')
        for cli in (self.arm_cli, self.tko_cli, self.setmode_cli):
            cli.wait_for_service()
        self.get_logger().info('Follower MAVROS services are available.')

    # ---------------- Callbacks ----------------
    def _state_cb(self, msg: State): self.my_state = msg
    def _alt_cb(self, msg: Float64): self.my_rel_alt = float(msg.data)
    def _pose_cb(self, msg: PoseStamped): self.my_pose = msg
    def _leader_pose_cb(self, msg: PoseStamped): self.leader_pose = msg
    def _leader_alt_cb(self, msg: Float64): self.leader_rel_alt = float(msg.data)

    def _leader_gps_cb(self, msg: NavSatFix):
        if self.leader_home is None and not math.isnan(msg.latitude) and msg.status.status != -1:
            self.leader_home = (float(msg.latitude), float(msg.longitude), float(msg.altitude))
            self.get_logger().info(f'Leader home set: {self.leader_home}')

    def _follower_gps_cb(self, msg: NavSatFix):
        if self.follower_home is None and not math.isnan(msg.latitude) and msg.status.status != -1:
            self.follower_home = (float(msg.latitude), float(msg.longitude), float(msg.altitude))
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

    def _takeoff(self, alt: float, timeout: float = 10.0) -> bool:
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
        return float(p.pose.position.x), float(p.pose.position.y), float(p.pose.position.z), float(yaw)

    def _slot_offset_xy(self, yaw: float, d: float) -> Tuple[float, float]:
        # ENU: left = (-sinψ, +cosψ)*d ; right = ( +sinψ, -cosψ)*d
        if self.side == 'left':
            return (-math.sin(yaw) * d,  math.cos(yaw) * d)
        else:
            return ( math.sin(yaw) * d, -math.cos(yaw) * d)

    @staticmethod
    def _enu_offset(lat0, lon0, alt0, lat1, lon1, alt1):
        """Approx ENU translation from (lat0,lon0,alt0) -> (lat1,lon1,alt1) around (lat0,lon0)."""
        R = 6378137.0
        lat0r = math.radians(lat0)
        dlat = math.radians(lat1 - lat0)
        dlon = math.radians(lon1 - lon0)
        dE = dlon * math.cos(lat0r) * R
        dN = dlat * R
        dU = (alt1 - alt0)
        return dE, dN, dU  # ENU

    def _bridge_leader_pose_if_needed(self, lx, ly, lz):
        """Translate leader local pose into follower local frame (if enabled and homes known)."""
        if not self.use_frame_bridge:
            return lx, ly, lz
        if self.leader_home is None or self.follower_home is None:
            return lx, ly, lz
        # follower_home is the origin; shift leader pose by the ENU delta between homes
        f_lat, f_lon, f_alt = self.follower_home
        l_lat, l_lon, l_alt = self.leader_home
        dE, dN, dU = self._enu_offset(f_lat, f_lon, f_alt, l_lat, l_lon, l_alt)
        return lx + dE, ly + dN, lz + dU

    def _publish_target_xyz(self, x: float, y: float, z: float):
        sp = PositionTarget()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED  # MAVROS handles ENU<->NED
        sp.type_mask = (
            PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE   # yaw ignored entirely
        )
        sp.position.x = float(x); sp.position.y = float(y); sp.position.z = float(z)
        sp.yaw = float(0.0)  # ignored
        self.sp_pub.publish(sp)

    # ---------------- Mission ----------------
    def run(self) -> None:
        # Wait FCU
        while rclpy.ok() and not self.my_state.connected:
            self.get_logger().info('Follower waiting for FCU connection...')
            rclpy.spin_once(self, timeout_sec=0.2)

        # Mode + arm + takeoff
        self._set_mode(self.guided_mode)
        while rclpy.ok() and not self.my_state.armed:
            if not self._arm_once():
                self.get_logger().warn('Follower arming failed, retrying...')
            rclpy.spin_once(self, timeout_sec=0.2)

        if not self._takeoff(self.target_alt):
            self.get_logger().error('Follower takeoff failed; aborting.')
            return

        while rclpy.ok() and self.my_rel_alt < (self.target_alt - self.alt_thresh):
            rclpy.spin_once(self, timeout_sec=0.2)
        self.get_logger().info(f'Follower airborne: rel_alt={self.my_rel_alt:.1f} m')

        # Wait for leader airborne
        while rclpy.ok() and self.leader_rel_alt < (self.target_alt - self.alt_thresh):
            self.get_logger().info('Waiting for leader to be airborne...')
            rclpy.spin_once(self, timeout_sec=0.3)

              # Track leader with lateral offset; altitude = leader rel_alt (GPS-agnostic)
        self.get_logger().info(f'Holding {self.side.upper()} slot at {self.slot_d:.1f} m from leader... (GPS-only bridge)')
        dt = 1.0 / max(1e-3, self.sp_rate_hz)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)

            # --- 1) Read leader pose (for yaw only) + leader rel_alt (for Z) ---
            lx_raw, ly_raw, lz_raw, lyaw = self._pose_xyz_yaw(self.leader_pose)  # yaw source only
            leader_rel_z = float(self.leader_rel_alt)  # match leader altitude reference

            # --- 2) Ensure GPS inputs are ready (leader fix + follower home) ---
            if not (hasattr(self, "leader_fix") and self.leader_fix is not None and
                    self.leader_fix.status.status != -1 and
                    self.follower_home is not None):
                # Hold position until GPS is ready
                fx, fy, fz, _ = self._pose_xyz_yaw(self.my_pose)
                self.get_logger().warn('GPS bridge not ready (need leader NavSatFix and follower home). Holding…')
                # CSV: idle row
                t = self.get_clock().now().nanoseconds * 1e-9
                self.csv_writer.writerow([f"{t:.3f}",
                                          f"{lx_raw:.3f}", f"{ly_raw:.3f}", f"{lz_raw:.3f}", f"{math.degrees(lyaw):.2f}",
                                          f"{0.0:.3f}",    f"{0.0:.3f}",    f"{0.0:.3f}",
                                          f"{fx:.3f}",     f"{fy:.3f}",     f"{fz:.3f}"])
                self.log_file.flush()
                self._publish_target_xyz(fx, fy, fz)
                time.sleep(dt)
                continue

            # --- 3) XY from GPS: leader ENU w.r.t follower home ---
            f_lat, f_lon, f_alt = self.follower_home
            le, ln, lu = self._enu_from_ref(
                f_lat, f_lon, f_alt,
                float(self.leader_fix.latitude),
                float(self.leader_fix.longitude),
                float(self.leader_fix.altitude)
            )
            blx, bly = float(le), float(ln)   # bridged XY in follower frame

            # --- 4) Z from leader rel_alt (not GPS altitude) ---
            blz = float(leader_rel_z)

            # --- 5) Slot offset in follower frame (use leader yaw just for left/right) ---
            ox, oy = self._slot_offset_xy(lyaw, self.slot_d)
            gx, gy, gz = blx + ox, bly + oy, blz

            # --- 6) Follower current pose (for logs) ---
            fx, fy, fz, _ = self._pose_xyz_yaw(self.my_pose)

            # --- 7) Console + CSV logs ---
            self.get_logger().info(
                f'LeaderRaw=({lx_raw:.1f},{ly_raw:.1f},{lz_raw:.1f}, yaw={math.degrees(lyaw):.1f}°) | '
                f'LeaderGPS→Follower=({blx:.1f},{bly:.1f},{blz:.1f}) | '
                f'Target=({gx:.1f},{gy:.1f},{gz:.1f}) | '
                f'Follower=({fx:.1f},{fy:.1f},{fz:.1f})'
            )
            t = self.get_clock().now().nanoseconds * 1e-9
            self.csv_writer.writerow([
                f"{t:.3f}",
                f"{lx_raw:.3f}", f"{ly_raw:.3f}", f"{lz_raw:.3f}", f"{math.degrees(lyaw):.2f}",
                f"{blx:.3f}",    f"{bly:.3f}",    f"{blz:.3f}",
                f"{gx:.3f}",     f"{gy:.3f}",     f"{gz:.3f}",
                f"{fx:.3f}",     f"{fy:.3f}",     f"{fz:.3f}",
            ])
            self.log_file.flush()

            # --- 8) Publish setpoint in follower's local frame (yaw ignored) ---
            self._publish_target_xyz(gx, gy, gz)

            time.sleep(dt)


def main():
    rclpy.init()
    node = SimpleFollower()
    try:
        node.run()
    finally:
        try:
            node.log_file.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
