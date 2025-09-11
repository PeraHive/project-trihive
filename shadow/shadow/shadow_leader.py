#!/usr/bin/env python3
"""
Shadow Leader (streaming setpoints, no speed limits)

- Arms and takes off to target_alt in GUIDED/OFFBOARD.
- Flies to a relative goal (dx, dy, dz) from current pose and sets target_yaw.
- Simple test sequence: rotate +90°, climb, move forward, descend, rotate +90°.
- Ends in POSHOLD/POSCTL.

Params:
  uav1_ns: '/uav1'
  uav2_ns: '/uav2'
  uav3_ns: '/uav3'
  guided_mode: 'GUIDED' | 'OFFBOARD'
  poshold_mode: 'POSHOLD' | 'POSCTL'
  target_alt: 10.0
  rel_dx: 0.0   (m, East+)
  rel_dy: 10.0  (m, North+)
  rel_dz: 0.0   (Up+; if 0, we hold target_alt)
  target_yaw: pi/2 (face North)
  sp_rate_hz: 12.0
  pos_tol: 0.8 m
  yaw_tol: 8 deg
"""

import math, time
from typing import Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode


class ShadowLeader(Node):
    def __init__(self) -> None:
        super().__init__('shadow_leader')

        qos_sensor = QoSProfile(depth=10)
        qos_sensor.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_sensor.durability  = QoSDurabilityPolicy.VOLATILE

        # ---------------- Parameters ----------------
        self.uav1_ns = self.declare_parameter('uav1_ns', '/uav1').get_parameter_value().string_value
        self.uav2_ns = self.declare_parameter('uav2_ns', '/uav2').get_parameter_value().string_value
        self.uav3_ns = self.declare_parameter('uav3_ns', '/uav3').get_parameter_value().string_value

        self.guided_mode  = self.declare_parameter('guided_mode', 'GUIDED').get_parameter_value().string_value
        self.poshold_mode = self.declare_parameter('poshold_mode', 'POSHOLD').get_parameter_value().string_value

        self.target_alt = float(self.declare_parameter('target_alt', 5.0).get_parameter_value().double_value)
        self.arm_retry_sec = float(self.declare_parameter('arm_retry_sec', 2.0).get_parameter_value().double_value)

        self.rel_dx = float(self.declare_parameter('rel_dx', 0.0).get_parameter_value().double_value)
        self.rel_dy = float(self.declare_parameter('rel_dy', 10.0).get_parameter_value().double_value)
        self.rel_dz = float(self.declare_parameter('rel_dz', 0.0).get_parameter_value().double_value)

        self.target_yaw = float(self.declare_parameter('target_yaw', math.pi/2).get_parameter_value().double_value)

        self.alt_air_thresh = float(self.declare_parameter('alt_air_thresh', 1).get_parameter_value().double_value)
        self.pos_tol = float(self.declare_parameter('pos_tol', 0.8).get_parameter_value().double_value)
        self.yaw_tol = float(self.declare_parameter('yaw_tol', math.radians(8.0)).get_parameter_value().double_value)

        self.sp_rate_hz = float(self.declare_parameter('sp_rate_hz', 12.0).get_parameter_value().double_value)

        # ---------------- Internal State ----------------
        self.state1 = State()
        self.rel_alt1 = 0.0
        self.rel_alt2 = 0.0
        self.rel_alt3 = 0.0
        self.pose1 = PoseStamped()

        # ---------------- I/O ----------------
        self.create_subscription(State, f'{self.uav1_ns}/state', self._state_cb, qos_sensor)
        self.create_subscription(Float64, f'{self.uav1_ns}/global_position/rel_alt', self._alt1_cb, qos_sensor)
        self.create_subscription(Float64, f'{self.uav2_ns}/global_position/rel_alt', self._alt2_cb, qos_sensor)
        self.create_subscription(Float64, f'{self.uav3_ns}/global_position/rel_alt', self._alt3_cb, qos_sensor)
        self.create_subscription(PoseStamped, f'{self.uav1_ns}/local_position/pose', self._pose1_cb, qos_sensor)

        self.raw_sp_pub = self.create_publisher(PositionTarget, f'{self.uav1_ns}/setpoint_raw/local', 10)

        self.arm_cli     = self.create_client(CommandBool, f'{self.uav1_ns}/cmd/arming')
        self.tko_cli     = self.create_client(CommandTOL,  f'{self.uav1_ns}/cmd/takeoff')
        self.setmode_cli = self.create_client(SetMode,     f'{self.uav1_ns}/set_mode')

        self.get_logger().info('Waiting for MAVROS services...')
        for cli in (self.arm_cli, self.tko_cli, self.setmode_cli):
            cli.wait_for_service()
        self.get_logger().info('MAVROS services are available.')

    # ---------------- Callbacks ----------------
    def _state_cb(self, msg: State): self.state1 = msg
    def _alt1_cb(self, msg: Float64): self.rel_alt1 = float(msg.data)
    def _alt2_cb(self, msg: Float64): self.rel_alt2 = float(msg.data)
    def _alt3_cb(self, msg: Float64): self.rel_alt3 = float(msg.data)
    def _pose1_cb(self, msg: PoseStamped): self.pose1 = msg

    # ---------------- Helpers ----------------
    def _set_mode(self, mode: str, timeout: float = 10.0) -> bool:
        req = SetMode.Request(); req.base_mode=0; req.custom_mode=mode
        fut = self.setmode_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        ok = bool(fut.done() and fut.result() and getattr(fut.result(), 'mode_sent', False))
        self.get_logger().info(f"SetMode('{mode}') -> {ok}")
        return ok

    def _arm_once(self, timeout: float = 10.0) -> bool:
        req = CommandBool.Request(); req.value=True
        fut = self.arm_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        ok = bool(fut.done() and fut.result() and getattr(fut.result(), 'success', False))
        self.get_logger().info(f"Arm attempt -> {ok}")
        return ok

    def _takeoff(self, alt: float = 10.0, timeout: float = 10.0) -> bool:
        req = CommandTOL.Request()
        req.altitude=float(alt); req.latitude=0.0; req.longitude=0.0; req.min_pitch=0.0; req.yaw=float('nan')
        fut = self.tko_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        ok = bool(fut.done() and fut.result() and getattr(fut.result(), 'success', False))
        self.get_logger().info(f"Takeoff({alt} m) -> {ok}")
        return ok

    @staticmethod
    def _ang_norm(a: float) -> float:
        return (a + math.pi) % (2.0*math.pi) - math.pi

    def _pose_xyz_yaw(self, p: PoseStamped) -> Tuple[float, float, float, float]:
        q = p.pose.orientation
        siny_cosp = 2.0 * (q.w*q.z + q.x*q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return p.pose.position.x, p.pose.position.y, p.pose.position.z, yaw

    def _publish_position_target(self, x: float, y: float, z: float, yaw: float) -> None:
        sp = PositionTarget()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        sp.type_mask = (
            PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        sp.position.x = x; sp.position.y = y; sp.position.z = z
        sp.yaw = yaw
        self.raw_sp_pub.publish(sp)

    # ---------------- Mission ----------------
    def run(self) -> None:
        # 1) FCU
        while rclpy.ok() and not self.state1.connected:
            self.get_logger().info('Waiting for FCU connection...')
            rclpy.spin_once(self, timeout_sec=0.2)

        # 2) GUIDED/OFFBOARD
        self._set_mode(self.guided_mode)

        # 3) Arm
        while rclpy.ok() and not self.state1.armed:
            if not self._arm_once():
                self.get_logger().warn('Arming failed, retrying...')
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(self.arm_retry_sec)

        # 4) Takeoff
        if not self._takeoff(self.target_alt):
            self.get_logger().error('Takeoff failed; aborting.')
            return

        # 5) Airborne check
        while rclpy.ok() and self.rel_alt1 < (self.target_alt - self.alt_air_thresh):
            rclpy.spin_once(self, timeout_sec=0.2)
        self.get_logger().info(f'Leader airborne: rel_alt={self.rel_alt1:.1f} m')

        # 6) Compute initial goal relative to current pose
        # rclpy.spin_once(self, timeout_sec=0.2)
        # x0, y0, z0, _ = self._pose_xyz_yaw(self.pose1)
        # gx = x0 + self.rel_dx
        # gy = y0 + self.rel_dy
        # gz = (z0 + self.rel_dz) if abs(self.rel_dz) > 1e-3 else self.target_alt
        # gyaw = self.target_yaw
        # self.get_logger().info(f'Initial goal -> ({gx:.2f},{gy:.2f},{gz:.2f}), yaw={math.degrees(gyaw):.1f}°')

        # 7) Stream setpoints until within tolerance
        dt = 1.0 / max(1e-3, self.sp_rate_hz)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            # cx, cy, cz, cyaw = self._pose_xyz_yaw(self.pose1)
            # pos_ok = (abs(cx-gx) <= self.pos_tol and abs(cy-gy) <= self.pos_tol and abs(cz-gz) <= self.pos_tol)
            # yaw_ok = (abs(self._ang_norm(gyaw - cyaw)) <= self.yaw_tol)
            # if pos_ok and yaw_ok:
            #     break
            self._publish_position_target(0, 0, 0, 0)
            time.sleep(dt)
        self.get_logger().info('Reached initial relative goal.')

        # 8) Wait follower airborne (uav2) while holding
        while rclpy.ok() and self.rel_alt2 < (self.target_alt - self.alt_air_thresh):
            rclpy.spin_once(self, timeout_sec=0.0)
            self._publish_position_target(0, 0, 0, 0)
            time.sleep(dt)
        self.get_logger().info('Follower airborne, starting test sequence.')

        # # ---------------- Test: rotate + climb + forward + descend + rotate ----------------
        # # Current pose snapshot
        # px, py, pz, pyaw = self._pose_xyz_yaw(self.pose1)

        # # a) Rotate +90° at current spot
        # yaw1 = self._ang_norm(pyaw + math.pi/2.0)
        # self.get_logger().info(f'Rotate to yaw1 = {math.degrees(yaw1):.1f}°')
        # while rclpy.ok() and abs(self._ang_norm(self._pose_xyz_yaw(self.pose1)[3] - yaw1)) > self.yaw_tol:
        #     rclpy.spin_once(self, timeout_sec=0.0)
        #     self._publish_position_target(px, py, pz, yaw1)
        #     time.sleep(dt)

        # # b) Climb 2 m (hold yaw1)
        # climb_alt = pz + 2.0
        # self.get_logger().info(f'Climb to {climb_alt:.2f} m')
        # while rclpy.ok():
        #     rclpy.spin_once(self, timeout_sec=0.0)
        #     cx, cy, cz, cyaw = self._pose_xyz_yaw(self.pose1)
        #     if abs(cz - climb_alt) <= self.pos_tol:
        #         break
        #     self._publish_position_target(px, py, climb_alt, yaw1)
        #     time.sleep(dt)

        # # c) Move 5 m forward along yaw1 (hold yaw1)
        # nx = px + math.cos(yaw1) * 5.0
        # ny = py + math.sin(yaw1) * 5.0
        # self.get_logger().info(f'Move forward to ({nx:.2f},{ny:.2f},{climb_alt:.2f})')
        # while rclpy.ok():
        #     rclpy.spin_once(self, timeout_sec=0.0)
        #     cx, cy, cz, cyaw = self._pose_xyz_yaw(self.pose1)
        #     if (abs(cx-nx)<=self.pos_tol and abs(cy-ny)<=self.pos_tol and abs(cz-climb_alt)<=self.pos_tol):
        #         break
        #     self._publish_position_target(nx, ny, climb_alt, yaw1)
        #     time.sleep(dt)

        # # d) Descend back to original altitude gz (hold yaw1)
        # self.get_logger().info(f'Descend to {gz:.2f} m')
        # while rclpy.ok():
        #     rclpy.spin_once(self, timeout_sec=0.0)
        #     cx, cy, cz, cyaw = self._pose_xyz_yaw(self.pose1)
        #     if abs(cz - gz) <= self.pos_tol:
        #         break
        #     self._publish_position_target(nx, ny, gz, yaw1)
        #     time.sleep(dt)

        # # e) Rotate +90° again at the new spot
        # yaw2 = self._ang_norm(yaw1 + math.pi/2.0)
        # self.get_logger().info(f'Rotate to yaw2 = {math.degrees(yaw2):.1f}°')
        # while rclpy.ok() and abs(self._ang_norm(self._pose_xyz_yaw(self.pose1)[3] - yaw2)) > self.yaw_tol:
        #     rclpy.spin_once(self, timeout_sec=0.0)
        #     self._publish_position_target(nx, ny, gz, yaw2)
        #     time.sleep(dt)

        # 9) Switch to POSHOLD/POSCTL
        self._set_mode(self.poshold_mode)
        self.get_logger().info('Test complete; switched to position hold.')

def main() -> None:
    rclpy.init()
    node = ShadowLeader()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
