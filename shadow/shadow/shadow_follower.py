#!/usr/bin/env python3
"""
Shadow Follower (v1)

Behavior (first parts)
- Arms and takes off to target altitude (GUIDED/APM or OFFBOARD/PX4 — configurable).
- Tracks the LEADER's pose and yaw.
- Goes to a relative slot that is 5 m LEFT or RIGHT of the leader (w.r.t. leader heading).
- Keeps adjusting the slot while the leader moves; when the leader switches to POSHOLD, the
  follower keeps formation at that slot.

Assumptions
- ENU frame on /<ns>/local_position/pose
- Yaw extracted from quaternion (same as leader node).
- Uses PositionTarget on /<ns>/setpoint_raw/local to command position + yaw.

Params
- follower_ns (string): default '/uav2'
- leader_ns (string): default '/uav1'
- side (string): 'left' or 'right' (default 'left')
- slot_offset_m (double): distance from leader, default 5.0
- target_alt (double): default 10.0
- guided_mode (string): 'GUIDED' (APM) or 'OFFBOARD' (PX4)
- poshold_mode (string): 'POSHOLD' (APM) or 'POSCTL' (PX4)
- sp_rate_hz (double): setpoint stream rate, default 10.0

QoS: matches MAVROS sensor topics (BEST_EFFORT / VOLATILE)
"""

import math
from typing import Tuple
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode


class ShadowFollower(Node):
    def __init__(self) -> None:
        super().__init__('shadow_follower')

        # --- QoS ---
        qos_sensor = QoSProfile(depth=10)
        qos_sensor.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_sensor.durability  = QoSDurabilityPolicy.VOLATILE

        # --- Params ---
        self.follower_ns = self.declare_parameter('follower_ns', '/uav2').get_parameter_value().string_value
        self.leader_ns   = self.declare_parameter('leader_ns',   '/uav1').get_parameter_value().string_value

        side = self.declare_parameter('side', 'left').get_parameter_value().string_value.lower()
        self.side = 'left' if side not in ('left','right') else side
        self.slot_offset_m = float(self.declare_parameter('slot_offset_m', 5.0).get_parameter_value().double_value)

        self.target_alt = float(self.declare_parameter('target_alt', 10.0).get_parameter_value().double_value)
        self.guided_mode  = self.declare_parameter('guided_mode', 'GUIDED').get_parameter_value().string_value
        self.poshold_mode = self.declare_parameter('poshold_mode', 'POSHOLD').get_parameter_value().string_value
        self.sp_rate_hz   = float(self.declare_parameter('sp_rate_hz', 10.0).get_parameter_value().double_value)
        self.arm_retry_sec = float(self.declare_parameter('arm_retry_sec', 2.0).get_parameter_value().double_value)

        # Tolerances
        self.alt_air_thresh = float(self.declare_parameter('alt_air_thresh', 0.5).get_parameter_value().double_value)  # m
        self.pos_tol = float(self.declare_parameter('pos_tol', 0.8).get_parameter_value().double_value)  # m
        self.yaw_tol = float(self.declare_parameter('yaw_tol', math.radians(8.0)).get_parameter_value().double_value)  # rad

        # --- State ---
        self.my_state = State()
        self.my_rel_alt = 0.0
        self.my_pose = PoseStamped()
        self.leader_pose = PoseStamped()
        self.leader_state = State()

        # --- Subs ---
        self.create_subscription(State, f'{self.follower_ns}/state', self._state_cb, qos_sensor)
        self.create_subscription(Float64, f'{self.follower_ns}/global_position/rel_alt', self._alt_cb, qos_sensor)
        self.create_subscription(PoseStamped, f'{self.follower_ns}/local_position/pose', self._pose_cb, qos_sensor)

        self.create_subscription(PoseStamped, f'{self.leader_ns}/local_position/pose', self._leader_pose_cb, qos_sensor)
        self.create_subscription(State, f'{self.leader_ns}/state', self._leader_state_cb, qos_sensor)

        # --- Pub ---
        self.sp_pub = self.create_publisher(PositionTarget, f'{self.follower_ns}/setpoint_raw/local', 10)

        # --- Srvs ---
        self.arm_cli     = self.create_client(CommandBool, f'{self.follower_ns}/cmd/arming')
        self.tko_cli     = self.create_client(CommandTOL,  f'{self.follower_ns}/cmd/takeoff')
        self.setmode_cli = self.create_client(SetMode,     f'{self.follower_ns}/set_mode')

        self.get_logger().info('Waiting for follower MAVROS services...')
        for cli in (self.arm_cli, self.tko_cli, self.setmode_cli):
            cli.wait_for_service()
        self.get_logger().info('Follower MAVROS services are available.')

    # --- Callbacks ---
    def _state_cb(self, msg: State):
        self.my_state = msg

    def _alt_cb(self, msg: Float64):
        self.my_rel_alt = float(msg.data)

    def _pose_cb(self, msg: PoseStamped):
        self.my_pose = msg

    def _leader_pose_cb(self, msg: PoseStamped):
        self.leader_pose = msg

    def _leader_state_cb(self, msg: State):
        self.leader_state = msg

    # --- Helpers ---
    def _set_mode(self, mode: str, timeout: float = 10.0) -> bool:
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = mode
        fut = self.setmode_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return bool(fut.done() and fut.result() and getattr(fut.result(), 'mode_sent', False))

    def _arm_once(self, timeout: float = 10.0) -> bool:
        req = CommandBool.Request()
        req.value = True
        fut = self.arm_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return bool(fut.done() and fut.result() and getattr(fut.result(), 'success', False))

    def _takeoff(self, alt: float = 10.0, timeout: float = 10.0) -> bool:
        req = CommandTOL.Request()
        req.altitude = float(alt)
        req.latitude = 0.0
        req.longitude = 0.0
        req.min_pitch = 0.0
        req.yaw = float('nan')
        fut = self.tko_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return bool(fut.done() and fut.result() and getattr(fut.result(), 'success', False))

    @staticmethod
    def _pose_xyz_yaw(p: PoseStamped) -> Tuple[float, float, float, float]:
        q = p.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return p.pose.position.x, p.pose.position.y, p.pose.position.z, yaw

    def _slot_offset_vector(self, yaw: float, d: float) -> Tuple[float, float]:
        # Heading unit: h = (cos(yaw), sin(yaw)) in ENU
        # Left  unit    l = (-sin(yaw), cos(yaw))
        # Right unit    r = ( sin(yaw),-cos(yaw))
        if self.side == 'left':
            return (-math.sin(yaw) * d, math.cos(yaw) * d)
        else:  # right
            return ( math.sin(yaw) * d,-math.cos(yaw) * d)

    def _publish_position_target(self, x: float, y: float, z: float, yaw: float) -> None:
        sp = PositionTarget()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        sp.type_mask = (
            PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        sp.position.x = x
        sp.position.y = y
        sp.position.z = z
        sp.yaw = yaw
        self.sp_pub.publish(sp)

    # --- Mission ---
    def run(self) -> None:
        # Wait FCU connection
        while rclpy.ok() and not self.my_state.connected:
            self.get_logger().info('Follower waiting for FCU connection...')
            rclpy.spin_once(self, timeout_sec=0.2)

        # Enter GUIDED/OFFBOARD
        self._set_mode(self.guided_mode)

        # Arm loop
        while rclpy.ok() and not self.my_state.armed:
            if not self._arm_once():
                self.get_logger().warn('Follower arming failed, retrying...')
            rclpy.spin_once(self, timeout_sec=0.1)

        # Takeoff
        if not self._takeoff(self.target_alt):
            self.get_logger().error('Follower takeoff failed; aborting.')
            return

        # Wait until comfortably airborne
        while rclpy.ok() and self.my_rel_alt < ( self.target_alt - self.alt_air_thresh):
            rclpy.spin_once(self, timeout_sec=0.2)
            # time.sleep(0.2)
        self.get_logger().info(f'Follower airborne: rel_alt={self.my_rel_alt:.1f} m')

        # Formation tracking loop
        dt = 1.0 / max(1e-3, self.sp_rate_hz)
        self.get_logger().info(f'Holding {self.side.upper()} slot at {self.slot_offset_m:.1f} m from leader...')
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)

            # Leader pose/yaw
            lx, ly, lz, lyaw = self._pose_xyz_yaw(self.leader_pose)

            # Desired slot relative to leader heading
            ox, oy = self._slot_offset_vector(lyaw, self.slot_offset_m)
            gx, gy, gz = lx + ox, ly + oy, self.target_alt

            # Face same direction as leader (optional; could also face outward)
            gyaw = lyaw

            # Stream raw local setpoints
            self._publish_position_target(gx, gy, gz, gyaw)

            # If leader is in POSHOLD/POSCTL, we still keep streaming to hold the slot.
            # (Future: add logic to switch to POSCTL locally if needed.)

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
