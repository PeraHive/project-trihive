#!/usr/bin/env python3
"""
Shadow Leader (no EXTENDED_SYS_STATE)

Behavior
- Arms and takes off UAV1 to target altitude in GUIDED (or OFFBOARD for PX4, configurable).
- Flies UAV1 to a RELATIVE pose: 10 m to the North (default) from its current local pose
  and yaws to face North.
- Waits until UAV2 is airborne (simple rel_alt check while holding the goal).
- Then performs: rotate in place (+90°) -> move 5 m forward -> rotate in place (+90°).
- Switches UAV1 to POSHOLD (APM) / POSCTL (PX4) (configurable).

Assumptions
- MAVROS namespaces per UAV: /uav1, /uav2, /uav3.
- Local frame is ENU: x=East, y=North, z=Up.
- ArduPilot tends to ignore yaw in PoseStamped; we publish PositionTarget with yaw.

Tested against: ROS 2 (Jazzy), mavros_msgs v2.* APIs.
"""

import math
import time
from typing import Tuple

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy


class ShadowLeader(Node):
    def __init__(self) -> None:
        super().__init__('shadow_leader')
        # QoS to match MAVROS sensor publishers (BEST_EFFORT / VOLATILE)
        qos_sensor = QoSProfile(depth=10)
        qos_sensor.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_sensor.durability  = QoSDurabilityPolicy.VOLATILE

        # ---------------- Parameters ----------------
        # Namespaces
        self.uav1_ns = self.declare_parameter('uav1_ns', '/uav1').get_parameter_value().string_value
        self.uav2_ns = self.declare_parameter('uav2_ns', '/uav2').get_parameter_value().string_value
        self.uav3_ns = self.declare_parameter('uav3_ns', '/uav3').get_parameter_value().string_value

        # Modes (ArduPilot: GUIDED/POSHOLD, PX4: OFFBOARD/POSCTL)
        self.guided_mode  = self.declare_parameter('guided_mode', 'GUIDED').get_parameter_value().string_value
        self.poshold_mode = self.declare_parameter('poshold_mode', 'POSHOLD').get_parameter_value().string_value

        # Flight targets
        self.target_alt = float(self.declare_parameter('target_alt', 10.0).get_parameter_value().double_value)
        self.arm_retry_sec = float(self.declare_parameter('arm_retry_sec', 2.0).get_parameter_value().double_value)

        # Leader relative move (from current local pose) — ENU
        # 10 m North (y +10), maintain z, yaw to face North
        self.rel_dx = float(self.declare_parameter('rel_dx', 0.0).get_parameter_value().double_value)   # East +
        self.rel_dy = float(self.declare_parameter('rel_dy', 10.0).get_parameter_value().double_value)  # North +
        self.rel_dz = float(self.declare_parameter('rel_dz', 0.0).get_parameter_value().double_value)   # Up +

        # Heading target (radians). Facing North in ENU is +90° = +pi/2.
        self.target_yaw = float(self.declare_parameter('target_yaw', math.pi/2).get_parameter_value().double_value)

        # Tolerances
        self.alt_air_thresh = float(self.declare_parameter('alt_air_thresh', 0.5).get_parameter_value().double_value)  # m
        self.pos_tol = float(self.declare_parameter('pos_tol', 0.8).get_parameter_value().double_value)  # m
        self.yaw_tol = float(self.declare_parameter('yaw_tol', math.radians(8.0)).get_parameter_value().double_value)  # rad

        # Streams
        self.sp_rate_hz = float(self.declare_parameter('sp_rate_hz', 10.0).get_parameter_value().double_value)

        # Speed limits

        self.max_lin_vel  = float(self.declare_parameter('max_lin_vel', 2).get_parameter_value().double_value)      # m/s
        self.max_yaw_rate = float(self.declare_parameter('max_yaw_rate', math.radians(90.0)).get_parameter_value().double_value)  # rad/s

        # ---------------- Internal State ----------------
        self.state1 = State()
        self.rel_alt1 = 0.0
        self.rel_alt2 = 0.0
        self.rel_alt3 = 0.0

        self.pose1 = PoseStamped()

        self.goal_pose = None  # type: Tuple[float, float, float]
        self.goal_yaw = None   # type: float

        # ---------------- Subscriptions ----------------
        self.create_subscription(State, f'{self.uav1_ns}/state', self._state_cb, qos_sensor)
        self.create_subscription(Float64, f'{self.uav1_ns}/global_position/rel_alt', self._alt1_cb, qos_sensor)
        self.create_subscription(Float64, f'{self.uav2_ns}/global_position/rel_alt', self._alt2_cb, qos_sensor)
        self.create_subscription(Float64, f'{self.uav3_ns}/global_position/rel_alt', self._alt3_cb, qos_sensor)

        self.create_subscription(PoseStamped, f'{self.uav1_ns}/local_position/pose', self._pose1_cb, qos_sensor)

        # ---------------- Publishers ----------------
        # Use PositionTarget so we can command position + yaw together
        self.raw_sp_pub = self.create_publisher(PositionTarget, f'{self.uav1_ns}/setpoint_raw/local', qos_sensor)

        # ---------------- Services ----------------
        self.arm_cli     = self.create_client(CommandBool, f'{self.uav1_ns}/cmd/arming')
        self.tko_cli     = self.create_client(CommandTOL,  f'{self.uav1_ns}/cmd/takeoff')
        self.setmode_cli = self.create_client(SetMode,     f'{self.uav1_ns}/set_mode')

        self.get_logger().info('Waiting for MAVROS services...')
        for cli in (self.arm_cli, self.tko_cli, self.setmode_cli):
            cli.wait_for_service()
        self.get_logger().info('MAVROS services are available.')

    # ---------------- Callbacks ----------------
    def _state_cb(self, msg: State):
        self.state1 = msg

    def _alt1_cb(self, msg: Float64):
        self.rel_alt1 = float(msg.data)

    def _alt2_cb(self, msg: Float64):
        self.rel_alt2 = float(msg.data)

    def _alt3_cb(self, msg: Float64):
        self.rel_alt3 = float(msg.data)

    def _pose1_cb(self, msg: PoseStamped):
        self.pose1 = msg

    # ---------------- Helpers -----------------
    def _set_mode(self, mode: str, timeout: float = 10.0) -> bool:
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = mode
        fut = self.setmode_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        ok = bool(fut.done() and fut.result() and getattr(fut.result(), 'mode_sent', False))
        self.get_logger().info(f"SetMode('{mode}') -> {ok}")
        return ok

    def _arm_once(self, timeout: float = 10.0) -> bool:
        req = CommandBool.Request()
        req.value = True
        fut = self.arm_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        ok = bool(fut.done() and fut.result() and getattr(fut.result(), 'success', False))
        self.get_logger().info(f"Arm attempt -> {ok}")
        return ok

    def _takeoff(self, alt: float = 10.0, timeout: float = 10.0) -> bool:
        req = CommandTOL.Request()
        req.altitude = float(alt)
        req.latitude = 0.0
        req.longitude = 0.0
        req.min_pitch = 0.0
        req.yaw = float('nan')  # let FCU decide initially
        fut = self.tko_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        ok = bool(fut.done() and fut.result() and getattr(fut.result(), 'success', False))
        self.get_logger().info(f"Takeoff({alt} m) -> {ok}")
        return ok

    def _publish_position_target(self, x: float, y: float, z: float, yaw: float) -> None:
        """Publish a PositionTarget with position (x,y,z) and yaw in ENU."""
        sp = PositionTarget()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED  # MAVROS converts; ENU->NED handled internally
        # Use only POSITION + YAW (ignore velocities/accels)
        sp.type_mask = (
            PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        sp.position.x = x
        sp.position.y = y
        sp.position.z = z
        sp.yaw = yaw
        self.raw_sp_pub.publish(sp)

    @staticmethod
    def _ang_norm(a: float) -> float:
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    def _pose_xyz_yaw(self, p: PoseStamped) -> Tuple[float, float, float, float]:
        # Extract yaw from quaternion (ENU)
        q = p.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return p.pose.position.x, p.pose.position.y, p.pose.position.z, yaw

    def _yaw_slew(self, target_yaw: float, hold_x: float, hold_y: float, hold_z: float, dt: float) -> None:
        """Hold current position while rotating to target_yaw."""
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            _, _, _, cyaw = self._pose_xyz_yaw(self.pose1)
            if abs(self._ang_norm(target_yaw - cyaw)) <= self.yaw_tol:
                break
            self._publish_position_target(hold_x, hold_y, hold_z, target_yaw)
            time.sleep(dt)

    # --------- NEW: unified, rate-limited mover ----------
    def _goto_with_limits(self, tx: float, ty: float, tz: float, tyaw: float) -> None:
        """
        Stream intermediate setpoints so motion never exceeds:
          - self.max_lin_vel (m/s) for linear motion
          - self.max_yaw_rate (rad/s) for yaw
        Exits when within pos/yaw tolerances.
        """
        dt = 1.0 / max(1e-3, self.sp_rate_hz)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)

            cx, cy, cz, cyaw = self._pose_xyz_yaw(self.pose1)

            # position step
            dx, dy, dz = (tx - cx), (ty - cy), (tz - cz)
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            done_pos = dist <= self.pos_tol

            if not done_pos:
                max_step = self.max_lin_vel * dt
                if dist > max_step:
                    scale = max_step / dist
                    nx = cx + dx * scale
                    ny = cy + dy * scale
                    nz = cz + dz * scale
                else:
                    nx, ny, nz = tx, ty, tz
            else:
                nx, ny, nz = tx, ty, tz

            # yaw step
            dyaw = self._ang_norm(tyaw - cyaw)
            done_yaw = abs(dyaw) <= self.yaw_tol
            if not done_yaw:
                max_yaw_step = self.max_yaw_rate * dt
                step_yaw = max(-max_yaw_step, min(max_yaw_step, dyaw))
                nyaw = self._ang_norm(cyaw + step_yaw)
            else:
                nyaw = tyaw

            self._publish_position_target(nx, ny, nz, nyaw)

            if done_pos and done_yaw:
                break

            time.sleep(dt)

    # ---------------- Mission -----------------
    def run(self) -> None:
        # 1) Wait FCU connection
        while rclpy.ok() and not self.state1.connected:
            self.get_logger().info('Waiting for FCU connection...')
            rclpy.spin_once(self, timeout_sec=0.2)
            time.sleep(0.2)

        # 2) Enter GUIDED / OFFBOARD
        self._set_mode(self.guided_mode)
        time.sleep(1.0)

        # 3) Arm loop
        while rclpy.ok() and not self.state1.armed:
            if not self._arm_once():
                self.get_logger().warn('Arming failed, retrying...')
            time.sleep(self.arm_retry_sec)
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('Leader is ARMED.')

        # 4) Takeoff
        if not self._takeoff(self.target_alt):
            self.get_logger().error('Takeoff command failed; aborting.')
            return

        # 5) Confirm airborne using relative altitude
        self.get_logger().info('Waiting for Leader to be airborne via rel_alt...')
        while rclpy.ok() and self.rel_alt1 < self.target_alt - self.alt_air_thresh:
            rclpy.spin_once(self, timeout_sec=0.2)
        self.get_logger().info(f'Leader airborne: rel_alt={self.rel_alt1:.1f} m')

        # 6) Compute leader goal (relative from current pose)
        rclpy.spin_once(self, timeout_sec=0.2)
        x0, y0, z0, _ = self._pose_xyz_yaw(self.pose1)
        gx = x0 + self.rel_dx
        gy = y0 + self.rel_dy
        gz = z0 + self.rel_dz if abs(self.rel_dz) > 1e-3 else self.target_alt
        self.goal_pose = (gx, gy, gz)
        self.goal_yaw = self.target_yaw
        self.get_logger().info(f'Leader goal -> x:{gx:.2f} y:{gy:.2f} z:{gz:.2f} yaw:{math.degrees(self.goal_yaw):.1f}°')

        # 7) Stream position+yaw setpoints until within tolerance
        dt = 1.0 / max(1e-3, self.sp_rate_hz)
        self.get_logger().info('Flying to relative goal...')
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            cx, cy, cz, cyaw = self._pose_xyz_yaw(self.pose1)
            yaw_err = abs(self._ang_norm(self.goal_yaw - cyaw))
            if (abs(cx - gx) <= self.pos_tol and
                abs(cy - gy) <= self.pos_tol and
                abs(cz - gz) <= self.pos_tol and
                yaw_err <= self.yaw_tol):
                break
            self._publish_position_target(gx, gy, gz, self.goal_yaw)
            time.sleep(dt)
        self.get_logger().info('Leader reached goal pose (within tolerance).')

        # 7) Rate-limited flight to relative goal
        # self.get_logger().info('Flying to relative goal (rate-limited)...')
        # self._goto_with_limits(gx, gy, gz, self.goal_yaw)
        # self.get_logger().info('Leader reached goal pose (within tolerance).')


        # 8) Wait for follower(s): simple airborne check for uav2 while holding position
        self.get_logger().info('Waiting for followers (uav2) to be airborne...')
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            air2 = self.rel_alt2 > self.target_alt - self.alt_air_thresh
            if air2:
                break
            # Keep leader stable at goal while waiting
            self._publish_position_target(gx, gy, gz, self.goal_yaw)
            time.sleep(dt)
        self.get_logger().info('Follower(s) ready.')

          # _________________________________________________________________________________________ 
        # TEST CODE (now rate-limited)
        # _________________________________________________________________________________________ 

        # 9a) rotate +90° in place at (gx, gy, gz)
        # _, _, _, cyaw = self._pose_xyz_yaw(self.pose1)
        # yaw1 = self._ang_norm(cyaw + math.pi/2.0)
        # self.get_logger().info(f'Rotate to yaw1 = {math.degrees(yaw1):.1f}° (rate-limited)')
        # self._goto_with_limits(gx, gy, gz, yaw1)

        # 9b) move 5 m forward along yaw1 while holding yaw1
        # step = 5.0
        # nx = gx + math.cos(yaw1) * step
        # ny = gy + math.sin(yaw1) * step
        # self.get_logger().info(f'Move 5 m to ({nx:.2f}, {ny:.2f}, {gz:.2f}) (rate-limited)')
        # self._goto_with_limits(nx, ny, gz, yaw1)

        # 9c) rotate +90° again at new spot
        # yaw2 = self._ang_norm(yaw1 + math.pi/2.0)
        # self.get_logger().info(f'Rotate to yaw2 = {math.degrees(yaw2):.1f}° (rate-limited)')
        # self._goto_with_limits(nx, ny, gz, yaw2)
        # _________________________________________________________________________________________ 

        # 10) Switch to POSHOLD / POSCTL
        self._set_mode(self.poshold_mode)
        self.get_logger().info('Rotate–move–rotate complete; switched to position hold.')


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
