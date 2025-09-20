#!/usr/bin/env python3
import time, math
from typing import Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode


class SimpleLeader(Node):
    def __init__(self) -> None:
        super().__init__('shadow_leader_simple')

        qos_sensor = QoSProfile(depth=10)
        qos_sensor.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_sensor.durability  = QoSDurabilityPolicy.VOLATILE

        # Params
        self.uav1_ns = self.declare_parameter('uav1_ns', '/uav1').get_parameter_value().string_value
        self.uav2_ns = self.declare_parameter('uav2_ns', '/uav2').get_parameter_value().string_value
        self.uav3_ns = self.declare_parameter('uav3_ns', '/uav3').get_parameter_value().string_value

        self.guided_mode  = self.declare_parameter('guided_mode', 'GUIDED').get_parameter_value().string_value
        self.poshold_mode = self.declare_parameter('poshold_mode', 'POSHOLD').get_parameter_value().string_value
        self.target_alt   = float(self.declare_parameter('target_alt', 3.0).get_parameter_value().double_value)
        self.alt_thresh   = float(self.declare_parameter('alt_air_thresh', 0.5).get_parameter_value().double_value)
        self.sp_rate_hz   = float(self.declare_parameter('sp_rate_hz', 12.0).get_parameter_value().double_value)

        # trial manoeuvre params / tolerances
        self.pos_tol = float(self.declare_parameter('pos_tol', 0.4).get_parameter_value().double_value)
        self.yaw_tol = float(self.declare_parameter('yaw_tol_deg', 8.0).get_parameter_value().double_value) * math.pi/180.0
        self.forward_m = float(self.declare_parameter('trial_forward_m', 2.0).get_parameter_value().double_value)
        self.climb_m   = float(self.declare_parameter('trial_climb_m',   2.0).get_parameter_value().double_value)
        self.yaw_step  = float(self.declare_parameter('trial_yaw_deg',  90.0).get_parameter_value().double_value) * math.pi/180.0

        # State
        self.state1 = State()
        self.rel_alt1 = 0.0
        self.rel_alt2 = 0.0
        self.rel_alt3 = 0.0
        self.pose1 = PoseStamped()

        # I/O
        self.create_subscription(State,      f'{self.uav1_ns}/state',                    self._state_cb, qos_sensor)
        self.create_subscription(Float64,    f'{self.uav1_ns}/global_position/rel_alt',  self._alt1_cb,  qos_sensor)
        self.create_subscription(Float64,    f'{self.uav2_ns}/global_position/rel_alt',  self._alt2_cb,  qos_sensor)
        self.create_subscription(Float64,    f'{self.uav3_ns}/global_position/rel_alt',  self._alt3_cb,  qos_sensor)
        self.create_subscription(PoseStamped,f'{self.uav1_ns}/local_position/pose',      self._pose1_cb, qos_sensor)

        self.sp_pub = self.create_publisher(PositionTarget, f'{self.uav1_ns}/setpoint_raw/local', 10)

        self.arm_cli     = self.create_client(CommandBool, f'{self.uav1_ns}/cmd/arming')
        self.tko_cli     = self.create_client(CommandTOL,  f'{self.uav1_ns}/cmd/takeoff')
        self.setmode_cli = self.create_client(SetMode,     f'{self.uav1_ns}/set_mode')

        self.get_logger().info('Waiting for MAVROS services...')
        for cli in (self.arm_cli, self.tko_cli, self.setmode_cli):
            cli.wait_for_service()
        self.get_logger().info('MAVROS services are available.')

    # Callbacks
    def _state_cb(self, msg: State): self.state1 = msg
    def _alt1_cb(self, msg: Float64): self.rel_alt1 = float(msg.data)
    def _alt2_cb(self, msg: Float64): self.rel_alt2 = float(msg.data)
    def _alt3_cb(self, msg: Float64): self.rel_alt3 = float(msg.data)
    def _pose1_cb(self, msg: PoseStamped): self.pose1 = msg

    # Helpers
    def _set_mode(self, mode: str, timeout: float = 10.0) -> bool:
        req = SetMode.Request(); req.base_mode = 0; req.custom_mode = mode
        fut = self.setmode_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        ok = bool(fut.done() and fut.result() and getattr(fut.result(), 'mode_sent', False))
        self.get_logger().info(f"SetMode('{mode}') -> {ok}")
        return ok

    def _arm_once(self, timeout: float = 10.0) -> bool:
        req = CommandBool.Request(); req.value = True
        fut = self.arm_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        ok = bool(fut.done() and fut.result() and getattr(fut.result(), 'success', False))
        self.get_logger().info(f"Arm() -> {ok}")
        return ok

    def _takeoff(self, alt: float, timeout: float = 10.0) -> bool:
        req = CommandTOL.Request()
        req.altitude = float(alt); req.latitude = 0.0; req.longitude = 0.0
        req.min_pitch = 0.0; req.yaw = float('nan')
        fut = self.tko_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        ok = bool(fut.done() and fut.result() and getattr(fut.result(), 'success', False))
        self.get_logger().info(f"Takeoff({alt}) -> {ok}")
        return ok

    @staticmethod
    def _ang_norm(a: float) -> float:
        return (a + math.pi) % (2.0*math.pi) - math.pi

    @staticmethod
    def _pose_xyz_yaw(p: PoseStamped) -> Tuple[float, float, float, float]:
        q = p.pose.orientation
        siny_cosp = 2.0 * (q.w*q.z + q.x*q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return float(p.pose.position.x), float(p.pose.position.y), float(p.pose.position.z), float(yaw)

    def _publish_xyz_yaw(self, x: float, y: float, z: float, yaw: float):
        sp = PositionTarget()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        sp.type_mask = (
            PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        sp.position.x = float(x); sp.position.y = float(y); sp.position.z = float(z)
        sp.yaw = float(yaw)
        self.sp_pub.publish(sp)

    def _drive_to(self, gx: float, gy: float, gz: float, gyaw: float):
        dt = 1.0 / max(1e-3, self.sp_rate_hz)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            cx, cy, cz, cyaw = self._pose_xyz_yaw(self.pose1)
            pos_ok = (abs(cx-gx) <= self.pos_tol and abs(cy-gy) <= self.pos_tol and abs(cz-gz) <= self.pos_tol)
            yaw_ok = (abs(self._ang_norm(gyaw - cyaw)) <= self.yaw_tol)
            self._publish_xyz_yaw(gx, gy, gz, gyaw)
            if pos_ok and yaw_ok:
                break
            time.sleep(dt)

    def _hold_here(self):
        x, y, z, yaw = self._pose_xyz_yaw(self.pose1)
        self._publish_xyz_yaw(x, y, z, yaw)

    # Mission
    def run(self) -> None:
        # Wait FCU
        while rclpy.ok() and not self.state1.connected:
            self.get_logger().info('Leader waiting for FCU...')
            rclpy.spin_once(self, timeout_sec=0.2)

        # GUIDED/OFFBOARD
        self._set_mode(self.guided_mode)

        # Arm
        while rclpy.ok() and not self.state1.armed:
            self._arm_once()
            rclpy.spin_once(self, timeout_sec=0.2)

        # Takeoff
        if not self._takeoff(self.target_alt):
            self.get_logger().error('Leader takeoff failed.')
            return

        # Wait until airborne
        while rclpy.ok() and self.rel_alt1 < (self.target_alt - self.alt_thresh):
            rclpy.spin_once(self, timeout_sec=0.2)
        self.get_logger().info(f'Leader airborne: rel_alt={self.rel_alt1:.1f} m')

        # Hold while waiting for any follower to be airborne
        dt = 1.0 / max(1e-3, self.sp_rate_hz)
        self.get_logger().info('Holding position until a follower is airborne...')
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            self._hold_here()
            if (self.rel_alt2 >= (self.target_alt - self.alt_thresh)) and (self.rel_alt3 >= (self.target_alt - self.alt_thresh)):
                break
            time.sleep(dt)

        # ---------- Trial manoeuvres ----------
        # px, py, pz, pyaw = self._pose_xyz_yaw(self.pose1)

        # # a) go forward 2 m along current yaw
        # fx = px + math.cos(pyaw) * self.forward_m
        # fy = py + math.sin(pyaw) * self.forward_m
        # self.get_logger().info(f'Trial: forward {self.forward_m:.1f} m')
        # self._drive_to(fx, fy, pz, pyaw)

        # # b) climb +2 m (hold XY, yaw)
        # cz = pz + self.climb_m
        # self.get_logger().info(f'Trial: climb +{self.climb_m:.1f} m')
        # self._drive_to(fx, fy, cz, pyaw)

        # # c) rotate +90° at the spot, step-wise
        # total_yaw = self._ang_norm(pyaw + self.yaw_step)  # final yaw target
        # step_deg  = 5.0                                  # yaw increment in degrees
        # step_rad  = math.radians(step_deg)

        # # choose direction of rotation (shortest path)
        # diff = self._ang_norm(total_yaw - pyaw)
        # direction = 1.0 if diff > 0 else -1.0
        # n_steps = int(abs(diff) / step_rad)

        # self.get_logger().info(f'Trial: rotate {math.degrees(diff):.1f}° in {n_steps} steps of {step_deg}°')

        # cyaw = pyaw
        # for i in range(n_steps):
        #     cyaw = self._ang_norm(cyaw + direction * step_rad)
        #     self._drive_to(fx, fy, cz, cyaw)
        #     self.get_logger().info(f'  → step {i+1}/{n_steps}, yaw={math.degrees(cyaw):.1f}°')

        # # final fine alignment to total_yaw
        # self._drive_to(fx, fy, cz, total_yaw)
        # self.get_logger().info(f'Final yaw = {math.degrees(total_yaw):.1f}°')


        # Switch to position hold
        self._set_mode("AUTO")
        # self._set_mode(self.poshold_mode)
        self.get_logger().info('Trial done → switched to position hold.')


def main():
    rclpy.init()
    node = SimpleLeader()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
