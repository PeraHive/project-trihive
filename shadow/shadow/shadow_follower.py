#!/usr/bin/env python3
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode


class PointShadowFollower(Node):
    def __init__(self):
        super().__init__('point_shadow_follower')

        qos_sensor = QoSProfile(depth=10)
        qos_sensor.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_sensor.durability  = QoSDurabilityPolicy.VOLATILE

        # Params
        self.follower_ns = self.declare_parameter('follower_ns', '/uav3').get_parameter_value().string_value
        self.leader_ns   = self.declare_parameter('leader_ns',   '/uav1').get_parameter_value().string_value
        self.target_alt  = float(self.declare_parameter('target_alt', 3.0).get_parameter_value().double_value)
        self.alt_thresh  = float(self.declare_parameter('alt_air_thresh', 0.5).get_parameter_value().double_value)
        self.sp_rate_hz  = float(self.declare_parameter('sp_rate_hz', 12.0).get_parameter_value().double_value)
        self.guided_mode = self.declare_parameter('guided_mode', 'GUIDED').get_parameter_value().string_value

        # State
        self.my_state = State()
        self.my_rel_alt = 0.0
        self.leader_pose = PoseStamped()
        self.leader_rel_alt = 0.0
        self.leader_heading = 0.0  # radians

        # Subscriptions
        self.create_subscription(State, f'{self.follower_ns}/state', self._state_cb, qos_sensor)
        self.create_subscription(Float64, f'{self.follower_ns}/global_position/rel_alt', self._alt_cb, qos_sensor)

        self.create_subscription(PoseStamped, f'{self.leader_ns}/local_position/pose', self._leader_pose_cb, qos_sensor)
        self.create_subscription(Float64, f'{self.leader_ns}/global_position/rel_alt', self._leader_alt_cb, qos_sensor)
        self.create_subscription(Float64, f'{self.leader_ns}/global_position/compass_hdg', self._leader_heading_cb, qos_sensor)

        # Publisher
        self.sp_pub = self.create_publisher(PositionTarget, f'{self.follower_ns}/setpoint_raw/local', 10)

        # Services
        self.arm_cli     = self.create_client(CommandBool, f'{self.follower_ns}/cmd/arming')
        self.tko_cli     = self.create_client(CommandTOL,  f'{self.follower_ns}/cmd/takeoff')
        self.setmode_cli = self.create_client(SetMode,     f'{self.follower_ns}/set_mode')

        self.get_logger().info('Waiting for MAVROS services...')
        for cli in (self.arm_cli, self.tko_cli, self.setmode_cli):
            cli.wait_for_service()
        self.get_logger().info('MAVROS services ready.')

    # Callbacks
    def _state_cb(self, msg: State): self.my_state = msg
    def _alt_cb(self, msg: Float64): self.my_rel_alt = float(msg.data)
    def _leader_pose_cb(self, msg: PoseStamped): self.leader_pose = msg
    def _leader_alt_cb(self, msg: Float64): self.leader_rel_alt = float(msg.data)
    def _leader_heading_cb(self, msg: Float64):
        self.leader_heading = math.radians(float(msg.data))  # deg → rad

    # Helpers
    def _set_mode(self, mode: str) -> bool:
        req = SetMode.Request(); req.base_mode = 0; req.custom_mode = mode
        fut = self.setmode_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        return fut.done() and fut.result() and fut.result().mode_sent

    def _arm(self) -> bool:
        req = CommandBool.Request(); req.value = True
        fut = self.arm_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        return fut.done() and fut.result() and fut.result().success

    def _takeoff(self, alt: float) -> bool:
        req = CommandTOL.Request()
        req.altitude = float(alt); req.latitude=0.0; req.longitude=0.0; req.min_pitch=0.0; req.yaw=float('nan')
        fut = self.tko_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        return fut.done() and fut.result() and fut.result().success

    def _publish_target_from_leader(self):
        # Copy leader pose (position) and compass heading (yaw)
        x = float(self.leader_pose.pose.position.x)
        y = float(self.leader_pose.pose.position.y)
        z = float(self.leader_pose.pose.position.z)
        yaw = self.leader_heading

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
        self.sp_pub.publish(sp)

    # Mission
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

        # Wait for leader airborne
        while rclpy.ok() and self.leader_rel_alt < (self.target_alt - self.alt_thresh):
            self.get_logger().info('Waiting for leader airborne…')
            rclpy.spin_once(self, timeout_sec=0.3)

        # Point shadow loop
        self.get_logger().info('Point shadow engaged (copy position + yaw from compass_hdg).')
        dt = 0.5 / max(1e-3, self.sp_rate_hz)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
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
