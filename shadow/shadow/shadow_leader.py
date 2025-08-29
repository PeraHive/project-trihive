#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode

class ShadowLeader(Node):
    def __init__(self):
        super().__init__('shadow_leader')

        # --- subs to monitor state/altitude ---
        self.state = State()
        self.rel_alt = 0.0
        self.state_sub = self.create_subscription(State, '/uav1/state', self._state_cb, 10)
        self.alt_sub   = self.create_subscription(Float64, '/uav1/global_position/rel_alt', self._alt_cb, 10)

        # --- service clients ---
        self.arm_cli      = self.create_client(CommandBool, '/uav1/cmd/arming')
        self.tko_cli      = self.create_client(CommandTOL,  '/uav1/cmd/takeoff')
        self.setmode_cli  = self.create_client(SetMode,     '/uav1/set_mode')

        # wait for services
        self.get_logger().info('Waiting for MAVROS services...')
        for cli in (self.arm_cli, self.tko_cli, self.setmode_cli):
            cli.wait_for_service()
        self.get_logger().info('MAVROS services are available.')

    # ---------------- Callbacks ----------------
    def _state_cb(self, msg: State):
        self.state = msg

    def _alt_cb(self, msg: Float64):
        self.rel_alt = msg.data

    # ---------------- Helpers -----------------
    def _set_mode(self, mode: str, timeout: float = 10.0) -> bool:
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = mode
        fut = self.setmode_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        ok = bool(fut.done() and fut.result() and fut.result().mode_sent)
        self.get_logger().info(f"SetMode('{mode}') -> {ok}")
        return ok

    def _arm_once(self, timeout: float = 10.0) -> bool:
        req = CommandBool.Request()
        req.value = True
        fut = self.arm_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        ok = bool(fut.done() and fut.result() and fut.result().success)
        self.get_logger().info(f"Arm attempt -> {ok}")
        return ok

    def _takeoff(self, alt: float = 10.0, timeout: float = 10.0) -> bool:
        req = CommandTOL.Request()
        req.altitude = float(alt)
        req.latitude = 0.0
        req.longitude = 0.0
        req.min_pitch = 0.0
        req.yaw = 0.0
        fut = self.tko_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        ok = bool(fut.done() and fut.result() and fut.result().success)
        self.get_logger().info(f"Takeoff({alt} m) -> {ok}")
        return ok

    # ---------------- Mission -----------------
    def run(self, target_alt: float = 10.0, arm_retry_sec: float = 2.0):
        # 1) wait FCU
        while rclpy.ok() and not self.state.connected:
            self.get_logger().info('Waiting for FCU connection...')
            rclpy.spin_once(self, timeout_sec=0.2)

        # 2) GUIDED
        self._set_mode('GUIDED')
        time.sleep(1.0)

        # 3) Loop arming until armed
        while rclpy.ok() and not self.state.armed:
            if not self._arm_once():
                self.get_logger().warn('Arming failed, retrying...')
            # small wait between attempts
            time.sleep(arm_retry_sec)
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info('Vehicle is ARMED.')

        # 4) Takeoff and wait until we’re at altitude
        if self._takeoff(target_alt):
            self.get_logger().info('Waiting to reach target altitude...')
            while rclpy.ok() and abs(self.rel_alt - target_alt) > 2:
                rclpy.spin_once(self, timeout_sec=0.2)
                time.sleep(0.2)

        # 5) Switch to POSHOLD
        self._set_mode('POSHOLD')
        self.get_logger().info('Switched to POSHOLD. Leader sequence complete.')

def main():
    rclpy.init()
    node = ShadowLeader()
    try:
        node.run(target_alt=10.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
