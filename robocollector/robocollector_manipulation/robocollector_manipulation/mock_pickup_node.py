#!/usr/bin/env python3
"""
RoboCollector Mock Pickup Node
Simulates arm movements without real hardware for testing
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import time


class MockPickupNode(Node):
    def __init__(self):
        super().__init__("mock_pickup_node")

        # Service interfaces (same as real pickup node)
        self.create_service(Trigger, "~/enter", self.enter_srv_callback)
        self.create_service(Trigger, "~/exit", self.exit_srv_callback)
        self.create_service(Trigger, "~/init_finish", self.get_node_state)
        self.create_service(Trigger, "~/pickup", self.pickup_srv_callback)
        self.create_service(Trigger, "~/place", self.place_srv_callback)
        self.create_service(Trigger, "~/home", self.home_srv_callback)

        self.get_logger().info(
            "\033[1;32m%s\033[0m" % "Mock pickup node initialized (simulation mode)"
        )

    def enter_srv_callback(self, request, response):
        self.get_logger().info("\033[1;32m%s\033[0m" % "[SIM] Pickup node entered")
        response.success = True
        response.message = "Mock pickup ready"
        return response

    def exit_srv_callback(self, request, response):
        self.get_logger().info("\033[1;32m%s\033[0m" % "[SIM] Pickup node exited")
        response.success = True
        return response

    def get_node_state(self, request, response):
        response.success = True
        return response

    def pickup_srv_callback(self, request, response):
        self.get_logger().info("\033[1;33m%s\033[0m" % "[SIM] === PICKUP SEQUENCE ===")
        self.get_logger().info("[SIM] Step 1: Opening gripper...")
        time.sleep(0.5)
        self.get_logger().info("[SIM] Step 2: Moving to pickup position...")
        time.sleep(1.0)
        self.get_logger().info("[SIM] Step 3: Closing gripper on block...")
        time.sleep(0.5)
        self.get_logger().info("[SIM] Step 4: Lifting block...")
        time.sleep(1.0)
        self.get_logger().info("\033[1;32m%s\033[0m" % "[SIM] ✓ Pickup complete!")

        response.success = True
        response.message = "Mock pickup successful"
        return response

    def place_srv_callback(self, request, response):
        self.get_logger().info("\033[1;33m%s\033[0m" % "[SIM] === PLACE SEQUENCE ===")
        self.get_logger().info("[SIM] Step 1: Lowering to place position...")
        time.sleep(1.0)
        self.get_logger().info("[SIM] Step 2: Opening gripper...")
        time.sleep(0.5)
        self.get_logger().info("[SIM] Step 3: Returning to home...")
        time.sleep(1.0)
        self.get_logger().info("\033[1;32m%s\033[0m" % "[SIM] ✓ Place complete!")

        response.success = True
        response.message = "Mock place successful"
        return response

    def home_srv_callback(self, request, response):
        self.get_logger().info(
            "\033[1;33m%s\033[0m" % "[SIM] Moving to home position..."
        )
        time.sleep(1.0)
        self.get_logger().info("\033[1;32m%s\033[0m" % "[SIM] ✓ Home position reached")

        response.success = True
        response.message = "Mock home complete"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MockPickupNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
