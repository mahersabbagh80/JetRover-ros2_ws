#!/usr/bin/env python3
"""
RoboCollector Pickup Node
Controls the robotic arm for picking up and placing blocks.
Follows JetRover service-based architecture pattern.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from interfaces.srv import SetString
from geometry_msgs.msg import Point
import threading
import time


class PickupNode(Node):
    def __init__(self, name="pickup_node"):
        rclpy.init()
        super().__init__(
            name,
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        self.name = name
        self.lock = threading.RLock()

        # Servo positions (calibrated for JetRover arm)
        # Format: [servo_id, position]
        self.positions = {
            "home": {
                "base": 500,  # Servo 6 - base rotation
                "shoulder": 500,  # Servo 5 - shoulder
                "elbow": 500,  # Servo 4 - elbow
                "wrist": 500,  # Servo 3 - wrist
                "gripper": 500,  # Servo 2 - gripper open/closed
            },
            "pickup": {
                "base": 500,
                "shoulder": 300,
                "elbow": 700,
                "wrist": 500,
                "gripper": 300,  # Open gripper
            },
            "lift": {
                "base": 500,
                "shoulder": 500,
                "elbow": 500,
                "wrist": 500,
                "gripper": 700,  # Closed gripper
            },
        }

        # Service clients to robot controller
        self.servo_client = None

        # Current state
        self.current_pose = "home"
        self.is_executing = False

        # Service interfaces (JetRover pattern)
        self.create_service(Trigger, "~/enter", self.enter_srv_callback)
        self.create_service(Trigger, "~/exit", self.exit_srv_callback)
        self.create_service(Trigger, "~/init_finish", self.get_node_state)
        self.create_service(Trigger, "~/pickup", self.pickup_srv_callback)
        self.create_service(Trigger, "~/place", self.place_srv_callback)
        self.create_service(Trigger, "~/home", self.home_srv_callback)

        self.get_logger().info("\033[1;32m%s\033[0m" % f"{self.name} initialized")

    def enter_srv_callback(self, request, response):
        """Initialize arm control when app enters"""
        self.get_logger().info("\033[1;32m%s\033[0m" % "pickup enter")
        with self.lock:
            # Create service client for servo control
            if self.servo_client is None:
                self.servo_client = self.create_client(
                    Trigger,  # Placeholder - actual service type depends on robot controller
                    "/ros_robot_controller/bus_servo/set_position",
                )

            # Move to home position
            self.move_to_pose("home")

        response.success = True
        response.message = "Pickup node started"
        return response

    def exit_srv_callback(self, request, response):
        """Clean up when app exits"""
        self.get_logger().info("\033[1;32m%s\033[0m" % "pickup exit")
        try:
            with self.lock:
                # Return to home position
                self.move_to_pose("home")

                if self.servo_client is not None:
                    self.destroy_client(self.servo_client)
                    self.servo_client = None
        except Exception as e:
            self.get_logger().error(f"Exit error: {str(e)}")

        response.success = True
        response.message = "Pickup node stopped"
        return response

    def get_node_state(self, request, response):
        """Return node initialization state"""
        response.success = True
        return response

    def pickup_srv_callback(self, request, response):
        """Execute pickup sequence"""
        self.get_logger().info("\033[1;32m%s\033[0m" % "Executing pickup")

        with self.lock:
            if self.is_executing:
                response.success = False
                response.message = "Already executing a command"
                return response

            self.is_executing = True

        try:
            # Pickup sequence
            self.get_logger().info("Step 1: Open gripper")
            self.set_gripper(300)  # Open
            time.sleep(0.5)

            self.get_logger().info("Step 2: Move to pickup position")
            self.move_to_pose("pickup")
            time.sleep(1.0)

            self.get_logger().info("Step 3: Close gripper")
            self.set_gripper(700)  # Close
            time.sleep(0.5)

            self.get_logger().info("Step 4: Lift object")
            self.move_to_pose("lift")
            time.sleep(1.0)

            response.success = True
            response.message = "Pickup complete"
        except Exception as e:
            self.get_logger().error(f"Pickup failed: {str(e)}")
            response.success = False
            response.message = str(e)
        finally:
            self.is_executing = False

        return response

    def place_srv_callback(self, request, response):
        """Execute place sequence"""
        self.get_logger().info("\033[1;32m%s\033[0m" % "Executing place")

        with self.lock:
            if self.is_executing:
                response.success = False
                response.message = "Already executing a command"
                return response

            self.is_executing = True

        try:
            # Place sequence
            self.get_logger().info("Step 1: Lower to place position")
            self.move_to_pose("pickup")
            time.sleep(1.0)

            self.get_logger().info("Step 2: Open gripper")
            self.set_gripper(300)  # Open
            time.sleep(0.5)

            self.get_logger().info("Step 3: Return to home")
            self.move_to_pose("home")
            time.sleep(1.0)

            response.success = True
            response.message = "Place complete"
        except Exception as e:
            self.get_logger().error(f"Place failed: {str(e)}")
            response.success = False
            response.message = str(e)
        finally:
            self.is_executing = False

        return response

    def home_srv_callback(self, request, response):
        """Return arm to home position"""
        self.get_logger().info("\033[1;32m%s\033[0m" % "Moving to home")

        try:
            self.move_to_pose("home")
            response.success = True
            response.message = "Home position reached"
        except Exception as e:
            self.get_logger().error(f"Home move failed: {str(e)}")
            response.success = False
            response.message = str(e)

        return response

    def move_to_pose(self, pose_name):
        """Move all servos to a predefined pose"""
        if pose_name not in self.positions:
            self.get_logger().error(f"Unknown pose: {pose_name}")
            return

        pose = self.positions[pose_name]
        self.current_pose = pose_name

        # TODO: Call actual servo control service
        # For now, just log the movement
        self.get_logger().info(f"Moving to {pose_name}: {pose}")

        # Placeholder for actual servo control:
        # servo_positions = [
        #     [6, pose['base']],
        #     [5, pose['shoulder']],
        #     [4, pose['elbow']],
        #     [3, pose['wrist']],
        #     [2, pose['gripper']]
        # ]
        # duration = 1000  # milliseconds
        # self.board.bus_servo_set_position(duration, servo_positions)

    def set_gripper(self, position):
        """Set gripper position (300=open, 700=closed)"""
        self.get_logger().info(f"Setting gripper to {position}")
        # TODO: Call actual servo control


def main(args=None):
    node = PickupNode("pickup_node")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
