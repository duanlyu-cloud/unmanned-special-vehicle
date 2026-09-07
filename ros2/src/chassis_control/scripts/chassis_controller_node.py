#!/usr/bin/env python3
"""
Chassis Controller Node — ROS2 service server for PS100 servo chassis.

Service: /chassis/move_to (robot_interfaces/srv/ChassisMove)
  Takes a station_id (string), looks up position from waypoints.yaml,
  moves the chassis via Modbus, and returns success/failure.

Usage:
  ros2 run chassis_control chassis_controller_node \
    --ros-args -p waypoints_yaml_path:=src/chassis_control/config/waypoints.yaml
"""
import os
import time
import math
import yaml
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from robot_interfaces.srv import ChassisMove

from chassis_control.chassis_control.ps100_driver import PS100Driver, ALARM_TORQUE_OVER


class ChassisController(Node):
    """ROS2 node wrapping PS100Driver as a /chassis/move_to service."""

    def __init__(self):
        super().__init__("chassis_controller")

        # ── Declare parameters ──
        self.declare_parameter("waypoints_yaml_path", "")
        self.declare_parameter("port", "/dev/ttyUSB1")
        self.declare_parameter("slave_addr", 1)
        self.declare_parameter("baudrate", 9600)

        # ── Load waypoints ──
        yaml_path = self.get_parameter("waypoints_yaml_path").value
        if not yaml_path:
            # Default: look in package share directory
            from ament_index_python.packages import get_package_share_directory
            yaml_path = os.path.join(
                get_package_share_directory("chassis_control"),
                "config", "waypoints.yaml"
            )

        self._waypoints = {}
        self._limits = {}
        self._motion = {}
        self._driver_config = {}
        try:
            with open(yaml_path, "r") as f:
                data = yaml.safe_load(f)
            self._waypoints = data.get("waypoints", {})
            self._limits = data.get("limits", {})
            self._motion = data.get("motion", {})
            self._driver_config = data.get("driver", {})
            self.get_logger().info(f"Loaded {len(self._waypoints)} waypoints from {yaml_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints: {e}")

        # ── Connect to PS100 ──
        port = self._driver_config.get("port", self.get_parameter("port").value)
        slave = self._driver_config.get("slave_addr", self.get_parameter("slave_addr").value)
        baud = self._driver_config.get("baudrate", self.get_parameter("baudrate").value)

        self._driver = PS100Driver(port=port, slave_addr=slave, baudrate=baud)
        self._connected = False

        # ── Create service ──
        self._cb_group = MutuallyExclusiveCallbackGroup()
        self._srv = self.create_service(
            ChassisMove, "/chassis/move_to",
            self._handle_move_to,
            callback_group=self._cb_group
        )

        self.get_logger().info("ChassisController ready (not connected yet — call /chassis/connect first)")

    # ── Connection management (can be called via service or auto) ──

    def connect(self) -> bool:
        if self._connected:
            return True
        self._connected = self._driver.connect()
        if self._connected:
            self.get_logger().info("Chassis connected to PS100")
        else:
            self.get_logger().error("Failed to connect to PS100")
        return self._connected

    def disconnect(self):
        self._driver.disconnect()
        self._connected = False

    # ── Service handler ──

    def _handle_move_to(self, request, response):
        """Service callback: move chassis to named station."""
        station_id = request.station_id

        if not self._connected:
            response.success = False
            response.error_code = 3
            response.message = "PS100 not connected"
            self.get_logger().error("Cannot move: PS100 not connected")
            return response

        # Look up station — supports "task_station" and "material_N" formats
        wp = None
        if station_id == "task_station":
            wp = self._waypoints.get("task_station")
        elif station_id.startswith("material_"):
            try:
                idx = int(station_id.split("_")[1]) - 1  # material_1 → index 0
                material_stations = self._waypoints.get("material_stations", [])
                if 0 <= idx < len(material_stations):
                    wp = material_stations[idx]
            except (ValueError, IndexError):
                pass

        if not wp:
            response.success = False
            response.error_code = 4
            response.message = f"Unknown station: {station_id}"
            self.get_logger().error(f"Unknown station: {station_id}")
            return response

        target_mm = wp["position"]
        speed_rpm = wp.get("speed", self._motion.get("default_speed", 200))
        tolerance_mm = self._motion.get("position_tolerance", 1.0)
        max_retries = self._motion.get("max_retries", 2)

        # Check soft limits
        min_pos = self._limits.get("min_position", -10.0)
        max_pos = self._limits.get("max_position", 500.0)
        if target_mm < min_pos or target_mm > max_pos:
            response.success = False
            response.error_code = 5
            response.message = f"Position {target_mm}mm out of limits [{min_pos}, {max_pos}]"
            return response

        self.get_logger().info(f"Moving to {station_id} ({target_mm}mm @ {speed_rpm}rpm)")

        # ── Execute move with retries ──
        for attempt in range(max_retries + 1):
            if attempt > 0:
                self.get_logger().warn(f"Retry {attempt}/{max_retries}")

            # Check alarm before moving
            alarm = self._driver.read_alarm()
            if alarm != 0:
                self.get_logger().warn(f"Alarm {alarm} detected, attempting recovery")
                if not self._driver.recover_alarm():
                    response.success = False
                    response.error_code = 2
                    response.message = f"Alarm {alarm} recovery failed"
                    return response

            # Trigger move
            self._driver.move_to(target_mm, speed_rpm)

            # ── Wait for completion ──
            estimated_seconds = abs(target_mm) / (speed_rpm * PITCH_MM_PER_REV / 60.0)
            timeout_factor = self._motion.get("move_timeout_factor", 2.0)
            timeout = max(5.0, estimated_seconds * timeout_factor)
            t0 = time.time()

            while time.time() - t0 < timeout:
                alarm = self._driver.read_alarm()
                if alarm == ALARM_TORQUE_OVER:
                    self.get_logger().error("Err29 stall detected during move!")
                    self._driver.recover_err29()
                    response.success = False
                    response.error_code = 2
                    response.message = "Err29 stall during move"
                    return response
                elif alarm != 0:
                    self.get_logger().warn(f"Alarm {alarm} during move")

                if not self._driver.is_moving():
                    break  # Motor stopped

                time.sleep(0.1)

            # Timeout check
            if self._driver.is_moving():
                self.get_logger().error("Move timed out!")
                self._driver.emergency_stop()
                self._driver.re_enable()
                if attempt < max_retries:
                    continue
                response.success = False
                response.error_code = 1
                response.message = "Move timed out"
                return response

            # ── Clear trigger (prevent residual auto-run) ──
            self._driver.clear_trigger()

            # ── Verify position ──
            actual_mm = self._driver.read_position_mm()
            error_mm = abs(actual_mm - target_mm)
            self.get_logger().info(f"Position: target={target_mm}mm, actual={actual_mm:.2f}mm, error={error_mm:.2f}mm")

            if error_mm <= tolerance_mm:
                response.success = True
                response.actual_position = actual_mm
                response.error_code = 0
                response.message = f"Reached {station_id} at {actual_mm:.2f}mm"
                return response

            # Position error — retry
            self.get_logger().warn(f"Position error {error_mm:.2f}mm > tolerance {tolerance_mm}mm")

        # Retries exhausted
        response.success = False
        response.error_code = 6
        response.message = f"Position accuracy failed after {max_retries+1} attempts"
        return response


# ── Constants ──
PITCH_MM_PER_REV = 5.0  # Must match ps100_driver.py


def main():
    rclpy.init()
    node = ChassisController()

    # Auto-connect
    if node.connect():
        node.get_logger().info("ChassisController running, service /chassis/move_to ready")
    else:
        node.get_logger().warn("ChassisController running WITHOUT PS100 connection")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.disconnect()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
