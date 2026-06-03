#!/usr/bin/env python3
"""MAVLink communication node for ArduPilot via UART. Uses pymavlink."""
import rclpy
from rclpy.node import Node
from drone_interfaces.msg import DroneStatus
from std_msgs.msg import String
import threading
import time

# ArduPilot MAVLink constants
MAV_MODE_FLAG_SAFETY_ARMED = 128
MAV_CMD_ARM = 400
MAV_CMD_TAKEOFF = 22
MAV_CMD_LAND = 21
MAV_CMD_RTL = 20


class MAVLinkNode(Node):
    def __init__(self):
        super().__init__('mavlink_node')
        self.declare_parameter('uart_device', '/dev/ttyS0')
        self.declare_parameter('baud_rate', 57600)

        self.uart_device = self.get_parameter('uart_device').value
        self.baud_rate = self.get_parameter('baud_rate').value

        self.conn = None
        self.running = True
        self._lock = threading.Lock()

        self.drone_status_pub = self.create_publisher(
            DroneStatus, '/drone/status', 10)
        self.cmd_sub = self.create_subscription(
            String, '/drone/command', self.command_callback, 10)

        self._connect()

        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()

        # Periodic status publish (5 Hz) even if no new MAVLink msg
        self._status_timer = self.create_timer(0.2, self._publish_status)

        self._last_status = DroneStatus()

    def _connect(self):
        try:
            from pymavlink import mavutil
            self.conn = mavutil.mavlink_connection(
                device=self.uart_device,
                baud=self.baud_rate,
                source_system=1,
            )
            self.get_logger().info(
                f'MAVLink connected: {self.uart_device} @ {self.baud_rate}')
        except ImportError:
            self.get_logger().error(
                'pymavlink not installed. Run: pip3 install pymavlink')
            self.conn = None
        except Exception as e:
            self.get_logger().error(f'Failed to open {self.uart_device}: {e}')
            self.conn = None

    def _read_loop(self):
        while self.running and self.conn:
            try:
                msg = self.conn.recv_match(blocking=True, timeout=0.5)
                if msg is None:
                    continue
                self._handle_mavlink(msg)
            except Exception as e:
                if self.running:
                    self.get_logger().error(f'Read error: {e}')
                    time.sleep(0.5)

    def _handle_mavlink(self, msg):
        msg_type = msg.get_type()
        s = self._last_status

        if msg_type == 'HEARTBEAT':
            s.armed = bool(msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
            # Map ArduPilot custom_mode to simplified modes
            mode_map = {0: 0, 1: 0, 2: 1, 3: 2, 4: 3, 5: 4, 6: 3, 7: 5, 9: 3}
            s.mode = mode_map.get(msg.custom_mode, 0)

        elif msg_type == 'ATTITUDE':
            s.roll = msg.roll
            s.pitch = msg.pitch
            s.yaw = msg.yaw

        elif msg_type == 'GLOBAL_POSITION_INT':
            s.altitude = msg.relative_alt / 1000.0  # mm to m
            s.heading = msg.hdg / 100.0  # centideg to deg
            s.latitude = msg.lat
            s.longitude = msg.lon

        elif msg_type == 'VFR_HUD':
            s.ground_speed = msg.groundspeed  # m/s

        elif msg_type == 'SYS_STATUS':
            s.battery_voltage = msg.voltage_battery / 1000.0  # mV to V
            s.battery_current = msg.current_battery / 100.0  # cA to A

    def _publish_status(self):
        if self._last_status is not None:
            self.drone_status_pub.publish(self._last_status)

    def command_callback(self, msg):
        cmd = msg.data.strip().upper()
        self.get_logger().info(f'Command received: {cmd}')

        if not self.conn:
            return

        with self._lock:
            try:
                if cmd == 'ARM':
                    self.conn.mav.command_long_send(
                        self.conn.target_system, self.conn.target_component,
                        MAV_CMD_ARM, 0, 1, 0, 0, 0, 0, 0, 0)
                elif cmd == 'TAKEOFF':
                    self.conn.mav.command_long_send(
                        self.conn.target_system, self.conn.target_component,
                        MAV_CMD_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 5)
                elif cmd == 'LAND':
                    self.conn.mav.command_long_send(
                        self.conn.target_system, self.conn.target_component,
                        MAV_CMD_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
                elif cmd == 'RTL':
                    self.conn.mav.command_long_send(
                        self.conn.target_system, self.conn.target_component,
                        MAV_CMD_RTL, 0, 0, 0, 0, 0, 0, 0, 0)
                elif cmd.startswith('GUIDED '):
                    _, lat, lon, alt = cmd.split()
                    self.conn.mav.command_long_send(
                        self.conn.target_system, self.conn.target_component,
                        MAV_CMD_TAKEOFF, 0, 0, 0, 0,
                        float(lat), float(lon), float(alt))
                else:
                    self.get_logger().warn(f'Unknown command: {cmd}')
            except Exception as e:
                self.get_logger().error(f'Failed to send command: {e}')

    def destroy(self):
        self.running = False
        if hasattr(self, '_status_timer'):
            self.destroy_timer(self._status_timer)
        with self._lock:
            if self.conn:
                self.conn.close()
        super().destroy()


def main(args=None):
    rclpy.init(args=args)
    node = MAVLinkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
