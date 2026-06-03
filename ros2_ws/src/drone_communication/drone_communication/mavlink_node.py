#!/usr/bin/env python3
"""
MAVLink communication node for ArduPilot flight controller.
Handles UART serial protocol, telemetry relay, and command dispatch.
"""
import rclpy
from rclpy.node import Node
import serial
import threading
import struct
import time

class MAVLinkNode(Node):
    def __init__(self):
        super().__init__('mavlink_node')
        self.declare_parameter('uart_device', '/dev/ttyS0')
        self.declare_parameter('baud_rate', 57600)

        self.uart_device = self.get_parameter('uart_device').value
        self.baud_rate = self.get_parameter('baud_rate').value

        self.serial = None
        self.running = True

        # Publishers
        self.drone_status_pub = self.create_publisher(
            DroneStatus, '/drone/status', 10)

        # Subscribers
        self.cmd_sub = self.create_subscription(
            String, '/drone/command', self.command_callback, 10)

        # Try to connect
        if not self.connect_serial():
            self.get_logger().error(f'Failed to open {self.uart_device}')

        # Start read thread
        self.read_thread = threading.Thread(target=self.read_loop)
        self.read_thread.start()

    def connect_serial(self):
        try:
            self.serial = serial.Serial(
                port=self.uart_device,
                baudrate=self.baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            self.get_logger().info(f'Connected to {self.uart_device} @ {self.baud_rate}')
            return True
        except Exception as e:
            self.get_logger().error(f'Serial error: {e}')
            return False

    def read_loop(self):
        """Continuously read MAVLink messages from serial."""
        while self.running and self.serial:
            try:
                if self.serial.in_waiting > 0:
                    data = self.serial.read(self.serial.in_waiting)
                    self.parse_mavlink(data)
            except Exception as e:
                self.get_logger().error(f'Read error: {e}')
                time.sleep(0.1)

    def parse_mavlink(self, data):
        """Parse MAVLink packets and publish relevant data."""
        # TODO: Implement full MAVLink v2 parsing
        # For now, basic heartbeat detection
        for byte in data:
            # MAVLink v1/v2 stub parsing
            pass

    def command_callback(self, msg):
        """Handle /drone/command messages (ARM, TAKEOFF, LAND, etc.)."""
        self.get_logger().info(f'Command received: {msg.data}')
        # TODO: Send MAVLink command to flight controller

    def send_mavlink(self, packet):
        """Send raw MAVLink packet over UART."""
        if self.serial and self.serial.is_open:
            self.serial.write(packet)

    def destroy(self):
        self.running = False
        if self.serial:
            self.serial.close()
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
