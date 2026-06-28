"""MAVLink communication node — pyserial + pymavlink parse_char with ACM-tolerant error handling."""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from drone_interfaces.msg import DroneStatus
from std_msgs.msg import String
import serial
import threading
import time

MAV_MODE_FLAG_SAFETY_ARMED = 128
MAV_CMD_ARM = 400
MAV_CMD_TAKEOFF = 22
MAV_CMD_LAND = 21
MAV_CMD_RTL = 20
MAV_DATA_STREAM_ALL = 2
MODE_MAP = {0: 0, 1: 0, 2: 1, 3: 2, 4: 3, 5: 4, 6: 3, 7: 5, 9: 3}


class _SerialWriter:
    """Thread-safe write wrapper — pymavlink calls write() to send packets."""

    def __init__(self, ser, lock):
        self.ser = ser
        self.lock = lock

    def write(self, data):
        with self.lock:
            if self.ser and self.ser.is_open:
                self.ser.write(data)


class MAVLinkNode(Node):
    def __init__(self):
        super().__init__('mavlink_node')
        self.declare_parameter('uart_device', '/dev/ttyS0')
        self.declare_parameter('baud_rate', 57600)
        self.declare_parameter('target_system', 1)

        self.uart_device = self.get_parameter('uart_device').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.target_sys = self.get_parameter('target_system').value

        self.ser = None
        self.mav = None
        self.running = True
        self._lock = threading.Lock()
        self._streams_requested = False
        self._msg_count = 0

        self.drone_status_pub = self.create_publisher(
            DroneStatus, '/drone/status', 10)
        self.cmd_sub = self.create_subscription(
            String, '/drone/command', self.command_callback, 10)

        self._connect()
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()

        self._status_timer = self.create_timer(0.2, self._publish_status)
        self._hb_timer = self.create_timer(1.0, self._send_heartbeat)
        self._last_status = DroneStatus()

    # ── connection ──────────────────────────────────────────────

    def _connect(self):
        try:
            self.ser = serial.Serial(
                port=self.uart_device,
                baudrate=self.baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.5,
            )
            from pymavlink.dialects.v20 import ardupilotmega as mavlink
            self.mav = mavlink.MAVLink(_SerialWriter(self.ser, self._lock))
            self.get_logger().info(
                f'Connected: {self.uart_device} @ {self.baud_rate}')
        except ImportError:
            self.get_logger().error('pymavlink not installed')
            self.ser = None
        except Exception as e:
            self.get_logger().error(f'Failed to open {self.uart_device}: {e}')
            self.ser = None

    def _reconnect(self):
        self.get_logger().info('Reconnecting...')
        try:
            with self._lock:
                if self.ser:
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                self.ser = None
                self.mav = None
                self._streams_requested = False
        except Exception:
            pass
        time.sleep(1.5)
        self._connect()

    # ── outbound ────────────────────────────────────────────────

    def _send_heartbeat(self):
        """Periodic heartbeat keeps ArduPilot data stream alive."""
        if not self.mav:
            return
        try:
            self.mav.heartbeat_send(6, 8, 0, 0, 0)
        except Exception:
            pass

    def _request_streams(self):
        if not self.mav:
            return
        self.get_logger().info(f'Requesting streams (sys={self.target_sys})')
        self.mav.request_data_stream_send(
            self.target_sys, 1, MAV_DATA_STREAM_ALL, 10, 1)

    # ── inbound ─────────────────────────────────────────────────

    def _read_loop(self):
        while self.running:
            try:
                ser = self.ser
                if not ser or not ser.is_open:
                    if self.running:
                        self._reconnect()
                    continue

                try:
                    data = ser.read(512)
                except (serial.SerialException, TypeError, AttributeError):
                    if not self.running:
                        continue
                    time.sleep(0.3)
                    ser = self.ser
                    if ser is None or not ser.is_open:
                        if self.running:
                            self._reconnect()
                        continue
                    try:
                        data = ser.read(512)
                    except (serial.SerialException, TypeError, AttributeError):
                        if self.running:
                            self.get_logger().warn('ACM read failed, reconnecting...')
                            self._reconnect()
                        continue

                if not data:
                    continue

                for b in data:
                    try:
                        msg = self.mav.parse_char(bytes([b]))
                    except Exception:
                        continue
                    if msg is None or msg.get_type() == 'BAD_DATA':
                        continue
                    self._msg_count += 1
                    self._handle_mavlink(msg)

            except Exception:
                self.get_logger().error(f'Read-loop error:\n{__import__("traceback").format_exc()}')
                time.sleep(1.0)

    def _handle_mavlink(self, msg):
        msg_type = msg.get_type()
        s = self._last_status

        if msg_type == 'HEARTBEAT':
            if not self._streams_requested:
                self._streams_requested = True
                self.get_logger().info(
                    f'Heartbeat — armed={bool(msg.base_mode & 128)}')
                self._request_streams()
            s.armed = bool(msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
            s.mode = MODE_MAP.get(msg.custom_mode, 0)

        elif msg_type == 'ATTITUDE':
            s.roll = msg.roll
            s.pitch = msg.pitch
            s.yaw = msg.yaw
            if self._msg_count % 50 == 0:
                self.get_logger().info(
                    f'Attitude: roll={msg.roll:.2f} '
                    f'pitch={msg.pitch:.2f} yaw={msg.yaw:.2f}')

        elif msg_type == 'GLOBAL_POSITION_INT':
            s.altitude = msg.relative_alt / 1000.0
            s.heading = msg.hdg / 100.0
            s.latitude = msg.lat
            s.longitude = msg.lon

        elif msg_type == 'VFR_HUD':
            s.ground_speed = msg.groundspeed

        elif msg_type == 'SYS_STATUS':
            s.battery_voltage = msg.voltage_battery / 1000.0
            s.battery_current = msg.current_battery / 100.0

    # ── ROS2 interface ──────────────────────────────────────────

    def _publish_status(self):
        self.drone_status_pub.publish(self._last_status)

    def command_callback(self, msg):
        cmd = msg.data.strip().upper()
        self.get_logger().info(f'Command: {cmd}')

        if not self.mav:
            return

        if cmd == 'ARM':
            self.mav.command_long_send(
                self.target_sys, 1, MAV_CMD_ARM, 0, 1, 0, 0, 0, 0, 0, 0)
        elif cmd == 'TAKEOFF':
            self.mav.command_long_send(
                self.target_sys, 1, MAV_CMD_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 5)
        elif cmd == 'LAND':
            self.mav.command_long_send(
                self.target_sys, 1, MAV_CMD_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
        elif cmd == 'RTL':
            self.mav.command_long_send(
                self.target_sys, 1, MAV_CMD_RTL, 0, 0, 0, 0, 0, 0, 0, 0)
        elif cmd.startswith('GUIDED '):
            _, lat, lon, alt = cmd.split()
            self.mav.command_long_send(
                self.target_sys, 1, MAV_CMD_TAKEOFF, 0, 0, 0, 0,
                float(lat), float(lon), float(alt))
        else:
            self.get_logger().warn(f'Unknown command: {cmd}')

    def destroy(self):
        self.running = False
        self.destroy_timer(self._status_timer)
        self.destroy_timer(self._hb_timer)
        with self._lock:
            if self.ser and self.ser.is_open:
                self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MAVLinkNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
