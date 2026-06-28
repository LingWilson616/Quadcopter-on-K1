#!/usr/bin/env python3
"""
Virtual ArduPilot Flight Controller for K1 Drone testing.

Creates a virtual serial port (PTY pair) and simulates a quadcopter FC
speaking MAVLink v2. Responds to commands and sends telemetry.

Usage:
    python3 virtual_fc.py
    # Prints the PTY path, then in another terminal:
    ros2 launch drone_bringup drone.launch.py uart_device:=/dev/pts/N

Keyboard controls (press key + Enter):
    a  = ARM
    d  = DISARM
    t  = TAKEOFF (to 5m)
    l  = LAND
    r  = RTL
    q  = quit
"""

import os
import sys
import time
import signal
import threading
import math
import select

from pymavlink.dialects.v20 import ardupilotmega as mavlink


class _PTYFile:
    """Minimal file-like wrapper around a PTY master fd."""
    def __init__(self, fd):
        self.fd = fd

    def write(self, data):
        return os.write(self.fd, data)

    def read(self, n):
        return os.read(self.fd, n)

    def fileno(self):
        return self.fd


MAV_CMD_ARM = 400
MAV_CMD_TAKEOFF = 22
MAV_CMD_LAND = 21
MAV_CMD_RTL = 20
MAV_DATA_STREAM_ALL = 2
MAV_MODE_FLAG_SAFETY_ARMED = 128


class VirtualFC:
    def __init__(self):
        self.master_fd, slave_fd = os.openpty()
        self.slave_path = os.ttyname(slave_fd)
        self.slave_fd = slave_fd

        self.pty = _PTYFile(self.master_fd)
        self.mav = mavlink.MAVLink(self.pty, srcSystem=1, srcComponent=1)

        # Flight state
        self.armed = False
        self.flying = False
        self.target_alt = 0.0

        # Position
        self.lat = 31_230_000  # degrees * 1e7
        self.lon = 121_470_000
        self.alt = 0.0
        self.heading = 0.0

        # Attitude
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # Battery
        self.voltage = 12.6
        self.current = 3.0
        self.battery_pct = 95

        self.groundspeed = 0.0

        self.running = True
        self.t0 = time.time()
        self.tick = 0

        os.set_blocking(self.master_fd, False)

    def start(self):
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        self._input = threading.Thread(target=self._input_loop, daemon=True)
        self._input.start()

    # ── MAVLink read ─────────────────────────────────────────

    def _read_loop(self):
        while self.running:
            try:
                r, _, _ = select.select([self.master_fd], [], [], 0.1)
                if not r:
                    continue
                data = self.pty.read(512)
                if not data:
                    continue
                for b in data:
                    msg = self.mav.parse_char(bytes([b]))
                    if msg is None:
                        continue
                    self._handle_msg(msg)
            except BlockingIOError:
                pass
            except Exception:
                if self.running:
                    time.sleep(0.5)

    def _handle_msg(self, msg):
        t = msg.get_type()
        if t == 'HEARTBEAT':
            print(f"  [MAVLink] HEARTBEAT from sys={msg.get_srcSystem()}")
        elif t == 'REQUEST_DATA_STREAM':
            print(f"  [MAVLink] Stream request: id={msg.req_stream_id} rate={msg.req_message_rate}")
        elif t == 'COMMAND_LONG':
            self._handle_command(msg)
        elif t == 'COMMAND_INT':
            self._handle_command(msg)

    def _handle_command(self, msg):
        cmd = msg.command
        if cmd == MAV_CMD_ARM:
            self.armed = bool(msg.param1)
            self.flying = False
            state = 'ARMED' if self.armed else 'DISARMED'
            print(f"\n  >>> {state} <<<")
            self._cmd_ack(cmd, 0)

        elif cmd == MAV_CMD_TAKEOFF:
            if not self.armed:
                print("  [WARN] TAKEOFF rejected: not armed")
                self._cmd_ack(cmd, 4)  # MAV_RESULT_DENIED
                return
            self.flying = True
            self.target_alt = msg.param7 if msg.param7 > 0 else 5.0
            print(f"\n  >>> TAKEOFF to {self.target_alt}m <<<")
            self._cmd_ack(cmd, 0)

        elif cmd == MAV_CMD_LAND:
            print(f"\n  >>> LANDING <<<")
            self.flying = False
            self.armed = False
            self.target_alt = 0.0
            self._cmd_ack(cmd, 0)

        elif cmd == MAV_CMD_RTL:
            print(f"\n  >>> RETURN TO LAUNCH <<<")
            self.flying = False
            self.armed = False
            self.target_alt = 0.0
            self._cmd_ack(cmd, 0)

    def _cmd_ack(self, cmd, result):
        self.mav.command_ack_send(cmd, result)

    # ── Telemetry output ─────────────────────────────────────

    def _send_telemetry(self):
        t = time.time() - self.t0
        self.tick += 1

        if self.flying:
            if self.alt < self.target_alt:
                self.alt = min(self.target_alt, self.alt + 0.25)
            self.roll = 3.0 * math.sin(t * 0.7)
            self.pitch = 2.0 * math.cos(t * 0.5)
            self.yaw = (t * 15.0) % 360.0
            self.heading = self.yaw
            self.groundspeed = 3.0
            self.current = 8.0
        elif self.armed:
            self.groundspeed = 0.0
            self.current = 4.0
        else:
            self.groundspeed = 0.0
            self.current = 3.0

        # Battery
        self.battery_pct = max(10, self.battery_pct - 0.001)
        self.voltage = 10.5 + self.battery_pct / 100.0 * 2.1

        now_ms = int(t * 1000)

        # Heartbeat (1 Hz)
        if self.tick % 10 == 0:
            base_mode = 0
            if self.armed:
                base_mode |= MAV_MODE_FLAG_SAFETY_ARMED
            custom_mode = 4 if self.flying else (1 if self.armed else 0)
            self.mav.heartbeat_send(
                mavlink.MAV_TYPE_QUADROTOR,
                mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                base_mode, custom_mode,
                mavlink.MAV_STATE_ACTIVE if self.armed else mavlink.MAV_STATE_STANDBY,
            )

        # Attitude (5 Hz)
        if self.tick % 2 == 0:
            self.mav.attitude_send(
                now_ms, self.roll, self.pitch, self.yaw,
                0.0, 0.0, 0.0,
            )

        # Global position (2 Hz)
        if self.tick % 5 == 0:
            self.mav.global_position_int_send(
                now_ms, self.lat, self.lon,
                int(self.alt * 1000), int(self.alt * 1000),
                0, 0, 0,
                int(self.heading * 100),
            )

        # VFR_HUD (2 Hz)
        if self.tick % 5 == 0:
            self.mav.vfr_hud_send(
                self.groundspeed, self.groundspeed,
                int(self.heading * 100),  # centidegrees (int16)
                int(60 if self.flying else 0),  # throttle (uint8)
                self.alt,
                0.5 if self.flying else 0.0,
            )

        # Battery (1 Hz)
        if self.tick % 10 == 0:
            self.mav.sys_status_send(
                0b111111111111111111, 0b111111111111111111, 0,
                300, int(self.voltage * 1000), int(self.current * 100),
                int(self.battery_pct), 0, 0, 0, 0, 0, 0,
            )

    # ── Keyboard input ───────────────────────────────────────

    def _input_loop(self):
        print("\n  [a]RM  [d]ISARM  [t]AKEOFF  [l]AND  [r]TL  [q]UIT")
        while self.running:
            try:
                r, _, _ = select.select([sys.stdin], [], [], 0.5)
                if not r:
                    continue
                ch = sys.stdin.readline().strip().lower()
                if not ch:
                    continue
                if ch == 'a':
                    self.armed = True
                    self.flying = False
                    print("  >>> ARMED")
                elif ch == 'd':
                    self.armed = False
                    self.flying = False
                    print("  >>> DISARMED")
                elif ch == 't':
                    if not self.armed:
                        print("  [WARN] Not armed!")
                        continue
                    self.flying = True
                    self.target_alt = 5.0
                    print("  >>> TAKEOFF to 5m")
                elif ch == 'l':
                    self.flying = False
                    self.armed = False
                    self.target_alt = 0.0
                    print("  >>> LANDING")
                elif ch == 'r':
                    self.flying = False
                    self.armed = False
                    self.target_alt = 0.0
                    print("  >>> RTL")
                elif ch == 'q':
                    self.running = False
                    break
            except Exception:
                pass

    # ── Main loop ────────────────────────────────────────────

    def run(self):
        print(f"\n{'='*60}")
        print(f"  Virtual ArduPilot FC")
        print(f"  PTY: {self.slave_path}")
        print(f"  Use: uart_device:={self.slave_path}")
        print(f"{'='*60}")

        # Write PTY path to temp file for automated scripts
        try:
            with open('/tmp/virtual_fc_pty.txt', 'w') as f:
                f.write(self.slave_path)
        except Exception:
            pass

        self.start()

        while self.running:
            self._send_telemetry()
            time.sleep(0.1)

        self._cleanup()

    def _cleanup(self):
        self.running = False
        try:
            os.close(self.master_fd)
        except OSError:
            pass
        try:
            os.close(self.slave_fd)
        except OSError:
            pass
        print("\n[FC] Stopped.")


def main():
    fc = VirtualFC()
    signal.signal(signal.SIGINT, lambda *_: setattr(fc, 'running', False))
    signal.signal(signal.SIGTERM, lambda *_: setattr(fc, 'running', False))
    fc.run()


if __name__ == '__main__':
    main()
