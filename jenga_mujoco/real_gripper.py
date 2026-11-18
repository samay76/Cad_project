"""
real_gripper.py - Robotiq 2F-85 on the laptop's USB/RS-485 adapter (Modbus RTU), for Real Sim (real_robot.py).

Robotiq Modbus RTU protocol:
  status  = FC04 read of 3 input registers at 0x07D0
  command = FC16 write of 3 output registers at 0x03E8: rACTION, 0, 0, position, speed, force
Position 0 = open .. 255 = closed. The fingers stop on an object at the set force.

Commands are non-blocking: send one, then poll done() while the sim mirrors the fingers.
FakeGripper stands in for it when the robot ip is 'fake'.
"""
import struct
import time

STATUS_REG, STATUS_COUNT = 0x07D0, 3
CMD_REG = 0x03E8
STROKE_MM = 85.0


def crc16_modbus(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return struct.pack("<H", crc)


def width_to_pos(width_mm):
    """Finger opening (mm) -> 2F-85 position command (0 open .. 255 closed), linear over the 85 mm stroke."""
    return int(round(max(0.0, min(255.0, 255.0 * (1.0 - float(width_mm) / STROKE_MM)))))


def pos_to_width(pos):
    return STROKE_MM * (1.0 - float(pos) / 255.0)


class RealGripper:
    def __init__(self, port="/dev/ttyUSB0", baud=115200, slave=9):
        import serial
        self.slave = slave
        try:
            self.ser = serial.Serial(port, baud, bytesize=8, parity="N", stopbits=1, timeout=0.2)
        except serial.SerialException as e:
            hint = (" Permission denied: run  sudo chmod a+rw " + port + "  (or add yourself to the dialout group "
                    "and log in again).") if "ermission" in str(e) else " Is the gripper's USB adapter plugged in?"
            raise RuntimeError(f"cannot open the gripper port {port}: {e}.{hint}") from e

    def _status_bytes(self):
        req = struct.pack(">BBHH", self.slave, 0x04, STATUS_REG, STATUS_COUNT)
        req += crc16_modbus(req)
        self.ser.reset_input_buffer()
        self.ser.write(req)
        expected = 3 + 2 * STATUS_COUNT + 2
        resp = self.ser.read(expected)
        if len(resp) < 5:
            raise RuntimeError(f"gripper: no reply ({len(resp)} bytes) - check power, port, baud, slave id")
        if resp[1] & 0x80:
            raise RuntimeError(f"gripper: Modbus exception {resp[2]}")
        if len(resp) != expected or crc16_modbus(resp[:-2]) != resp[-2:]:
            raise RuntimeError(f"gripper: bad reply {resp.hex(' ')}")
        return resp[3:3 + 2 * STATUS_COUNT]

    def status(self):
        gstatus, _r, gflt, gpr, gpo, gcu = self._status_bytes()
        return {"activated": bool(gstatus & 0x01), "act_status": (gstatus >> 4) & 0x03,
                "object": (gstatus >> 6) & 0x03, "fault": gflt, "pos_request": gpr, "position": gpo,
                "current_mA": gcu * 10}

    def _write(self, action, pos=0, speed=0, force=0):
        data = bytes([action, 0, 0, pos & 0xFF, speed & 0xFF, force & 0xFF])
        req = struct.pack(">BBHHB", self.slave, 0x10, CMD_REG, 3, len(data)) + data
        req += crc16_modbus(req)
        self.ser.reset_input_buffer()
        self.ser.write(req)
        resp = self.ser.read(8)
        if len(resp) < 5:
            raise RuntimeError("gripper: no reply to a command")
        if resp[1] & 0x80:
            raise RuntimeError(f"gripper: Modbus exception {resp[2]}")

    def position(self):
        return self.status()["position"]

    def activated(self):
        return self.status()["act_status"] == 3

    def activate(self, timeout=10.0):
        """Reset (clears faults) and activate. The fingers close fully and open once."""
        self._write(0x00)
        time.sleep(0.5)
        self._write(0x01)
        t0 = time.time()
        while time.time() - t0 < timeout:
            time.sleep(0.2)
            if self.activated():
                return
        raise RuntimeError(f"gripper activation did not finish: {self.status()}")

    def command(self, pos, speed, force):
        self._write(0x09, pos, speed, force)   # rACT | rGTO

    def done(self, pos):
        """(finished, status): at the requested position, or stopped on an object."""
        st = self.status()
        if st["fault"] and st["fault"] not in (0x05, 0x09):   # 0x05/0x09: minor, cleared by the next command
            raise RuntimeError(f"gripper fault 0x{st['fault']:02X}")
        return st["pos_request"] == pos and st["object"] != 0, st

    def close(self):
        self.ser.close()


class FakeGripper:
    """Moves at the commanded speed; never detects an object."""

    def __init__(self, pos=0):
        self.pos, self.target, self.rate, self.t, self.active = float(pos), float(pos), 0.0, time.time(), False

    def _update(self):
        now = time.time()
        step = self.rate * (now - self.t)
        self.t = now
        self.pos += max(-step, min(step, self.target - self.pos))

    def status(self):
        self._update()
        return {"activated": self.active, "act_status": 3 if self.active else 0,
                "object": 3 if abs(self.pos - self.target) < 1 else 0, "fault": 0,
                "pos_request": int(self.target), "position": int(round(self.pos)), "current_mA": 0}

    def position(self):
        return self.status()["position"]

    def activated(self):
        return self.active

    def activate(self, timeout=10.0):
        self.active = True

    def command(self, pos, speed, force):
        self._update()
        self.target, self.rate = float(pos), 30.0 + 220.0 * speed / 255.0

    def done(self, pos):
        st = self.status()
        return st["object"] != 0, st

    def close(self):
        pass
