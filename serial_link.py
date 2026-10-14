"""
UART link from the Raspberry Pi to the ESP32 base controller
(bin_control/sketch_sep18a/sketch_sep18a.ino).

Protocol (must match the ESP32 sketch):
  Pi -> ESP32   V,<vx>,<vy>,<w>*<CS>\n        m/s, m/s, rad/s
                CS = XOR of the bytes after 'V' up to '*', 2 hex digits
  ESP32 -> Pi   T,<vx>,<vy>,<w>,<vbat>,<flags>*<CS>\n
                CS = XOR of every byte before '*', including the 'T'

Commands are sent from a background thread at a fixed rate, so the ESP32's
200 ms watchdog stays fed even when a camera frame is slow. If the main loop
stops updating the command (crash, hang), the sender switches to zeros after
STALE_AFTER seconds, so the robot still stops.
"""

import threading
import time

import serial  # pyserial

FLAG_TIMEOUT = 0x01
FLAG_BADPARSE = 0x02
FLAG_SATURATED = 0x04
FLAG_LOWBATT = 0x08
FLAG_NAMES = {
    FLAG_TIMEOUT: "TIMEOUT",
    FLAG_BADPARSE: "BADPARSE",
    FLAG_SATURATED: "SAT",
    FLAG_LOWBATT: "LOWBATT",
}


def xor_checksum(text):
    c = 0
    for b in text.encode("ascii"):
        c ^= b
    return c


def format_command(vx, vy, w):
    body = f",{vx:.3f},{vy:.3f},{w:.3f}"
    return f"V{body}*{xor_checksum(body):02X}\n"


def parse_telemetry(line):
    """Return dict for a valid 'T,...*CS' line, else None."""
    if not line.startswith("T,") or "*" not in line:
        return None
    body, cs = line.rsplit("*", 1)
    try:
        if int(cs, 16) != xor_checksum(body):
            return None
        _, vx, vy, w, vbat, flags = body.split(",")
        return {"vx": float(vx), "vy": float(vy), "w": float(w),
                "vbat": float(vbat), "flags": int(flags)}
    except ValueError:
        return None


def flags_to_text(flags):
    names = [n for bit, n in FLAG_NAMES.items() if flags & bit]
    return "|".join(names) if names else "OK"


class RobotLink:
    SEND_HZ = 30          # well above the ESP32's 200 ms watchdog
    STALE_AFTER = 0.5     # s without set_command() -> send zeros

    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0.05, write_timeout=0.1)
        self._lock = threading.Lock()
        self._cmd = (0.0, 0.0, 0.0)
        self._cmd_time = 0.0
        self.telemetry = None       # last valid telemetry dict
        self.telemetry_time = 0.0
        self.sent = 0
        self.bad_rx = 0
        self._running = True
        self._tx = threading.Thread(target=self._tx_loop, daemon=True)
        self._rx = threading.Thread(target=self._rx_loop, daemon=True)
        self._tx.start()
        self._rx.start()

    def set_command(self, vx, vy, w):
        with self._lock:
            self._cmd = (vx, vy, w)
            self._cmd_time = time.monotonic()

    def esp_alive(self, max_age=0.5):
        """True if telemetry arrived recently (ESP32 powered and wired)."""
        return self.telemetry is not None and time.monotonic() - self.telemetry_time < max_age

    def _tx_loop(self):
        period = 1.0 / self.SEND_HZ
        next_t = time.monotonic()
        while self._running:
            with self._lock:
                cmd = self._cmd
                if time.monotonic() - self._cmd_time > self.STALE_AFTER:
                    cmd = (0.0, 0.0, 0.0)
            try:
                self.ser.write(format_command(*cmd).encode("ascii"))
                self.sent += 1
            except serial.SerialException:
                pass
            next_t += period
            time.sleep(max(0.0, next_t - time.monotonic()))

    def _rx_loop(self):
        while self._running:
            try:
                raw = self.ser.readline()
            except serial.SerialException:
                time.sleep(0.1)
                continue
            if not raw:
                continue
            line = raw.decode("ascii", errors="replace").strip()
            t = parse_telemetry(line)
            if t is None:
                self.bad_rx += 1
            else:
                self.telemetry = t
                self.telemetry_time = time.monotonic()

    def close(self):
        """Stop the threads and tell the robot to stop."""
        self._running = False
        self._tx.join(timeout=0.5)
        try:
            for _ in range(3):
                self.ser.write(format_command(0.0, 0.0, 0.0).encode("ascii"))
            self.ser.flush()
        except serial.SerialException:
            pass
        self._rx.join(timeout=0.5)
        self.ser.close()
