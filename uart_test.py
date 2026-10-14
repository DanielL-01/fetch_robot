#!/usr/bin/env python3
"""
Send a fixed velocity to the ESP32, with no camera involved.

Use this to calibrate the motors before touching the vision side:
check which way each wheel spins, find the duty where the robot starts
moving, and confirm the UART link works at all.

PUT THE WHEELS OFF THE GROUND FIRST.

Examples:
    python3 uart_test.py --vx 0.15              # forward 2 s
    python3 uart_test.py --vy 0.15 -t 3         # strafe left 3 s
    python3 uart_test.py --w 0.8                # turn anticlockwise
    python3 uart_test.py --sequence             # fwd, back, left, right, turn
    python3 uart_test.py --listen               # just watch telemetry
"""

import argparse
import time

from serial_link import RobotLink, flags_to_text

SEQUENCE = [
    ("forward   (vx +)", 0.15, 0.0, 0.0),
    ("backward  (vx -)", -0.15, 0.0, 0.0),
    ("strafe L  (vy +)", 0.0, 0.15, 0.0),
    ("strafe R  (vy -)", 0.0, -0.15, 0.0),
    ("turn CCW  (w  +)", 0.0, 0.0, 0.8),
    ("turn CW   (w  -)", 0.0, 0.0, -0.8),
]


def hold(link, label, vx, vy, w, seconds):
    print(f"\n{label}: vx={vx:+.2f} vy={vy:+.2f} w={w:+.2f} for {seconds:.1f}s")
    link.set_command(vx, vy, w)
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        t = link.telemetry
        state = (f"vbat {t['vbat']:.1f}V  {flags_to_text(t['flags'])}"
                 if link.esp_alive() else "no telemetry from ESP32")
        print(f"\r  tx {link.sent:5d}  {state:40s}", end="", flush=True)
        time.sleep(0.1)
    link.set_command(0.0, 0.0, 0.0)
    print()


def main():
    p = argparse.ArgumentParser(description="Send a fixed velocity to the ESP32")
    p.add_argument("--port", default="/dev/serial0")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--vx", type=float, default=0.0, help="m/s, + forward")
    p.add_argument("--vy", type=float, default=0.0, help="m/s, + left")
    p.add_argument("--w", type=float, default=0.0, help="rad/s, + anticlockwise")
    p.add_argument("-t", "--seconds", type=float, default=2.0)
    p.add_argument("--sequence", action="store_true", help="run all six directions in turn")
    p.add_argument("--listen", action="store_true", help="send zeros, just print telemetry")
    args = p.parse_args()

    link = RobotLink(args.port, args.baud)
    print(f"{args.port} @ {args.baud}. Ctrl+C to stop.")
    try:
        if args.listen:
            hold(link, "idle (zeros)", 0.0, 0.0, 0.0, 1e9)
        elif args.sequence:
            print("Wheels off the ground! Watch which way each one spins.")
            for label, vx, vy, w in SEQUENCE:
                hold(link, label, vx, vy, w, args.seconds)
                time.sleep(0.5)
        else:
            hold(link, "manual", args.vx, args.vy, args.w, args.seconds)
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        link.close()   # sends a stop
        print("stopped.")


if __name__ == "__main__":
    main()
