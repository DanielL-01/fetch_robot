#!/usr/bin/env python3
"""
Web control panel for the mecanum robot.

Open http://<pi-ip>:8000 from a laptop on the same network to pick what the
robot follows (hand, tennis ball, ping pong ball), tune everything live, and
arm or stop the robot.

    python3 app.py                          # vision only, no motors
    python3 app.py --serial /dev/serial0    # also drive the ESP32

THREE SAFETY LAYERS, outermost first:
  1. ARM.        Starts disarmed every time. While disarmed the detector runs
                 and the panel updates, but the robot is sent zeros.
  2. Heartbeat.  The browser polls /api/state about twice a second. If those
                 polls stop (tab closed, laptop asleep, Wi-Fi dropped) for
                 UI_TIMEOUT, the robot disarms itself.
  3. ESP32.      Its own 200 ms watchdog stops the motors if this program
                 dies outright and the serial messages stop.
"""

import argparse
import threading
import time

import cv2
from flask import Flask, Response, jsonify, render_template, request

from camera import Camera
from control import Controller
from settings import Settings
from trackers import TRACKER_INFO, make_tracker

UI_TIMEOUT = 2.0        # s without a browser poll -> disarm

app = Flask(__name__)
settings = Settings()
state_lock = threading.Lock()

# Everything the control loop publishes for the UI to read
state = {
    "armed": False,
    "tracker": settings.get("tracker"),
    "vx": 0.0, "vy": 0.0,
    "fps": 0.0,
    "detect": "starting",
    "ex": 0.0, "ed": 0.0, "size": 0.0,
    "esp": "no link",
    "note": "",
}

last_ui_ping = 0.0
latest_jpeg = None
jpeg_lock = threading.Lock()
loop = None             # the ControlLoop, created in main()


class ControlLoop(threading.Thread):
    """Grab frame -> find target -> compute command -> send to the ESP32."""

    def __init__(self, camera, link):
        super().__init__(daemon=True)
        self.camera = camera
        self.link = link
        self.controller = Controller()
        self.running = True
        self.tracker = None
        self._tracker_name = None
        self._start_time = time.monotonic()

    # ------------------------------------------------------------------
    def _ensure_tracker(self, name, s):
        """Swap the tracker if the panel changed it. Loading the hand model
        takes a moment, which is why this is not done every frame."""
        if name == self._tracker_name:
            return
        if self.tracker is not None:
            self.tracker.close()
        set_state(note=f"loading {name} tracker...")
        self.tracker = make_tracker(name, s)
        self._tracker_name = name
        self.controller.reset()
        set_state(note="")

    # ------------------------------------------------------------------
    def run(self):
        global latest_jpeg
        frames, fps_timer, fps = 0, time.monotonic(), 0.0

        while self.running:
            s = settings.snapshot()

            try:
                self._ensure_tracker(s["tracker"], s)
            except Exception as e:
                set_state(note=f"tracker failed: {e}", detect="error")
                time.sleep(1.0)
                continue

            frame = self.camera.read()
            if frame is None:
                set_state(detect="no camera frame")
                self.send(0.0, 0.0)
                time.sleep(0.1)
                continue

            if s["mirror"]:
                frame = cv2.flip(frame, 1)

            ts_ms = int((time.monotonic() - self._start_time) * 1000)
            try:
                target = self.tracker.find(frame, s, ts_ms)
            except Exception as e:
                set_state(note=f"detection error: {e}")
                target = None

            vx, vy, info = self.controller.update(target, s)

            # ---- safety gates, in order ----
            armed = get_state("armed")
            if armed and time.monotonic() - last_ui_ping > UI_TIMEOUT:
                set_armed(False, note="browser went quiet - disarmed")
                armed = False
            if not armed:
                vx = vy = 0.0

            self.send(vx, vy)

            # ---- FPS ----
            frames += 1
            now = time.monotonic()
            if now - fps_timer >= 1.0:
                fps = frames / (now - fps_timer)
                frames, fps_timer = 0, now

            set_state(vx=vx, vy=vy, fps=fps, tracker=s["tracker"],
                      detect=info["state"], ex=info["ex"], ed=info["ed"],
                      size=info["size"], esp=self.esp_status())

            # ---- preview, only if someone is watching ----
            if s["preview"]:
                view = self.render(frame, target, s, info, vx, vy)
                ok, buf = cv2.imencode(".jpg", view, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ok:
                    with jpeg_lock:
                        latest_jpeg = buf.tobytes()
            else:
                with jpeg_lock:
                    latest_jpeg = None

    # ------------------------------------------------------------------
    def render(self, frame, target, s, info, vx, vy):
        """Draw the overlay. Returns the image to stream."""
        # Mask view: see exactly what the HSV sliders are selecting.
        if s["show_mask"] and getattr(self.tracker, "mask", None) is not None:
            frame = cv2.cvtColor(self.tracker.mask, cv2.COLOR_GRAY2BGR)

        h, w = frame.shape[:2]
        cx_px, cy_px = w // 2, h // 2
        up = s["mount"] == "up"

        # Centre line and the dead zone either side of it. The up mount
        # steers on both image axes, so it gets a horizontal pair too.
        cv2.line(frame, (cx_px, 0), (cx_px, h), (255, 255, 255), 1)
        dz = int(s["deadzone"] * w / 2)
        for dx in (-dz, dz):
            cv2.line(frame, (cx_px + dx, 0), (cx_px + dx, h), (110, 110, 110), 1)
        if up:
            cv2.line(frame, (0, cy_px), (w, cy_px), (255, 255, 255), 1)
            dzy = int(s["deadzone"] * h / 2)
            for dy in (-dzy, dzy):
                cv2.line(frame, (0, cy_px + dy), (w, cy_px + dy), (110, 110, 110), 1)

        self.tracker.draw(frame, target)

        if target is not None:
            c = (int(info["cx"] * w), int(info["cy"] * h))
            colour = (0, 255, 0) if (vx == 0 and vy == 0) else (0, 165, 255)
            if up:
                # Point at the centre, but only along the image axes that
                # are outside the dead zone, i.e. that are actually moving.
                tip = (cx_px if abs(info["ex"]) >= s["deadzone"] else c[0],
                       cy_px if abs(info["ey"]) >= s["deadzone"] else c[1])
                if tip != c:
                    cv2.arrowedLine(frame, c, tip, colour, 3, tipLength=0.2)
            elif vy != 0:
                cv2.arrowedLine(frame, c, (cx_px, c[1]), colour, 3, tipLength=0.2)
            if s["mount"] == "forward":
                # Dashed-looking circle showing the size we are aiming for
                cv2.circle(frame, c, int(s["target_size"] * w / 2), (255, 255, 0), 1)
                if vx != 0:
                    cv2.putText(frame, "FWD" if vx > 0 else "BACK",
                                (c[0] - 25, c[1] - 18), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, colour, 2)

        banner = "ARMED" if get_state("armed") else "DISARMED"
        colour = (0, 0, 255) if get_state("armed") else (160, 160, 160)
        cv2.putText(frame, banner, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.8, colour, 2)
        cv2.putText(frame, f"vx {vx:+.2f}  vy {vy:+.2f}", (10, h - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        return frame

    def send(self, vx, vy):
        if self.link is not None:
            self.link.set_command(vx, vy, 0.0)      # w is always zero

    def esp_status(self):
        if self.link is None:
            return "no link"
        if not self.link.esp_alive():
            return "no reply"
        from serial_link import flags_to_text
        t = self.link.telemetry
        return f"{flags_to_text(t['flags'])}  {t['vbat']:.1f}V"

    def stop(self):
        self.running = False
        if self.tracker is not None:
            self.tracker.close()


# -------------------------------------------------------------- helpers ----

def set_state(**kw):
    with state_lock:
        state.update(kw)


def get_state(key):
    with state_lock:
        return state[key]


def set_armed(value, note=""):
    with state_lock:
        state["armed"] = bool(value)
        state["note"] = note
    if not value and loop is not None:
        loop.send(0.0, 0.0)


# ----------------------------------------------------------------- routes ----

@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/state")
def api_state():
    """Polled by the browser twice a second. Doubles as the heartbeat."""
    global last_ui_ping
    last_ui_ping = time.monotonic()
    with state_lock:
        return jsonify(dict(state))


@app.route("/api/settings", methods=["GET", "POST"])
def api_settings():
    if request.method == "POST":
        return jsonify(settings.update(request.get_json(force=True)))
    return jsonify(settings.snapshot())


@app.route("/api/trackers")
def api_trackers():
    return jsonify(TRACKER_INFO)


@app.route("/api/arm", methods=["POST"])
def api_arm():
    global last_ui_ping
    last_ui_ping = time.monotonic()         # arming counts as a heartbeat
    want = bool(request.get_json(force=True).get("armed"))
    set_armed(want, note="" if want else "stopped by user")
    return jsonify({"armed": get_state("armed")})


@app.route("/api/save", methods=["POST"])
def api_save():
    return jsonify({"saved": settings.save()})


@app.route("/api/reset", methods=["POST"])
def api_reset():
    return jsonify(settings.reset())


@app.route("/video")
def video():
    def frames():
        blank = None
        while True:
            with jpeg_lock:
                jpeg = latest_jpeg
            if jpeg is None:
                # Preview is off: send a placeholder slowly so the <img>
                # does not sit there looking broken.
                if blank is None:
                    import numpy as np
                    img = np.zeros((240, 320, 3), dtype="uint8")
                    cv2.putText(img, "preview off", (70, 130),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (90, 90, 90), 2)
                    blank = cv2.imencode(".jpg", img)[1].tobytes()
                jpeg = blank
                time.sleep(0.5)
            else:
                time.sleep(0.03)
            yield b"--f\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"

    return Response(frames(), mimetype="multipart/x-mixed-replace; boundary=f")


# ------------------------------------------------------------------- main ----

def lan_ip():
    """This machine's address on the local network, or None. Connecting a
    UDP socket sends nothing; it just makes the OS pick the outgoing interface."""
    import socket
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect(("10.255.255.255", 1))
            return sock.getsockname()[0]
    except OSError:
        return None


def main():
    global loop
    p = argparse.ArgumentParser(description="Robot web control panel")
    p.add_argument("--serial", metavar="PORT",
                   help="ESP32 serial port, e.g. /dev/serial0 on a Pi or "
                        "/dev/cu.usbserial-XXXX on a Mac. Omit to run vision only.")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--source", choices=["auto", "picam", "usb"], default="auto")
    p.add_argument("--usb-index", type=int, default=0)
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--port", type=int, default=8000, help="web server port")
    args = p.parse_args()

    link = None
    if args.serial:
        from serial_link import RobotLink
        link = RobotLink(args.serial, args.baud)
        print(f"ESP32 link on {args.serial} @ {args.baud}")
    else:
        print("No --serial given: vision only, the robot will not move.")

    camera = Camera(args.source, args.width, args.height, args.usb_index)
    loop = ControlLoop(camera, link)
    loop.start()

    print(f"\nControl panel: http://localhost:{args.port}")
    ip = lan_ip()
    if ip:
        print(f"  from another device: http://{ip}:{args.port}")
    print()
    try:
        app.run(host="0.0.0.0", port=args.port, threaded=True,
                debug=False, use_reloader=False)
    finally:
        loop.stop()
        time.sleep(0.2)
        if link is not None:
            link.close()        # sends a final stop
        camera.release()
        print("stopped.")


if __name__ == "__main__":
    main()
