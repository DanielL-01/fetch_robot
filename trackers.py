"""
Trackers: three ways to find a target, one common answer.

Every tracker returns a Target (or None), so the control code in control.py
never needs to know whether it is following a hand or a ball:

    Target.cx, Target.cy   centre, 0..1 across the frame
    Target.size            apparent size as a fraction of frame WIDTH
                           (bigger = closer)

Adding a fourth target later means writing one more class with a find()
method and adding it to make_tracker().
"""

import math
import os
import subprocess
import urllib.error
import urllib.request

import cv2
import numpy as np

MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
    "hand_landmarker/float16/latest/hand_landmarker.task"
)
MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hand_landmarker.task")

HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (17, 18), (18, 19), (19, 20),
    (0, 17),
]
PALM_POINTS = [0, 5, 9, 13, 17]     # wrist + knuckles: stable when fingers bend


class Target:
    """What every tracker returns. Units are fractions of the frame."""

    __slots__ = ("cx", "cy", "size", "landmarks", "radius_px")

    def __init__(self, cx, cy, size, landmarks=None, radius_px=0.0):
        self.cx = cx
        self.cy = cy
        self.size = size
        self.landmarks = landmarks      # hand tracker only, for drawing
        self.radius_px = radius_px      # ball trackers only, for drawing


# ---------------------------------------------------------------- hand ----

class HandTracker:
    name = "hand"
    label = "Hand"
    uses_hsv = False

    def __init__(self, min_conf=0.5):
        self._ensure_model()
        import mediapipe as mp
        from mediapipe.tasks import python as mp_python
        from mediapipe.tasks.python import vision

        self._mp = mp
        options = vision.HandLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=MODEL_PATH),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=1,
            min_hand_detection_confidence=min_conf,
            min_hand_presence_confidence=min_conf,
            min_tracking_confidence=min_conf,
        )
        self._landmarker = vision.HandLandmarker.create_from_options(options)
        self._last_ts = -1
        self.mask = None                # hand tracker has no mask view

    @staticmethod
    def _ensure_model():
        if os.path.exists(MODEL_PATH):
            return
        print(f"Downloading hand landmark model to {MODEL_PATH} ...")
        tmp = MODEL_PATH + ".part"          # so a failed download leaves nothing behind
        try:
            urllib.request.urlretrieve(MODEL_URL, tmp)
        except urllib.error.URLError as e:
            # python.org's Python on macOS has no root certificates until
            # "Install Certificates.command" is run, so HTTPS fails. The
            # system curl uses the Mac's own certificates.
            print(f"Python download failed ({e.reason}); trying curl")
            subprocess.run(["curl", "-fsSL", "-o", tmp, MODEL_URL], check=True)
        os.replace(tmp, MODEL_PATH)
        print("Done.")

    def find(self, frame_bgr, settings, ts_ms):
        h, w = frame_bgr.shape[:2]
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        image = self._mp.Image(image_format=self._mp.ImageFormat.SRGB, data=rgb)

        # VIDEO mode demands strictly increasing timestamps
        ts_ms = max(ts_ms, self._last_ts + 1)
        self._last_ts = ts_ms

        result = self._landmarker.detect_for_video(image, ts_ms)
        if not result.hand_landmarks:
            return None

        lm = result.hand_landmarks[0]
        cx = sum(lm[i].x for i in PALM_POINTS) / len(PALM_POINTS)
        cy = sum(lm[i].y for i in PALM_POINTS) / len(PALM_POINTS)

        # Average two palm spans so a bent or rotated hand is less jumpy
        def span(a, b):
            dx = (lm[a].x - lm[b].x) * w
            dy = (lm[a].y - lm[b].y) * h
            return math.hypot(dx, dy)

        size = (span(0, 9) + span(5, 17)) / 2.0 / w
        return Target(cx, cy, size, landmarks=lm)

    def draw(self, frame, target):
        if target is None or target.landmarks is None:
            return
        h, w = frame.shape[:2]
        pts = [(int(p.x * w), int(p.y * h)) for p in target.landmarks]
        for a, b in HAND_CONNECTIONS:
            cv2.line(frame, pts[a], pts[b], (0, 180, 0), 1)
        for p in pts:
            cv2.circle(frame, p, 3, (0, 0, 255), -1)

    def close(self):
        self._landmarker.close()


# ---------------------------------------------------------------- balls ----

class ColourBallTracker:
    """Finds the largest blob within an HSV range and fits a circle to it.

    Works for any strongly coloured ball. Tennis balls are easy (that
    yellow-green is almost unique indoors). ORANGE ping pong balls are fine.
    WHITE ping pong balls are genuinely hard by colour - white matches walls,
    paper, and skin highlights - so use an orange one if you can.
    """

    uses_hsv = True

    def __init__(self, name, label, hsv_key):
        self.name = name
        self.label = label
        self.hsv_key = hsv_key          # which settings block holds our range
        self.mask = None                # last binary mask, for the mask view
        self._kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    def find(self, frame_bgr, settings, ts_ms):
        h, w = frame_bgr.shape[:2]
        rng = settings[self.hsv_key]

        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        lo = np.array([rng["h_lo"], rng["s_lo"], rng["v_lo"]], dtype=np.uint8)
        hi = np.array([rng["h_hi"], rng["s_hi"], rng["v_hi"]], dtype=np.uint8)

        if rng["h_lo"] <= rng["h_hi"]:
            mask = cv2.inRange(hsv, lo, hi)
        else:
            # Hue wraps at 180 (red), so a range like 170..10 needs two passes
            lo1 = np.array([0, rng["s_lo"], rng["v_lo"]], dtype=np.uint8)
            hi1 = np.array([rng["h_hi"], rng["s_hi"], rng["v_hi"]], dtype=np.uint8)
            lo2 = np.array([rng["h_lo"], rng["s_lo"], rng["v_lo"]], dtype=np.uint8)
            hi2 = np.array([179, rng["s_hi"], rng["v_hi"]], dtype=np.uint8)
            mask = cv2.bitwise_or(cv2.inRange(hsv, lo1, hi1), cv2.inRange(hsv, lo2, hi2))

        # Open then close: removes speckle, then fills small holes
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel, iterations=2)
        self.mask = mask

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        biggest = max(contours, key=cv2.contourArea)
        (x, y), radius = cv2.minEnclosingCircle(biggest)
        if radius < settings["min_radius_px"]:
            return None

        # Reject shapes that are not round: a ball fills most of its circle,
        # a stray patch of carpet usually does not.
        circle_area = math.pi * radius * radius
        if circle_area > 0 and cv2.contourArea(biggest) / circle_area < 0.55:
            return None

        # size uses DIAMETER / width so it means the same thing as the hand's
        # palm span: a fraction of the frame width.
        return Target(x / w, y / h, (2.0 * radius) / w, radius_px=radius)

    def draw(self, frame, target):
        if target is None:
            return
        h, w = frame.shape[:2]
        c = (int(target.cx * w), int(target.cy * h))
        cv2.circle(frame, c, int(target.radius_px), (0, 255, 255), 2)
        cv2.circle(frame, c, 3, (0, 0, 255), -1)

    def close(self):
        pass


# ---------------------------------------------------------------- factory ----

TRACKER_INFO = [
    {"name": "hand", "label": "Hand", "uses_hsv": False},
    {"name": "tennis", "label": "Tennis ball", "uses_hsv": True},
    {"name": "pingpong", "label": "Ping pong ball", "uses_hsv": True},
]


def make_tracker(name, settings):
    if name == "hand":
        return HandTracker(min_conf=settings["min_conf"])
    if name == "tennis":
        return ColourBallTracker("tennis", "Tennis ball", "tennis_hsv")
    if name == "pingpong":
        return ColourBallTracker("pingpong", "Ping pong ball", "pingpong_hsv")
    raise ValueError(f"unknown tracker: {name}")
