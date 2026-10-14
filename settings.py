"""
Every tunable value in one place, saved to settings.json so your tuning
survives a restart.

Anything in DEFAULTS automatically appears in the web panel's /api/settings,
so adding a new knob here is usually all that is needed.
"""

import copy
import json
import os
import threading

SETTINGS_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "settings.json")

DEFAULTS = {
    # ---- what we are tracking ----
    "tracker": "hand",            # hand | tennis | pingpong

    # ---- camera mounting ----
    # forward = camera faces ahead, distance comes from apparent target size
    # up      = camera faces the ceiling, image y is forward/back
    "mount": "forward",
    "invert_x": False,            # sideways correction goes the wrong way
    "invert_y": False,            # forward/back correction goes the wrong way
    "mirror": False,              # flip the image before detection
    "swap_axes": False,           # up mount only: image x is forward, image y is sideways

    # ---- control ----
    "k_lin": 0.40,                # m/s per unit of sideways error
    "k_dist": 0.60,               # m/s per unit of relative size error
    "max_v": 0.30,                # m/s clamp on vx and vy
    "deadzone": 0.08,             # sideways error below this -> vy = 0
    "dist_deadzone": 0.15,        # within +/-15% of target size -> vx = 0
    "target_size": 0.22,          # wanted target size as a fraction of frame width
    "smoothing": 0.4,             # EMA on the detection (0 = none, 0.9 = heavy)

    # ---- detection ----
    "min_conf": 0.5,              # hand tracker only
    "min_radius_px": 8,           # ball trackers: ignore blobs smaller than this

    # ---- HSV colour ranges for the ball trackers (OpenCV H is 0-179) ----
    "tennis_hsv": {"h_lo": 25, "h_hi": 45, "s_lo": 70, "s_hi": 255, "v_lo": 60, "v_hi": 255},
    "pingpong_hsv": {"h_lo": 5, "h_hi": 20, "s_lo": 120, "s_hi": 255, "v_lo": 120, "v_hi": 255},

    # ---- view ----
    "preview": True,              # stream video to the browser (costs FPS)
    "show_mask": False,           # show the HSV mask instead of the camera image
}


class Settings:
    """Thread-safe settings store. The control loop reads it, the web app writes it."""

    def __init__(self):
        self._lock = threading.Lock()
        self._data = copy.deepcopy(DEFAULTS)
        self.load()

    def snapshot(self):
        """A copy safe to read without holding the lock."""
        with self._lock:
            return copy.deepcopy(self._data)

    def get(self, key):
        with self._lock:
            return copy.deepcopy(self._data.get(key))

    def update(self, changes):
        """Merge a dict of changes. Nested dicts (the HSV blocks) merge one level deep."""
        with self._lock:
            for k, v in changes.items():
                if k not in self._data:
                    continue                       # ignore unknown keys
                if isinstance(self._data[k], dict) and isinstance(v, dict):
                    self._data[k].update(v)
                else:
                    self._data[k] = v
            return copy.deepcopy(self._data)

    def reset(self):
        with self._lock:
            self._data = copy.deepcopy(DEFAULTS)
            return copy.deepcopy(self._data)

    def save(self):
        with self._lock:
            data = copy.deepcopy(self._data)
        with open(SETTINGS_PATH, "w") as f:
            json.dump(data, f, indent=2)
        return SETTINGS_PATH

    def load(self):
        if not os.path.exists(SETTINGS_PATH):
            return False
        try:
            with open(SETTINGS_PATH) as f:
                saved = json.load(f)
        except (OSError, ValueError) as e:
            print(f"settings.json unreadable ({e}); using defaults")
            return False
        self.update(saved)
        return True
