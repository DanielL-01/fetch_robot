"""
Control: turn "where the target is" into "how the robot should move".

Strafe only. The ESP32 sketch ignores w, so this never produces one - the
robot slides sideways to line up instead of turning, which also keeps the
camera pointing the same way.

Knows nothing about hands or balls. It only sees a Target.
"""


def clamp(v, limit):
    return max(-limit, min(limit, v))


class Controller:
    """Holds the smoothing state between frames."""

    def __init__(self):
        self.smooth = None          # (cx, cy, size) after EMA

    def reset(self):
        """Call when the target is lost, so a reappearing target does not
        get blended with where it was a minute ago."""
        self.smooth = None

    def update(self, target, s):
        """
        target - a Target from a tracker, or None
        s      - settings snapshot (a plain dict)

        Returns (vx, vy, info) where info is a dict for the display.
        """
        if target is None:
            self.reset()
            return 0.0, 0.0, {"state": "no target", "ex": 0.0, "ey": 0.0, "ed": 0.0,
                              "cx": 0.5, "cy": 0.5, "size": 0.0}

        # --- smooth the raw detection (exponential moving average) ---
        a = s["smoothing"]
        if self.smooth is None:
            self.smooth = (target.cx, target.cy, target.size)
        else:
            px, py, ps = self.smooth
            self.smooth = (a * px + (1 - a) * target.cx,
                           a * py + (1 - a) * target.cy,
                           a * ps + (1 - a) * target.size)
        cx, cy, size = self.smooth

        # --- errors, normalised to -1..+1 across the frame ---
        ex = (cx - 0.5) * 2.0       # + = target is right of centre
        ey = (0.5 - cy) * 2.0       # + = target is above centre

        # Which image axis means "sideways" and which means "forward". Only
        # the up mount can swap them: if the camera is turned 90 degrees on
        # the robot, image left/right is the robot's forward/back.
        right, fwd = ex, ey
        if s["mount"] == "up" and s["swap_axes"]:
            right, fwd = ey, ex

        # --- sideways: always a strafe ---
        if s["invert_x"]:
            right = -right
        vy = 0.0 if abs(right) < s["deadzone"] else clamp(-s["k_lin"] * right, s["max_v"])
        # vy is positive-LEFT, so a target on the right needs a negative vy.

        # --- forward/back: depends on how the camera is mounted ---
        if s["mount"] == "up":
            # Camera points at the ceiling, so the image is a map of the
            # floor: a target high in the frame is simply further forward.
            if s["invert_y"]:
                fwd = -fwd
            vx = 0.0 if abs(fwd) < s["deadzone"] else clamp(s["k_lin"] * fwd, s["max_v"])
            ed = 0.0
        else:
            # Camera points ahead. Image y is useless here - a target higher
            # in the frame is held higher, not further away - so distance
            # comes from how big the target looks.
            target_size = s["target_size"]
            ed = (target_size - size) / target_size if size > 0 else 0.0
            if abs(ed) < s["dist_deadzone"]:
                vx = 0.0
            else:
                vx = clamp(s["k_dist"] * ed, s["max_v"])   # looks small -> too far -> forward
            if s["invert_y"]:
                vx = -vx

        return vx, vy, {"state": "tracking", "ex": ex, "ey": ey, "ed": ed,
                        "cx": cx, "cy": cy, "size": size}
