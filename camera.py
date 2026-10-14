"""
Camera wrapper so Picamera2 and a USB webcam look the same to the rest of
the program. read() always returns a BGR frame, or None.

Lifted almost unchanged from hand_centering.py - it already worked.
Also runs on a Mac with the built-in webcam (for testing off the robot).
"""

import sys
import time

import cv2

IS_MAC = sys.platform == "darwin"
IS_LINUX = sys.platform.startswith("linux")


class Camera:
    def __init__(self, source="auto", width=640, height=480, usb_index=0):
        self.picam = None
        self.cap = None
        self.width = width
        self.height = height

        # Picamera2 only exists on a Pi, so "auto" skips straight to the
        # webcam anywhere else.
        if source == "picam" or (source == "auto" and IS_LINUX):
            try:
                from picamera2 import Picamera2
                self.picam = Picamera2()
                # Picamera2's "RGB888" is laid out as BGR in memory, so it is
                # already what OpenCV expects.
                config = self.picam.create_preview_configuration(
                    main={"size": (width, height), "format": "RGB888"}
                )
                self.picam.configure(config)
                self.picam.start()
                time.sleep(1.0)            # let auto-exposure settle
                print("Camera: Raspberry Pi camera (Picamera2)")
                return
            except Exception as e:
                self.picam = None
                if source == "picam":
                    sys.exit(f"Could not open Pi camera: {e}")
                print(f"Pi camera unavailable ({e}); trying USB camera")

        if IS_LINUX:
            self.cap = cv2.VideoCapture(usb_index, cv2.CAP_V4L2)
        elif IS_MAC:
            self.cap = cv2.VideoCapture(usb_index, cv2.CAP_AVFOUNDATION)
        else:
            self.cap = cv2.VideoCapture(usb_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)     # keep latency low
        if not self.cap.isOpened() or not self._warm_up():
            hint = ""
            if IS_MAC:
                hint = ("\nOn a Mac: allow camera access for the app running "
                        "Python (Terminal, iTerm or VS Code) in System Settings > "
                        "Privacy & Security > Camera, then restart that app. "
                        "If an iPhone is the default camera (Continuity Camera), "
                        "try --usb-index 1.")
            sys.exit(f"Could not open USB camera at index {usb_index}{hint}")
        print(f"Camera: USB camera (index {usb_index})")

    def _warm_up(self, seconds=3.0):
        """Webcams (the Mac's especially) can open fine and then return
        nothing for the first moment, or forever if permission was denied.
        Wait for one real frame so that shows up now, not as a blank panel."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            ok, frame = self.cap.read()
            if ok and frame is not None:
                return True
            time.sleep(0.1)
        return False

    def read(self):
        if self.picam is not None:
            frame = self.picam.capture_array()
        else:
            ok, frame = self.cap.read()
            if not ok:
                return None
        if frame is not None and frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        # Some webcams ignore the requested size (Macs often give 1280x720 or
        # larger). Scale down to --width, keeping the aspect ratio so balls
        # stay round, so detection runs at the speed it would on the Pi.
        if frame is not None and frame.shape[1] > self.width:
            h = round(frame.shape[0] * self.width / frame.shape[1])
            frame = cv2.resize(frame, (self.width, h), interpolation=cv2.INTER_AREA)
        return frame

    def release(self):
        if self.picam is not None:
            self.picam.stop()
        if self.cap is not None:
            self.cap.release()
