# Robot control panel

Web panel for the mecanum robot. Pick what it follows, tune everything live
from a laptop, arm and stop it.

## Run

    python3 app.py --serial /dev/serial0

Then open `http://<pi-ip>:8000` from a laptop on the same network.
Find the Pi's address with `hostname -I`.

Leave `--serial` off to test the vision with the motors disconnected.

Other options: `--source usb|picam|auto`, `--width`, `--height`, `--port`.
Everything else is changed in the panel, not on the command line.

## Testing on a Mac

The panel runs on a Mac with the built-in webcam, so the vision and the
tuning can be tried before anything goes on the Pi.

    python3 -m venv .venv
    source .venv/bin/activate
    pip install -r requirements.txt
    python3 app.py

Then open `http://localhost:8000`.

- The first run asks for camera access. If it was refused, or the program
  says it cannot open the camera, allow the app you launched it from
  (Terminal, iTerm or VS Code) in **System Settings > Privacy & Security >
  Camera**, then quit and reopen that app.
- If an iPhone gets picked as the camera (Continuity Camera), add
  `--usb-index 1`.
- A laptop webcam faces you, so tick **mirror** in the panel to make the
  left/right correction read naturally.
- Leave `--serial` off and nothing moves. The arm and stop buttons, the
  heartbeat and the vx/vy readout all still work, so the whole control
  logic can be checked. To drive the ESP32 over USB from the Mac instead,
  find its port with `ls /dev/cu.*` and pass e.g.
  `--serial /dev/cu.usbserial-0001`.
- If port 8000 is taken, use `--port 8080`.

## Safety

Three layers, outermost first:

1. **Arm.** Starts disarmed every time. While disarmed the detector runs and
   the panel updates, but the robot is sent zeros.
2. **Heartbeat.** The browser polls twice a second. If those polls stop for
   2 seconds (tab closed, laptop asleep, Wi-Fi dropped) the robot disarms.
3. **ESP32 watchdog.** 200 ms without a command and the motors stop, which
   covers this program crashing outright.

The STOP button disarms immediately.

## Tuning a ball

1. Pick the ball in **Target**.
2. Tick **mask view**. The image becomes black and white.
3. Adjust the HSV sliders until only the ball is white.
4. Untick mask view, check the circle tracks the ball, then **Save**.

Hue is the colour itself, saturation how vivid, value how bright. Widen
saturation and value first; narrow the hue range to reject other objects.

Tennis balls are easy. Orange ping pong balls are fine. **White ping pong
balls are unreliable** — white matches walls, paper and skin highlights.

## Calibrating the control

Do it in this order. Each step depends on the one before.

**1. Motors alone.** No camera, wheels off the ground:

    python3 uart_test.py --sequence --port /dev/serial0

Check that every direction matches its label. If one is wrong, fix it in the
wiring or the ESP32 sketch, not the panel: the vision code assumes
`+vx = forward` and `+vy = left`. Then find the slowest speed that actually
moves the robot (`--vx 0.05`, `0.08`, `0.10` ...) and remember it.

**2. Directions.** Robot on the floor, armed, **max_v** at about 0.10 so
mistakes are slow. Set **mirror** first if you use it, because it flips the
sideways direction.

- Target to the right of the middle line: the robot should slide right. If
  it slides away, tick **invert_x**.
- Target moves away: the robot should follow. If it backs off, tick
  **invert_y**.
- Up mount only: if moving the target forward makes the robot strafe (the
  camera is turned 90° on the robot), tick **swap axes** first, then redo
  the two checks above.

**3. Distance** (forward mount). Hold the target where the robot should stop,
read **size** in the panel, and set **target_size** to it. If the robot
creeps back and forth at the stopping point, raise **dist_deadzone**.

**4. Gains.** One value at a time, in small steps.

| Setting      | Too low                       | Too high                     |
|--------------|-------------------------------|------------------------------|
| k_lin        | slow to centre, stops short   | overshoots, wobbles sideways |
| k_dist       | lazy following                | lunges, rocks forward/back   |
| max_v        | can't keep up                 | fast mistakes                |
| deadzone     | twitches when nearly centred  | stops off-centre             |
| smoothing    | jittery (hand tracking)       | lags, then overshoots        |

Raise a gain until the robot starts to overshoot, then back it off by about
a third. The robot can't move slower than the minimum speed from step 1: if
small corrections do nothing, raise the gain or widen the deadzone.

**5. Re-check on the Pi.** Detection is slower there than on a laptop, so
tuning done on a Mac can wobble on the Pi. Lower the gains a little or raise
**smoothing**, and turn **preview** off for more FPS.

On a Mac you can already set **target_size**, **smoothing**, **deadzone**
and the ball colours, and check the vx/vy readout points the right way.
Everything else needs the robot.

Press **Save** when done.

## Settings

Saved to `settings.json` by the Save button and loaded on startup.
Delete that file to go back to defaults.

## Files

    app.py          web server, control loop, arming
    trackers.py     hand and colour-ball detectors
    control.py      target position -> vx, vy
    camera.py       Picamera2 / USB webcam (or Mac webcam) wrapper
    settings.py     defaults and settings.json
    serial_link.py  UART to the ESP32 (unchanged)
    templates/      the panel UI

`uart_test.py` from the old project still works for motor testing without
the camera.

## Notes

- Rotation is not used. The ESP32 sketch ignores `w`, so the robot strafes
  sideways to line up instead of turning. `w` is always sent as 0.
- Turning the video preview off stops JPEG encoding entirely and gives the
  detector the whole CPU. Worth doing once it is tuned.
- Hand tracking is much slower than colour tracking on a Pi.
