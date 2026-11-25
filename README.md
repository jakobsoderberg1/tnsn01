# tnsn01

## Debug dashboard

A local web page that talks to the robot over the USB cable (Web Serial).

1. Flash `maze_robot/maze_robot.ino`.
2. Run `dashboard/serve.sh` and open http://localhost:8000 in Chrome/Chromium/Edge (Firefox has no Web Serial).
3. Click **Connect** and pick the Leonardo (`/dev/ttyACM0`). Close the Arduino Serial Monitor first, because only one program can have the port open.

On Linux you may need `sudo usermod -aG dialout $USER` (then log out and back in) to get access to the port.

What it shows: raw sensor values with thresholds, the black pattern and line position, the wheel PWM
(commanded, since there are no encoders), a 10 s chart, and route progress (current node, next node, items).

What it controls:
- **Mode**: maze route / line test / sensors only (motors off) / manual / motor test. This replaces the `#define`s at runtime.
- **Run / Pause / Stop**: pause works anywhere, even in the middle of a turn. Sensors keep updating while paused.
- **Step**: run N ms, then pause. **To next node**: run until the next corner or junction, pause *before* the turn. **Pause at every node**.
- **Slow motion** (5-100 %): scales motor speed and every timed move, so turns still end in the same place.
- **Manual**: hold WASD or the arrow keys. The robot stops by itself 300 ms after the page goes quiet. **Jog** gives a single timed move, per wheel or both.

Without the dashboard the robot runs as before. Telemetry only starts when the page connects.
