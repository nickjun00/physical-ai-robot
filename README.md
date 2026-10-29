# Pygame robot simulator

## Launch on macOS

From Terminal:

```sh
cd ~/Python/physical-ai-robot-agent-1
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --no-cache-dir -r requirements.txt
python pygame_app.py
```

The local `.venv` is already created in this worktree. For subsequent launches:

```sh
cd ~/Python/physical-ai-robot-agent-1
.venv/bin/python pygame_app.py
```

Click the 900 × 600 window to give it keyboard focus.

- WASD or arrow keys: move in manual mode. Diagonal movement has the same speed.
- Space: switch between manual and automatic navigation.
- R: reset the robot and return to manual mode.
- Escape or the window close button: quit.

Blue is the circular robot, red rectangles are obstacles, and green is the target.
The yellow line is the forward-facing sensor; the grey line shows the planned route.
Coordinates use pixels, with (0, 0) at the top left. Speed is actual travelled pixels
per second. The sensor reports distance from the robot centre, capped at 300 pixels;
a capped reading means nothing was detected closer than that distance.

## How it works

`robot_world.py` contains logic without any Pygame dependency. It checks whole
movement segments against rectangles expanded by the robot radius and against
window boundaries. This prevents tunneling even on a long time step. Expansion
is conservative at obstacle corners: it may reject a close pass that a true circle
could fit through. A blocked manual command leaves the robot in place.

Automatic mode uses A* over a 20-pixel grid with eight movement directions.
Every connection is collision checked, so diagonal routes cannot cut through
corners. The robot follows waypoints at up to 160 pixels per second. If no route
exists, it stays still. Toggle modes to plan again from the current position.
The grid may miss passages narrower than its resolution. The map is static and
known; navigation uses the map, while the sensor is a simulated geometric ray.

`pygame_app.py` handles keyboard input, timing, and drawing. It limits frame time
to 0.05 seconds after pauses. `app.py` and `simulator.py` are the earlier Streamlit
prototype and are not needed to launch the Pygame application.

## Tests

```sh
cd ~/Python/physical-ai-robot-agent-1
.venv/bin/python -m unittest discover -s tests -v
```

Tests cover collision tunneling, robot radius, all window edges, sensor readings,
mode switching, unreachable targets, and safe navigation to the default target.
The suite also includes tests for the earlier prototype.

Run a short headless rendering smoke test:

```sh
SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy .venv/bin/python pygame_app.py --auto --frames 20
```

Run a desktop smoke test that closes automatically:

```sh
.venv/bin/python pygame_app.py --auto --frames 180
```
