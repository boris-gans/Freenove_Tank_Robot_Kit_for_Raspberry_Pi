# Software Architecture Report — Freenove 4WD Tank Robot

## High-Level Architecture

The codebase is an embedded robotics stack split into three top-level areas under `Code/`:

| Directory | Role |
|---|---|
| `Code/Server/` | Runs on the Raspberry Pi. Drives all hardware and exposes the robot over TCP. |
| `Code/Client/` | PyQt5 desktop app (Mac/Windows/Linux) that controls the robot remotely over TCP. |
| `Code/Libs/` | Vendored `rpi-ws281x-python` library for WS2812 LED strips. |

The robot supports two distinct operating regimes:

1. **Tele-op mode** — `Server/main.py` runs a Qt GUI that hosts a TCP server. `Client/Main.py` connects to it; the user drives the robot manually, watches the camera stream, and toggles canned modes (ultrasonic obstacle avoidance, line follow, clamp).
2. **Standalone autonomous mode** — `Server/autonomous.py` runs directly on the Pi with no networking. It executes a competition state machine (line follow → fetch red ball → deliver to drop zone). This is the file we've been iterating on.

The two regimes share the *hardware-driver layer* (motor, servo, infrared, ultrasonic, camera, led) but have completely separate top-level controllers (`main.py` vs. `autonomous.py`).

---

## `Code/Server/` — Module-by-module

### Top-level entry points

**`main.py` (342 LoC)** — the tele-op orchestrator. Defines `mywindow(QMainWindow)`, which owns:
- a `TankServer` (TCP)
- a `Car` (hardware aggregate)
- a `Camera`, a `Led`, a `Command` symbol table, a `MessageParser`
- four worker threads and one subprocess:
  - `threading_cmd_receive` — reads control messages, dispatches `CMD_MOTOR`, `CMD_SERVO`, `CMD_MODE`, `CMD_ACTION` to the `Car`
  - `threading_video_send` — pulls JPEGs from `Camera` and ships them with a 4-byte length prefix to the video TCP port
  - `threading_car_task` — runs the active car-mode state (`mode_ultrasonic`, `mode_infrared`, clamp sequences)
  - `process_led_running` — separate `multiprocessing.Process` so LED animations don't stall the UI
- `set_threading_*` / `set_process_led_running` are start/stop wrappers; `close_application` and `signal_handler` provide clean shutdown.

**`autonomous.py` (441 LoC)** — the standalone competition controller. Defines `AutonomousRobot(Car)`, which adds a 5-state state machine on top of `Car`'s hardware:
- `STATE_FOLLOW_LINE` — IR-based line following with search recovery
- `STATE_EVADE` — non-blocking ultrasonic-triggered evasion sub-sequence (`_EVADE_ACTIONS` list of `(left, right, duration)` tuples driven by `_run_evade_step`)
- `STATE_FETCH` — vision-based red-ball approach using HSV masking + circularity gating in `_detect_red_ball`, proportional steering in `_approach_ball`
- `STATE_DELIVER` — drives via `_follow_line` while watching for a circular white blob via `_detect_drop_zone`
- `STATE_RELEASE` — runs the arm release sequence
- Arm helpers: `_arm_ready`, `_arm_grab`, `_arm_release` (all blocking servo sweeps with `time.sleep` settling times).
- Constants at the top (HSV ranges, motor speeds, radius/area thresholds) are the *primary tuning surface* — they're meant to be edited per-track, not buried in functions.

**`test.py`** — bench-test entry points: `test_Parameter()`, `test_Led()`, `test_Servo()`, `test_Motor()`, etc.

### Hardware abstraction layer

**`car.py` (247 LoC)** — `Car` is the hardware aggregate. In `__init__`/`start()` it lazily constructs `Servo`, `Ultrasonic`, `tankMotor`, `Infrared`. It also implements three legacy "modes" used by the tele-op pipeline:
- `mode_ultrasonic` — drive forward unless distance < 45 cm
- `mode_infrared` — original IR line follower (the reference implementation we cross-checked against in `autonomous.py`)
- `mode_clamp_up`, `mode_clamp_down`, `mode_clamp` — closed-loop ball pickup using ultrasonic distance to position the chassis before the gripper sequence
- `set_mode_clamp` / `get_mode_clamp` — clamp-mode getter/setter pair used by `main.py`'s car thread.

**`motor.py` (72 LoC)** — `tankMotor` wraps two `gpiozero.Motor` objects (left on GPIO 23/24, right on 5/6). Public surface is just `setMotorModel(duty1, duty2)`, which clamps to ±4095 and routes to `forward`/`backward`/`stop`. This is the single chokepoint every higher-level controller writes through.

**`servo.py` (177 LoC)** — three implementations of the same interface, picked by config:
- `PigpioServo` — uses `pigpio` daemon
- `GpiozeroServo` — uses `gpiozero.AngularServo` (software PWM)
- `HardwareServo` — uses `rpi_hardware_pwm` (hardware PWM, more jitter-resistant)
- `Servo` (the public class) — selects backend based on `Pcb_Version` / `Pi_Version` from `ParameterManager`. Public method is `setServoAngle(channel, angle)`, used everywhere from arm sequences to the calibration UI.

**`infrared.py` (65 LoC)** — `Infrared` wraps three `gpiozero.LineSensor` instances. PCB-version-dependent pin mapping. `read_one_infrared(channel)` returns 0/1 per sensor; `read_all_infrared()` packs all three into a 3-bit integer `(IR1<<2)|(IR2<<1)|IR3` — this is the `v` value the line follower switches on.

**`ultrasonic.py` (138 LoC)** — three backends like `servo.py`: `gpiozero_ultrasonic`, `lgpiod_ultrasonic`, and a fallback. Public `Ultrasonic` class selects one and exposes `get_distance()` returning cm or `-1` on error. The `DistanceSensorNoEcho` warnings you saw at startup come from here.

**`camera.py` (98 LoC)** — `Camera` wraps `picamera2`. Two modes:
- `start_image` / `save_image` — preview window + still capture
- `start_stream` — JPEG encoder writing into a `StreamingOutput(io.BufferedIOBase)` ring buffer with a `threading.Condition` for frame-arrival notification
- `get_frame(timeout=0.1)` — blocks on the condition (with timeout, after our recent fix) and returns the latest JPEG bytes
- `close` / `stop_stream` — teardown.

**`led.py` (182 LoC), `rpi_ledpixel.py` (191 LoC), `spi_ledpixel.py` (299 LoC)** — `Led` selects between `Freenove_RPI_WS281X` (PCB v1, Pi 4) and `Freenove_SPI_LedPixel` (PCB v2, Pi 4 or 5) based on `ParameterManager`. Animation primitives: `colorWipe`, `Blink`, `Breathing`, `rainbow`, `rainbowCycle`, `theaterChaseRainbow`, `ledIndex(mask, r, g, b)`. Driven from a separate process (see `process_led_running` in `main.py`) because WS281x writes are timing-sensitive.

### Configuration / parameters

**`parameter.py` (139 LoC)** — `ParameterManager` reads/writes `params.json` with two keys: `Pcb_Version` (1|2) and `Pi_Version` (1=Pi4-and-below, 2=Pi5). On first run it shells out to detect the Pi model and asks the user for the PCB revision. Used by every hardware module that has version-dependent behavior. `get_pcb_version()` / `get_raspberry_pi_version()` are the most-called accessors; `validate_params()` ensures the JSON has both fields.

### Networking layer

**`tcp_server.py` (168 LoC)** — `TCPServer` is a generic single-threaded `select`-based TCP server. Accepts up to `max_clients`, queues received messages to `message_queue`, exposes `send_to_client` / `send_to_all_client` for outgoing. Uses a `socketpair` as a stop-pipe to break out of `select.select` cleanly on shutdown. `accept_connections` runs as a daemon thread.

**`server.py` (114 LoC)** — `TankServer` is the application-specific wrapper over `TCPServer`. Holds *two* `TCPServer` instances:
- `cmdServer` on port **5003** — newline-delimited ASCII commands
- `videoServer` on port **8003** — binary, length-prefixed JPEG frames
- `get_interface_ip` — pulls the `wlan0` IP via an `ioctl(SIOCGIFADDR)` so the GUI can display "Server On at 192.168.x.y"
- `sendDataToCmdClinet` / `sendDataToVideoClient` — guarded by busy flags to avoid concurrent writes
- `readDataFromCmdServer` — drain helper used by the command-receive thread.

**`message.py` (60 LoC)** — `MessageParser`. Stateful parser with `clearParameters()` and `parser(msg)`. Splits on `#`: first element → `commandString`, rest → `intParameter` (rounded floats). Used to decode wire messages like `CMD_MOTOR#1500#-1500`.

**`command.py` (10 LoC)** — Command symbol table. Just string constants: `CMD_MOTOR`, `CMD_LED`, `CMD_SERVO`, `CMD_ACTION`, `CMD_SONIC`, `CMD_MODE`. Imported on both server and client to avoid drift.

### UI

**`server_ui.py` (98 LoC)** — Qt Designer–generated `Ui_server_ui` class for the Pi-side server status window (one button + one IP label). Pure layout, no logic.

---

## `Code/Client/` — Desktop control app

**`Main.py` (812 LoC)** — `mywindow(QMainWindow, Ui_Client)`. The user-facing app. Handles keyboard/joystick input, button bindings, video display, and ultrasonic radar plot. Builds outgoing `CMD_*#arg#arg\n` strings and ships them via the `VideoStreaming` socket wrapper.

**`Client_Ui.py` (1010 LoC)** — Qt Designer–generated `Ui_Client` layout. Largest file in the repo, all auto-generated.

**`Video.py` (115 LoC)** — `VideoStreaming` class. Holds two sockets (cmd + video), runs a thread that reads the 4-byte length prefix, then the JPEG payload, decodes it via OpenCV/PIL, and pushes a `QImage` to the GUI.

**`Thread.py` (27 LoC)** — small `stoppable_thread` helper for clean teardown of background workers.

**`Command.py` (13 LoC)** — mirror of the server's `command.py` so both ends agree on string constants.

**`PID.py` (48 LoC)** — `Incremental_PID` class. Standalone PID with anti-windup saturation. Used by `Main.py` for closed-loop correction (e.g., cradle pan/tilt smoothing). Note: not used by `autonomous.py`; the autonomous controller is purely bang-bang/proportional.

**`calibrate.py` (15 LoC)** — small launcher for servo calibration UI.

---

## Configuration / setup

- `Code/setup.py`, `setup_macos.py`, `setup_windows.py` — install scripts for the host OS.
- `params.json` — generated at runtime by `ParameterManager`.
- `current.md`, `project.md`, `README.md` — top-level documentation. `current.md` is your working notes for the competition.

---

## Cross-cutting design patterns

**Hardware abstraction by PCB/Pi version.** `servo.py`, `ultrasonic.py`, and `led.py` all follow the same pattern: a public class selects from 2–3 backends at construction time based on `ParameterManager`. Means the rest of the code never needs `if pcb_version == ...` — version branching is contained at the driver boundary.

**Single threading model in tele-op, single-process in autonomous.** `main.py` uses three threads + one process (LED) all coordinated via `multiprocessing.Queue` and busy-flags. `autonomous.py` is *deliberately* single-threaded — one ~25 Hz loop in `run()` reads sensors, dispatches by state, sleeps 0.04 s. This is why decoupling the line follower from `Camera.get_frame()` mattered so much: any blocking call in that loop pauses the whole robot.

**Wire protocol.** ASCII, `#`-separated, `\n`-terminated. `MessageParser` on both ends. Commands are one-shot (no acks, no sequence numbers). Video is binary on a separate port. Simple, easy to debug with `nc`, but offers no flow control or replay protection.

**State machines.** Two of them:
- `Car.clamp_mode` (0/1/2) — a tiny stop/up/down state used by `mode_clamp` and exposed to the GUI through `set_mode_clamp` / `get_mode_clamp`.
- `AutonomousRobot.state` (FOLLOW_LINE/EVADE/FETCH/DELIVER/RELEASE) — the competition-day machine. Sub-state for evasion is handled with an index + timestamp into `_EVADE_ACTIONS`.

**Tuning constants exposed at top of file.** `autonomous.py` lines 30–67 are the entire calibration surface (HSV ranges, ball/zone thresholds, motor speeds, evasion durations). The `_follow_line` hardcoded mid-function values for `1400`, `2200`, etc. are the one place this discipline isn't perfect.

---

## Data flow at runtime

**Tele-op:** Client keypress → `Main.py` builds `CMD_MOTOR#L#R\n` → TCP `:5003` → `tcp_server.py` queues it → `main.py:threading_cmd_receive` parses via `MessageParser` → dispatches to `car.motor.setMotorModel()` → `gpiozero.Motor` → GPIO PWM. In parallel: `picamera2` → `StreamingOutput` ring buffer → `main.py:threading_video_send` length-prefixes → TCP `:8003` → `Client/Video.py` decodes → `Client_Ui` `QLabel`.

**Autonomous:** `autonomous.py:run()` loop tick → `Ultrasonic.get_distance()` + state dispatch → IR/camera read → `_follow_line` / `_detect_red_ball` / `_detect_drop_zone` → `motor.setMotorModel()` and/or `servo.setServoAngle()` → GPIO. The camera is now only consulted for ball scans (every 5 ticks) and the FETCH/DELIVER states, not the line-following hot path.
