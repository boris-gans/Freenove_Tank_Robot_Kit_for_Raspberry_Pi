---
name: Competition Project Context
description: Final project — autonomous logistics robot on a factory floor arena
type: project
---

The robot must run a single autonomous Python script (Code/Server/autonomous.py) on the Pi.

**Why:** School competition requiring full autonomy — follow line, dodge obstacles, grab red ball, deliver to drop zone.

**How to apply:** All new logic goes in autonomous.py. The old Client Qt GUI (Main.py) is not used for the competition run; it stays as a manual debugging tool. The Server hardware modules (motor, servo, infrared, ultrasonic, camera) are the source of truth for hardware APIs.

Key files:
- Code/Server/autonomous.py — competition state machine (FOLLOW_LINE / EVADE / FETCH / DELIVER / RELEASE)
- Code/Server/car.py — hardware Car class, extended by autonomous.py
- Code/Client/requirements.txt — all pip dependencies

Servo calibration (from current.md, performed 2026-04-15):
- Servo 0 (Gripper): 90° = OPEN, 130° = CLOSED (tune up/down if crushing/dropping)
- Servo 1 (Lift Arm): 90° = DOWN, 140° = UP
- mode_clamp_up grab window was 7.5–7.7 cm (too narrow for sensor accuracy); widened to 6.5–9.5 cm

Red ball HSV detection uses two ranges (H 0–10 and H 170–180) because red wraps around 0/180 in HSV.
