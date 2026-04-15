#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Autonomous competition script for the Freenove 4WD Tank Robot.
Run this directly on the Raspberry Pi: python3 autonomous.py

State machine:
  FOLLOW_LINE  -> follow black line with IR sensors
  EVADE        -> maneuver around obstacle detected by ultrasonic
  FETCH        -> leave line, approach red ball with camera, grab it
  DELIVER      -> navigate to large drop-zone circle, release ball
"""

import sys
import os
import time
import cv2
import numpy as np

# Ensure imports resolve from this directory
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from camera import Camera
from car import Car

# ---------------------------------------------------------------------------
# Tunable constants — adjust these on competition day
# ---------------------------------------------------------------------------

# Servo angles (set during calibration, see current.md)
GRIPPER_OPEN   = 90    # Servo 0: fully open
GRIPPER_CLOSED = 130   # Servo 0: gripping — lower if crushing, raise if dropping
ARM_DOWN       = 90    # Servo 1: lowered to floor level
ARM_UP         = 140   # Servo 1: raised for transport — raise if not lifting enough

# Camera / vision
FRAME_W = 320
FRAME_H = 240
FRAME_CX = FRAME_W // 2

# Red ball HSV ranges — red wraps around 0/180 in HSV
RED_LOWER1 = np.array([0,   120,  60])
RED_UPPER1 = np.array([10,  255, 255])
RED_LOWER2 = np.array([170, 120,  60])
RED_UPPER2 = np.array([180, 255, 255])

# Drop-zone detection: large white/light circle on the floor
DROP_LOWER = np.array([0,   0, 180])
DROP_UPPER = np.array([180, 50, 255])
DROP_PIXEL_RATIO = 0.12    # fraction of frame that must be white to confirm

# Ball approach thresholds
BALL_MIN_RADIUS  = 12    # px  — ignore noise below this
BALL_GRAB_RADIUS = 62    # px  — close enough to trigger grab

# Scan for red ball only every N frames while line-following (saves CPU)
BALL_SCAN_INTERVAL = 5

# Obstacle distances
OBSTACLE_WARN_CM = 30    # start evasion
SAFE_CM          = 50    # clear of obstacle

# Motor speeds
SPD_FWD    = 1200
SPD_SLOW   = 700
SPD_TURN   = 1500
SPD_SEARCH = 600


class AutonomousRobot(Car):
    """Extends Car with a full autonomous competition state machine."""

    STATE_FOLLOW_LINE = "FOLLOW_LINE"
    STATE_EVADE       = "EVADE"
    STATE_FETCH       = "FETCH"
    STATE_DELIVER     = "DELIVER"
    STATE_RELEASE     = "RELEASE"

    def __init__(self):
        super().__init__()
        self.state       = self.STATE_FOLLOW_LINE
        self.has_ball    = False
        self.frame_count = 0

        # Evasion sub-step state
        self._evade_step  = 0
        self._evade_timer = 0.0

        # Camera (JPEG streaming mode, decoded per-frame)
        self.cam = Camera(stream_size=(FRAME_W, FRAME_H))
        self.cam.start_stream()
        print("[init] Camera started.")

        # Arm starts up/open; lower and open gripper to ready position
        self._arm_ready()

    # -----------------------------------------------------------------------
    # Arm sequences
    # -----------------------------------------------------------------------

    def _arm_ready(self):
        """Move to starting position: arm up, gripper open."""
        self.servo.setServoAngle('1', ARM_UP)
        time.sleep(0.3)
        self.servo.setServoAngle('0', GRIPPER_OPEN)
        time.sleep(0.3)

    def _arm_grab(self):
        """Full grab sequence: lower arm → close gripper → lift arm."""
        print("[arm] Lowering arm...")
        for angle in range(ARM_UP, ARM_DOWN - 1, -1):
            self.servo.setServoAngle('1', angle)
            time.sleep(0.01)
        time.sleep(0.15)

        print("[arm] Closing gripper...")
        for angle in range(GRIPPER_OPEN, GRIPPER_CLOSED + 1):
            self.servo.setServoAngle('0', angle)
            time.sleep(0.01)
        time.sleep(0.2)

        print("[arm] Lifting arm...")
        for angle in range(ARM_DOWN, ARM_UP + 1):
            self.servo.setServoAngle('1', angle)
            time.sleep(0.01)
        time.sleep(0.3)
        print("[arm] Grab complete.")

    def _arm_release(self):
        """Full release sequence: lower arm → open gripper → lift arm."""
        print("[arm] Lowering arm for release...")
        for angle in range(ARM_UP, ARM_DOWN - 1, -1):
            self.servo.setServoAngle('1', angle)
            time.sleep(0.01)
        time.sleep(0.15)

        print("[arm] Opening gripper...")
        for angle in range(GRIPPER_CLOSED, GRIPPER_OPEN - 1, -1):
            self.servo.setServoAngle('0', angle)
            time.sleep(0.01)
        time.sleep(0.2)

        print("[arm] Lifting arm...")
        for angle in range(ARM_DOWN, ARM_UP + 1):
            self.servo.setServoAngle('1', angle)
            time.sleep(0.01)
        time.sleep(0.3)
        print("[arm] Release complete.")

    # -----------------------------------------------------------------------
    # Vision helpers
    # -----------------------------------------------------------------------

    def _get_frame(self):
        """Decode the latest JPEG frame from picamera2 into an OpenCV BGR array."""
        raw = self.cam.get_frame()
        if raw is None:
            return None
        arr = np.frombuffer(raw, dtype=np.uint8)
        return cv2.imdecode(arr, cv2.IMREAD_COLOR)

    def _detect_red_ball(self, frame):
        """
        Returns (cx, cy, radius) of the largest red blob, or None.
        Uses two HSV ranges to cover both sides of the red hue wrap-around.
        """
        blur = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv  = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        mask1 = cv2.inRange(hsv, RED_LOWER1, RED_UPPER1)
        mask2 = cv2.inRange(hsv, RED_LOWER2, RED_UPPER2)
        mask  = cv2.bitwise_or(mask1, mask2)
        mask  = cv2.erode(mask,  None, iterations=2)
        mask  = cv2.dilate(mask, None, iterations=2)

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        if radius < BALL_MIN_RADIUS:
            return None
        M = cv2.moments(c)
        if M["m00"] == 0:
            return None
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return (cx, cy, float(radius))

    def _detect_drop_zone(self, frame):
        """
        Returns True when we're over / right next to the large white drop-zone circle.
        Checks that a large fraction of the bottom half of the frame is white.
        """
        bottom = frame[FRAME_H // 2:, :]
        hsv    = cv2.cvtColor(bottom, cv2.COLOR_BGR2HSV)
        mask   = cv2.inRange(hsv, DROP_LOWER, DROP_UPPER)
        ratio  = cv2.countNonZero(mask) / (bottom.shape[0] * bottom.shape[1])
        return ratio > DROP_PIXEL_RATIO

    # -----------------------------------------------------------------------
    # Motion helpers
    # -----------------------------------------------------------------------

    def _follow_line(self):
        """IR-based line following. Maps sensor bitmask to motor commands."""
        v = self.infrared.read_all_infrared()
        if   v == 2: self.motor.setMotorModel( SPD_FWD,   SPD_FWD)    # centre on line
        elif v == 4: self.motor.setMotorModel(-SPD_TURN,  2500)        # line right → turn right
        elif v == 6: self.motor.setMotorModel(-2000,      4000)        # strong right
        elif v == 1: self.motor.setMotorModel( 2500,     -SPD_TURN)    # line left → turn left
        elif v == 3: self.motor.setMotorModel( 4000,     -2000)        # strong left
        elif v == 7: self.motor.setMotorModel( 0,         0)           # all sensors: stop
        # v == 0: no line detected — keep last command (do nothing)

    def _approach_ball(self, ball):
        """
        Steer toward ball using proportional control on horizontal error.
        Returns True when radius is large enough to grab.
        """
        cx, cy, radius = ball
        if radius >= BALL_GRAB_RADIUS:
            self.motor.setMotorModel(0, 0)
            return True

        err   = cx - FRAME_CX           # positive = ball is right of centre
        gain  = 4.0
        base  = SPD_SLOW if radius > 35 else SPD_FWD
        left  = int(base + err * gain)
        right = int(base - err * gain)
        left  = max(-2000, min(2000, left))
        right = max(-2000, min(2000, right))
        self.motor.setMotorModel(left, right)
        return False

    # -----------------------------------------------------------------------
    # Evasion sub-state machine (non-blocking, time-based)
    # -----------------------------------------------------------------------
    # Steps: 0=reverse, 1=turn-left, 2=forward, 3=turn-right, 4=re-align
    _EVADE_ACTIONS = [
        # (left, right, duration_s)
        (-1200, -1200, 0.35),   # 0: reverse
        (-SPD_TURN, SPD_TURN, 0.55),  # 1: turn left
        ( SPD_FWD,  SPD_FWD,  0.65),  # 2: forward past obstacle
        ( SPD_TURN, -SPD_TURN, 0.50),  # 3: turn right to re-align
        ( SPD_FWD,  SPD_FWD,  0.30),  # 4: small forward to re-acquire line
    ]

    def _run_evade_step(self):
        """Drive one evasion sub-step; transitions back to FOLLOW_LINE when done."""
        step = self._evade_step
        if step >= len(self._EVADE_ACTIONS):
            # Finished evasion sequence
            self.motor.setMotorModel(0, 0)
            self._evade_step = 0
            self.state = self.STATE_FOLLOW_LINE
            print("[evade] Done — resuming line follow.")
            return

        left, right, duration = self._EVADE_ACTIONS[step]
        now = time.time()
        if now - self._evade_timer >= duration:
            # Move to next sub-step
            self._evade_step += 1
            self._evade_timer = now
            if self._evade_step < len(self._EVADE_ACTIONS):
                l2, r2, _ = self._EVADE_ACTIONS[self._evade_step]
                self.motor.setMotorModel(l2, r2)
        # else: still waiting in this sub-step — motors already set

    # -----------------------------------------------------------------------
    # Main loop
    # -----------------------------------------------------------------------

    def run(self):
        print("=== Autonomous competition mode ===")
        print(f"States: {self.STATE_FOLLOW_LINE} -> {self.STATE_EVADE} / {self.STATE_FETCH} -> {self.STATE_DELIVER} -> {self.STATE_RELEASE}")
        print("Press Ctrl+C to stop.\n")

        # Kick off evasion with an initial motor command for step 0
        try:
            while True:
                frame = self._get_frame()
                if frame is None:
                    time.sleep(0.03)
                    continue

                distance = self.sonic.get_distance()   # cm, -1 on error
                self.frame_count += 1

                # ── FOLLOW_LINE ──────────────────────────────────────────
                if self.state == self.STATE_FOLLOW_LINE:

                    # Priority 1 — obstacle
                    if 0 < distance < OBSTACLE_WARN_CM:
                        self.motor.setMotorModel(0, 0)
                        self._evade_step  = 0
                        self._evade_timer = time.time()
                        # Execute first sub-step immediately
                        l, r, _ = self._EVADE_ACTIONS[0]
                        self.motor.setMotorModel(l, r)
                        self.state = self.STATE_EVADE
                        print(f"[line] Obstacle at {distance:.1f} cm — evading.")
                        continue

                    # Priority 2 — ball scan (every N frames)
                    if self.frame_count % BALL_SCAN_INTERVAL == 0:
                        ball = self._detect_red_ball(frame)
                        if ball:
                            self.motor.setMotorModel(0, 0)
                            self.state = self.STATE_FETCH
                            print(f"[line] Red ball detected (r={ball[2]:.1f}px) — fetching.")
                            continue

                    self._follow_line()

                # ── EVADE ────────────────────────────────────────────────
                elif self.state == self.STATE_EVADE:
                    self._run_evade_step()

                # ── FETCH ────────────────────────────────────────────────
                elif self.state == self.STATE_FETCH:
                    ball = self._detect_red_ball(frame)

                    if ball is None:
                        # Lost ball — spin slowly to search
                        self.motor.setMotorModel(-SPD_SEARCH, SPD_SEARCH)
                        time.sleep(0.08)
                        continue

                    close_enough = self._approach_ball(ball)
                    if close_enough:
                        print(f"[fetch] Ball at grab distance (r={ball[2]:.1f}px). Grabbing...")
                        self._arm_grab()
                        self.has_ball = True
                        self.state    = self.STATE_DELIVER
                        print("[fetch] Heading to drop zone.")

                # ── DELIVER ──────────────────────────────────────────────
                elif self.state == self.STATE_DELIVER:
                    if self._detect_drop_zone(frame):
                        self.motor.setMotorModel(0, 0)
                        self.state = self.STATE_RELEASE
                        print("[deliver] Drop zone found — releasing.")
                    else:
                        # Drive slowly forward; use IR to stay vaguely on track
                        self._follow_line()

                # ── RELEASE ──────────────────────────────────────────────
                elif self.state == self.STATE_RELEASE:
                    self._arm_release()
                    self.has_ball = False
                    self._arm_ready()
                    self.state = self.STATE_FOLLOW_LINE
                    print("[release] Ball dropped. Resuming patrol.")

                time.sleep(0.04)   # ~25 Hz loop

        except KeyboardInterrupt:
            print("\n[stop] Interrupted by user.")
        finally:
            self.motor.setMotorModel(0, 0)
            try:
                self.cam.close()
            except Exception:
                pass
            self.close()
            print("[stop] Shutdown complete.")


if __name__ == '__main__':
    robot = AutonomousRobot()
    robot.run()
