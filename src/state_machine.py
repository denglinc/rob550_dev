"""
State machine for the arm lab runtime shell.
"""
import math
import time

import numpy as np
from PyQt5.QtCore import QThread, pyqtSignal

from kinematics import Q_DEFAULT_DEG
from lite6arm import GRIPPER_DWELL_S

JOINT_TOL_RAD = 0.02       # per-joint arrival tolerance
MOVE_TIMEOUT_S = 15.0      # give up on a move after this long
WAYPOINT_PAUSE_S = 1.0     # settle time at each waypoint before the gripper acts
APPROACH_MM = 60.0         # hover height above a grasp / drop point
VERTICAL_RPY = (math.pi, 0.0, 0.0)  # tool pointing straight down

# Task 1.3 demo routine: (joint angles in degrees, gripper_closed) per waypoint,
# converted to radians below. Starts and ends at the home pose.
HOME_DEG = list(Q_DEFAULT_DEG)            # [0, 9.9, 31.8, 0, 21.9, 0]
ROUTINE_DEG = [
    (HOME_DEG,                        False),
    ([ 45,  15, 40,   0, 30,   0],    False),   # turn left, lean forward
    ([ 45,   0, 70,   0, 60,   0],    True),    # curl the forearm in, close gripper
    ([-45,   0, 70,   0, 60,   0],    True),    # sweep to the right (gripper stays closed)
    ([-45,  15, 40,  45, 30, -45],    False),   # lean forward, twist wrist, open gripper
    ([  0,  15, 40, -45, 30,  45],    False),   # centre, twist the other way
    ([  0, -10, 60,   0, 80,   0],    False),   # lean back, wrist up
    (HOME_DEG,                        False),
]
ROUTINE = [(np.radians(q).tolist(), closed) for q, closed in ROUTINE_DEG]


class StateMachine:

    def __init__(self, arm, camera):
        self.arm = arm
        self.camera = camera
        self.status_message = "Idle - waiting for input."
        self.current_state = "idle"
        self.next_state = "idle"

        # Waypoints: [(joint_angles_rad, gripper_closed), ...]. Starts loaded with
        # the Task 1.3 routine; the first Add Waypoint (or Clear All) discards it.
        self.waypoints = list(ROUTINE)
        self._routine_loaded = True

        self._holding = False   # pick & place: currently carrying an object

        self._handlers = {
            "initial_pose":      self.initial_pose,
            "sleep_arm":         self.sleep_arm,
            "estop":             self.estop,
            "calibrate":         self.calibrate,
            "add_waypoint":      self.add_waypoint,
            "clear_waypoints":   self.clear_waypoints,
            "playback_waypoints":self.playback_waypoints,
            "direct_control":    self.direct_control,
            "pick_place":        self.pick_place,
        }

    def set_next_state(self, state):
        self.next_state = state

    def _go_idle(self, message):
        self.status_message = message
        self.current_state = "idle"
        self.next_state = "idle"

    def run(self):
        self._handlers.get(self.next_state, self.idle)()

    def idle(self):
        if self.current_state != "idle":
            self.status_message = "Idle - waiting for input."
        self.current_state = "idle"

    def direct_control(self):
        if not self.arm.connected or not self.arm.initialized:
            self._go_idle("Direct control unavailable until the arm is initialized.")
            return
        if self.current_state != "direct_control":
            self.status_message = "Direct Control - use jog controls to move the arm."
        self.current_state = "direct_control"

    def estop(self):
        if self.current_state != "estop":
            self.arm.stop()
            self.status_message = "STOP active - clear the arm and click Go Home to recover."
            self.current_state = "estop"

    def initial_pose(self):
        if not self.arm.connected:
            self._go_idle("Arm offline - cannot initialize.")
            return
        self.status_message = "Initializing arm..."
        if not self.arm.initialize():
            self._go_idle("Failed to initialize arm.")
        else:
            self._go_idle("Arm initialized - at initial pose.")

    def sleep_arm(self):
        if not self.arm.connected:
            self._go_idle("Arm offline - cannot send Sleep.")
            return
        self.status_message = "Sleep: returning to home position, motors will shut off..."
        self.arm.sleep()
        self._go_idle("Sleep: home position reached, motors off.")

    def calibrate(self):
        _, message = self.camera.estimate_extrinsics_from_tags()
        self._go_idle(message)

    def set_gripper(self, closed):
        # Open or close the gripper, then wait for it to finish. It has no feedback,
        # so the wait is a fixed time (GRIPPER_DWELL_S from lite6arm.py; after an
        # open the arm also switches the gripper motor off at that time).
        if closed:
            self.arm.close_gripper()
        else:
            self.arm.open_gripper()
        time.sleep(GRIPPER_DWELL_S)

    def add_waypoint(self):
        # TODO: student lab
        # Each waypoint stores joint angles + gripper state together.
        # Two consecutive waypoints can have identical joint angles but different gripper states
        # (e.g. WP: arm at grasp position, gripper open -> next WP: same position, gripper closed).
        # Playback executes each waypoint sequentially: move joints first, then apply gripper state.
        if not self.arm.connected:
            self._go_idle("Arm offline - cannot record a waypoint.")
            return

        angles = self.arm.get_joint_angles()
        if angles is None:
            self._go_idle("Could not read joint angles - waypoint not recorded.")
            return

        # First Add Waypoint replaces the preloaded routine (no need to click Clear All).
        if self._routine_loaded:
            self.waypoints = []
            self._routine_loaded = False
            print("Preloaded routine discarded - recording your own waypoints now.")

        # No gripper feedback: arm.gripper_closed is the last commanded state,
        # set by arm.open_gripper() / arm.close_gripper() (the GUI buttons call these).
        closed = self.arm.gripper_closed
        self.waypoints.append((list(angles), closed))
        self._print_waypoint(len(self.waypoints), angles, closed)
        grip = "closed" if closed else "open"
        self._go_idle(f"Waypoint {len(self.waypoints)} recorded (gripper {grip}).")

    def clear_waypoints(self):
        # TODO: student lab
        count = len(self.waypoints)
        self.waypoints = []
        self._routine_loaded = False
        self._go_idle(f"Cleared {count} waypoint(s).")

    def playback_waypoints(self):
        # TODO: student lab
        # For each waypoint: move to joint angles (wait), then apply gripper state (wait), then advance.
        # Blocking loop: the GUI status only refreshes when this returns, so progress is
        # printed to the terminal. STOP still works because _wait_for_arrival watches for it.
        if not self.arm.connected or not self.arm.initialized:
            self._go_idle("Playback unavailable until the arm is initialized.")
            return
        if not self.waypoints:
            self._go_idle("No waypoints - record some (or restart the GUI to reload the routine).")
            return

        # Check every waypoint against the joint limits before moving at all.
        for i, (angles, _closed) in enumerate(self.waypoints, 1):
            if not self._within_limits(angles):
                self._go_idle(f"Playback refused: waypoint {i} is outside the joint limits.")
                return

        total = len(self.waypoints)
        print(f"--- Playback: {total} waypoints ---")
        self._print_waypoints(self.waypoints)

        self.current_state = "playback_waypoints"
        self.arm.enable()   # set_joint_angles needs position mode (mode 0)

        prev_closed = None  # gripper state of the previous waypoint; None = none yet
        for i, (angles, closed) in enumerate(self.waypoints, 1):
            print(f"Playback: moving to waypoint {i}/{total}")
            if not self.arm.set_joint_angles(angles):
                self._go_idle(f"Playback stopped: move command for waypoint {i} failed.")
                return
            if not self._wait_for_arrival(angles):
                if self.next_state == "estop":
                    # STOP was pressed: leave next_state alone so estop() runs next tick.
                    self.current_state = "idle"
                    return
                self._go_idle(f"Playback stopped: did not reach waypoint {i} (fault or timeout).")
                return

            # Arrived: pause so the arm settles, then apply the gripper state stored
            # with this waypoint (set_gripper waits for the gripper too). Skip it
            # when the state is the same as at the previous waypoint; the first
            # waypoint always actuates so the gripper starts in a known state.
            time.sleep(WAYPOINT_PAUSE_S)
            grip = "close" if closed else "open"
            if closed == prev_closed:
                print(f"Playback: gripper already {grip} at waypoint {i}/{total} - no action")
            else:
                print(f"Playback: gripper {grip} at waypoint {i}/{total}")
                self.set_gripper(closed)
            prev_closed = closed

        self._go_idle(f"Playback complete ({total} waypoints).")

    def _within_limits(self, angles):
        # True when every joint angle (rad) is inside the arm's joint limits.
        if len(angles) != len(self.arm.joint_limits):
            return False
        return all(lo <= a <= hi for a, (lo, hi) in zip(angles, self.arm.joint_limits))

    def _print_waypoint(self, index, angles, gripper_closed):
        # One table row: index, six joint angles in degrees, gripper state.
        deg = ", ".join(f"{math.degrees(a):7.2f}" for a in angles)
        grip = "closed" if gripper_closed else "open"
        print(f"WP {index:2d} | {deg} | gripper {grip}")

    def _print_waypoints(self, waypoints):
        print("WP    | base, shoulder, elbow, forearm roll, wrist pitch, wrist roll (deg) | gripper")
        for i, (angles, closed) in enumerate(waypoints, 1):
            self._print_waypoint(i, angles, closed)

    def pick_place(self):
        # TODO: student lab
        # Student lab: click an object in the video feed to pick it up and place it.
        # 1. Poll camera.new_click; when True, read camera.last_click for pixel (u, v).
        # 2. camera.image_to_world(u, v) -> world XYZ using live depth + extrinsics.
        # 3. arm.get_ik(pose) -> joint angles; arm.set_joint_angles(...) to move above object.
        # 4. arm.close_gripper() to grasp, move to drop position, arm.open_gripper().
        # State persists until the GUI toggle is unchecked.
        if not self.arm.connected or not self.arm.initialized:
            self._go_idle("Pick & place unavailable until the arm is initialized.")
            return

        if self.current_state != "pick_place":
            self.arm.enable()
            self.current_state = "pick_place"
            self._holding = False
            self.set_gripper(False)
            self.status_message = "Pick & Place - click an object in the video feed."

        if not getattr(self.camera, "new_click", False):
            return
        self.camera.new_click = False

        u, v = self.camera.last_click
        world = self.camera.image_to_world(u, v)
        if world is None:
            self.status_message = "Could not deproject that click (no depth) - try again."
            return

        if not self._holding:
            self.status_message = "Picking..."
            if self._visit(world, close_gripper=True):
                self._holding = True
                self.status_message = "Picked up - click where to place it."
            else:
                self.status_message = "Pick failed - click the object again."
        else:
            self.status_message = "Placing..."
            if self._visit(world, close_gripper=False):
                self._holding = False
                self.status_message = "Placed - click the next object."
            else:
                self.status_message = "Place failed - click the destination again."

    def _visit(self, world_xyz, close_gripper):
        # Approach from above, descend, actuate the gripper, retreat.
        x, y, z = world_xyz[0], world_xyz[1], world_xyz[2]
        above = [x, y, z + APPROACH_MM, *VERTICAL_RPY]
        at = [x, y, z, *VERTICAL_RPY]

        if not self._move_to_pose(above):
            return False
        if not self._move_to_pose(at):
            return False

        self.set_gripper(close_gripper)

        return self._move_to_pose(above)

    def _move_to_pose(self, pose):
        # Solve IK for a Cartesian pose and move there, blocking until arrival.
        angles = self.arm.get_ik(pose)
        if angles is None:
            angles = self.arm.get_ik_numerical(pose)
        if angles is None:
            self.status_message = "No IK solution for that point."
            return False

        self.arm.set_joint_angles(angles)
        return self._wait_for_arrival(angles)

    def _wait_for_arrival(self, target, timeout=MOVE_TIMEOUT_S):
        # Block until the arm reaches target, bailing out on STOP, fault or timeout.
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.next_state == "estop" or self.arm.has_fault():
                return False
            if self._at_target(target):
                return True
            time.sleep(0.02)
        return False

    def _at_target(self, target):
        # True when every joint is within tolerance of the target angles.
        current = self.arm.joint_angles
        if current is None or len(current) < len(target):
            return False
        return all(abs(c - t) < JOINT_TOL_RAD for c, t in zip(current, target))


class StateMachineThread(QThread):
    updateStatusMessage = pyqtSignal(str)

    def __init__(self, state_machine, parent=None):
        QThread.__init__(self, parent=parent)
        self.sm = state_machine
        self._running = True

    def stop(self):
        self._running = False

    def run(self):
        while self._running:
            self.sm.run()
            self.updateStatusMessage.emit(self.sm.status_message)
            time.sleep(0.05)
