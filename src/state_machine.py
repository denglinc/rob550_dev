"""
State machine for the arm lab runtime shell.
"""
import math
import time

from PyQt5.QtCore import QThread, pyqtSignal

try:
    from lite6arm import GRIPPER_DWELL_S
except Exception:
    GRIPPER_DWELL_S = 0.8

JOINT_TOL_RAD = 0.02       # per-joint arrival tolerance
MOVE_TIMEOUT_S = 15.0      # give up on a move after this long
APPROACH_MM = 60.0         # hover height above a grasp / drop point
VERTICAL_RPY = (math.pi, 0.0, 0.0)  # tool pointing straight down


class StateMachine:

    def __init__(self, arm, camera):
        self.arm = arm
        self.camera = camera
        self.status_message = "Idle - waiting for input."
        self.current_state = "idle"
        self.next_state = "idle"

        # --- waypoint teach/playback ---
        self.waypoints = []           # [(joint_angles_rad, gripper_closed), ...]
        self.gripper_closed = False   # commanded gripper state (no feedback on the LG-1000)
        self._pb_idx = 0
        self._pb_phase = "move"
        self._pb_t0 = 0.0

        # --- pick & place ---
        self._holding = False

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

    # ------------------------------------------------------------------
    # Gripper bookkeeping
    # ------------------------------------------------------------------

    def set_gripper(self, closed):
        """
        Drive the gripper AND remember the commanded state.

        The LG-1000 has no position feedback, so the only way a waypoint can
        store a gripper state is if every open/close goes through here. Wire the
        GUI's gripper buttons to this instead of calling arm.open/close_gripper.
        """
        if not self.arm.connected:
            return
        if closed:
            self.arm.close_gripper()
        else:
            self.arm.open_gripper()
        self.gripper_closed = bool(closed)

    # ------------------------------------------------------------------
    # Waypoints
    # ------------------------------------------------------------------

    def add_waypoint(self):
        """Record current joint angles + gripper state as one waypoint."""
        if not self.arm.connected:
            self._go_idle("Arm offline - cannot record a waypoint.")
            return

        angles = self.arm.get_joint_angles()
        if angles is None:
            self._go_idle("Could not read joint angles - waypoint not recorded.")
            return

        self.waypoints.append((list(angles), self.gripper_closed))
        grip = "closed" if self.gripper_closed else "open"
        self._go_idle(f"Waypoint {len(self.waypoints)} recorded (gripper {grip}).")

    def clear_waypoints(self):
        """Erase all recorded waypoints."""
        count = len(self.waypoints)
        self.waypoints = []
        self._pb_idx = 0
        self._pb_phase = "move"
        self._go_idle(f"Cleared {count} waypoint(s).")

    def playback_waypoints(self):
        """
        Replay waypoints in order: move joints, wait for arrival, apply the
        gripper state, dwell, advance.

        Written as one step per run() tick rather than a blocking loop so the
        status message keeps updating and the GUI can preempt with STOP.
        """
        if not self.arm.connected or not self.arm.initialized:
            self._go_idle("Playback unavailable until the arm is initialized.")
            return
        if not self.waypoints:
            self._go_idle("No waypoints recorded - nothing to play back.")
            return

        # First tick of this state: reset progress and leave velocity mode,
        # since set_servo_angle only works in position mode (mode 0).
        if self.current_state != "playback_waypoints":
            self.arm.enable()
            self.current_state = "playback_waypoints"
            self._pb_idx = 0
            self._pb_phase = "move"

        if self.arm.has_fault():
            self._go_idle("Arm fault during playback - stopped.")
            return

        total = len(self.waypoints)
        angles, gripper_closed = self.waypoints[self._pb_idx]

        if self._pb_phase == "move":
            self.status_message = f"Playback: moving to waypoint {self._pb_idx + 1}/{total}..."
            self.arm.set_joint_angles(angles)
            self._pb_t0 = time.time()
            self._pb_phase = "wait_move"

        elif self._pb_phase == "wait_move":
            if self._at_target(angles):
                self._pb_phase = "gripper"
            elif time.time() - self._pb_t0 > MOVE_TIMEOUT_S:
                self._go_idle(f"Playback timed out reaching waypoint {self._pb_idx + 1}.")

        elif self._pb_phase == "gripper":
            grip = "closing" if gripper_closed else "opening"
            self.status_message = f"Playback: {grip} gripper at waypoint {self._pb_idx + 1}/{total}..."
            self.set_gripper(gripper_closed)
            self._pb_t0 = time.time()
            self._pb_phase = "wait_gripper"

        elif self._pb_phase == "wait_gripper":
            if time.time() - self._pb_t0 >= GRIPPER_DWELL_S:
                self._pb_idx += 1
                self._pb_phase = "move"
                if self._pb_idx >= total:
                    self._go_idle(f"Playback complete ({total} waypoints).")

    # ------------------------------------------------------------------
    # Pick & place
    # ------------------------------------------------------------------

    def pick_place(self):
        """
        Click an object in the video feed to pick it up, then click a
        destination to drop it. Stays active until the GUI toggle is cleared.
        """
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
        """Approach from above, descend, actuate the gripper, retreat."""
        x, y, z = world_xyz[0], world_xyz[1], world_xyz[2]
        above = [x, y, z + APPROACH_MM, *VERTICAL_RPY]
        at = [x, y, z, *VERTICAL_RPY]

        if not self._move_to_pose(above):
            return False
        if not self._move_to_pose(at):
            return False

        self.set_gripper(close_gripper)
        time.sleep(GRIPPER_DWELL_S)

        return self._move_to_pose(above)

    def _move_to_pose(self, pose):
        """Solve IK for a Cartesian pose and move there, blocking until arrival."""
        angles = self.arm.get_ik(pose)
        if angles is None:
            angles = self.arm.get_ik_numerical(pose)
        if angles is None:
            self.status_message = "No IK solution for that point."
            return False

        self.arm.set_joint_angles(angles)
        return self._wait_for_arrival(angles)

    def _wait_for_arrival(self, target, timeout=MOVE_TIMEOUT_S):
        """Block until the arm reaches target, bailing out on STOP or fault."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.next_state == "estop" or self.arm.has_fault():
                return False
            if self._at_target(target):
                return True
            time.sleep(0.02)
        return False

    def _at_target(self, target):
        """True when every joint is within tolerance of the target angles."""
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