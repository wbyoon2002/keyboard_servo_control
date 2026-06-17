"""
Keyboard-driven 5-servo motor control.

Pairs with the firmware in:
    firmware/keyboard_motor_control/keyboard_motor_control.ino

What it does
------------
1. Rotate motors from the keyboard. Several motors can move at the SAME time
   because key state is read directly (you can hold several keys together).
   Each motor has two keys (see CW_KEYS / CCW_KEYS below): one drives its
   angle up, the other drives it down. The real-life direction each key
   produces is documented next to those lists and printed in the banner.
   The rotation speed (degrees per second) is adjustable on the fly.

2. Save the current angles of all motors to a NEW timestamped file with one
   key press (space).

3. Recall a saved pose while controlling: press a number key (1-9) to load
   saved_angles/angles_<n>.txt and smoothly drive all motors to that pose at
   the current speed. Touching any motor key cancels the move and hands
   control straight back to you.

Requirements
------------
    pip install pyserial keyboard
(see requirements.txt in this folder)

The `keyboard` library reads real hardware key state, which is what makes
holding several keys at once work. On Windows it may need to be run from a
terminal with administrator rights; if key reading fails, re-run the
terminal "as administrator".

Usage
-----
    python motor_keyboard_control.py            # uses default port COM3
    python motor_keyboard_control.py COM5       # override the port
"""

import os
import sys
import time
from datetime import datetime

try:
    import serial  # pyserial
except ImportError:
    sys.exit("Missing dependency 'pyserial'. Install with: pip install pyserial keyboard")

try:
    import keyboard
except ImportError:
    sys.exit("Missing dependency 'keyboard'. Install with: pip install pyserial keyboard")


# --------------------------------------------------------------------------
# Configuration & Bindings
# --------------------------------------------------------------------------
DEFAULT_PORT = "COM3"          # change or pass as a command line argument
BAUD = 115200                  # must match the Arduino sketch
SERVO_COUNT = 5

# Give each motor a name. Edit these to match your build; order is motor 0..4.
# Names are only used for display and in the saved files (the firmware uses
# the index), so you can rename them freely without touching anything else.
MOTOR_NAMES = ["arm_top", "arm_wheel", "arm_bottom", "arm_left", "arm_right"]

# Per-motor key bindings. CW_KEYS[i] increases servo i's angle, CCW_KEYS[i]
# decreases it. Which physical motion that produces depends on the build, so
# the real-life effect of each key is noted below (motors 0..4):
#
#   motor 0 arm_top    q: rotate CW        a: rotate CCW
#   motor 1 arm_wheel  s: turn CW          w: turn CCW
#   motor 2 arm_bottom d: turn CW          e: turn CCW
#   motor 3 arm_left   f: drop linkage     r: lift linkage
#   motor 4 arm_right  t: lift linkage     g: drop linkage
CW_KEYS = ["q", "s", "d", "f", "t"]   # increase angle, motors 0..4
CCW_KEYS = ["a", "w", "e", "r", "g"]  # decrease angle, motors 0..4

SAVE_KEY = "space"             # save current angles to a new file
HOME_KEY = "h"                 # move all motors to center (90 deg)
OFF_KEY = "o"                  # release (de-energize) all motors
SPEED_UP_KEYS = ["=", "+"]     # faster
SPEED_DOWN_KEYS = ["-"]        # slower
QUIT_KEY = "esc"

# Number key N loads saved_angles/angles_<N>.txt and moves there.
LOAD_SLOT_KEYS = [str(d) for d in range(1, 10)]   # 1..9
LIST_KEY = "l"                 # print the available saved sets

START_ANGLE = 90.0             # center; matches the firmware "H" command
ANGLE_MIN = 0.0
ANGLE_MAX = 180.0

SPEED_DEG_PER_S = 60.0         # initial rotation speed
SPEED_MIN = 5.0
SPEED_MAX = 300.0
SPEED_STEP = 15.0

LOOP_HZ = 50.0                 # control / streaming rate
SAVE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "saved_angles")


# =============================================================================
# SERIAL COMMUNICATIONS COMMAND FUNCTIONS
# =============================================================================

def send_set_all_angles(ser, angles):
    """Sends 'S a0 a1 a2 a3 a4' command to set all servo angles immediately."""
    rounded = [int(round(a)) for a in angles]
    cmd = f"S {' '.join(str(a) for a in rounded)}\n"
    ser.write(cmd.encode())


def send_move_servo(ser, index, angle, speed=None):
    """Sends 'M i a [v]' command to move a single servo to target angle.
    If speed (v) is provided, the movement will be handled at v deg/s on the Arduino.
    """
    angle_val = int(round(angle))
    if speed is not None and speed > 0:
        cmd = f"M {index} {angle_val} {int(round(speed))}\n"
    else:
        cmd = f"M {index} {angle_val}\n"
    ser.write(cmd.encode())


def send_home_all(ser):
    """Sends 'H' command to home all servos to center (90 deg) immediately."""
    ser.write(b"H\n")


def send_release_all(ser):
    """Sends 'O' command to release (de-energize) all servos."""
    ser.write(b"O\n")


def send_query_angles(ser):
    """Sends 'Q' command to query current angles from the Arduino."""
    ser.write(b"Q\n")


def send_print_calibration(ser):
    """Sends 'P' command to query the active calibration arrays."""
    ser.write(b"P\n")


# =============================================================================
# HELPER UTILITIES
# =============================================================================

def clamp(value, low, high):
    """Clamps a numeric value within the range [low, high]."""
    return max(low, min(high, value))


def motor_name(i):
    """Name for motor i, falling back to a generic label if unset."""
    if i < len(MOTOR_NAMES) and MOTOR_NAMES[i]:
        return MOTOR_NAMES[i]
    return f"motor {i}"


class EdgeDetector:
    """Fires once per key press (rising edge), not while the key is held."""

    def __init__(self, keys):
        self._prev = {k: False for k in keys}

    def pressed(self, key):
        now = keyboard.is_pressed(key)
        fired = now and not self._prev[key]
        self._prev[key] = now
        return fired


def save_angles(angles, speed):
    """Write the current angles to a new timestamped file and return its path."""
    os.makedirs(SAVE_DIR, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(SAVE_DIR, f"angles_{stamp}.txt")

    rounded = [int(round(a)) for a in angles]
    name_width = max(len(motor_name(i)) for i in range(len(rounded)))
    with open(path, "w", encoding="utf-8") as f:
        f.write("# Saved motor angles\n")
        f.write(f"# timestamp : {datetime.now().isoformat(timespec='seconds')}\n")
        f.write(f"# speed     : {speed:.0f} deg/s\n")
        for i, a in enumerate(rounded):
            f.write(f"motor {i} ({motor_name(i):<{name_width}}) : {a} deg\n")
        f.write("\n")
        # Convenient copy-paste forms.
        f.write("names  = [" + ", ".join(f'"{motor_name(i)}"' for i in range(len(rounded))) + "]\n")
        f.write("angles = [" + ", ".join(str(a) for a in rounded) + "]\n")
        f.write("S " + " ".join(str(a) for a in rounded) + "\n")
    return path


def _parse_numbers(text):
    """Pull every number out of a line, ignoring commas, brackets and labels."""
    out = []
    for tok in text.replace(",", " ").replace("[", " ").replace("]", " ").split():
        try:
            out.append(float(tok))
        except ValueError:
            pass
    return out


def load_angle_set(slot):
    """Read saved_angles/angles_<slot>.txt -> list of SERVO_COUNT angles, or None."""
    path = os.path.join(SAVE_DIR, f"angles_{slot}.txt")
    if not os.path.isfile(path):
        return None
    try:
        with open(path, encoding="utf-8") as f:
            lines = f.readlines()
    except OSError:
        return None
    for line in lines:
        s = line.strip()
        # The "angles = [...]" or "S ..." line both carry the values.
        if s.startswith("angles") and "=" in s:
            nums = _parse_numbers(s.split("=", 1)[1])
        elif s.startswith("S "):
            nums = _parse_numbers(s[1:])
        else:
            continue
        if len(nums) >= SERVO_COUNT:
            return [clamp(n, ANGLE_MIN, ANGLE_MAX) for n in nums[:SERVO_COUNT]]
    return None


def available_slots():
    """Slot numbers (1..9) that have a saved_angles/angles_<n>.txt file."""
    return [d for d in range(1, 10)
            if os.path.isfile(os.path.join(SAVE_DIR, f"angles_{d}.txt"))]


def print_saved_sets():
    """Displays all available saved angle sets."""
    slots = available_slots()
    if not slots:
        print("\n(no saved sets found in saved_angles/ as angles_<n>.txt)")
        return
    print("\nSaved sets (press the number to move there):")
    for d in slots:
        vals = load_angle_set(d)
        shown = " ".join(str(int(round(v))) for v in vals) if vals else "?"
        print(f"   {d} : angles_{d}.txt -> [{shown}]")


def print_banner(port, speed):
    """Prints a single comprehensive string guide for keyboard control bindings."""
    name_width = max(len(motor_name(i)) for i in range(SERVO_COUNT))
    motor_lines = [
        f"   {i}  {motor_name(i):<{name_width}}     {CW_KEYS[i]}       {CCW_KEYS[i]}"
        for i in range(SERVO_COUNT)
    ]
    motor_lines_str = "\n".join(motor_lines)
    
    banner = f"""{"=" * 60}
 {SERVO_COUNT}-Servo Keyboard Control
{"=" * 60}
 Port: {port}   Baud: {BAUD}

 Hold a key to rotate; hold several at once to move several motors.
   #  {"name":<{name_width}}   CW(+)   CCW(-)
{motor_lines_str}

 {'/'.join(SPEED_UP_KEYS)} : speed up      {'/'.join(SPEED_DOWN_KEYS)} : speed down
 {SAVE_KEY:<5}: save angles to a new file
 1-9  : move to saved set saved_angles/angles_<n>.txt
 {LIST_KEY:<5}: list the saved sets
 {HOME_KEY:<5}: home all motors to center (90 deg)
 {OFF_KEY:<5}: release (power off) all motors
 {QUIT_KEY:<5}: quit
{"=" * 60}
 Speed: {speed:.0f} deg/s
"""
    print(banner)


# =============================================================================
# KEYBOARD CONTROL APPLICATION CLASS
# =============================================================================

class KeyboardControllerApp:
    """Manages the keyboard inputs, local position updates, and states."""

    def __init__(self, ser):
        self.ser = ser
        self.angles = [START_ANGLE] * SERVO_COUNT
        self.last_sent = [None] * SERVO_COUNT
        self.speed = SPEED_DEG_PER_S
        self.target = None
        
        self.edges = EdgeDetector(
            SPEED_UP_KEYS + SPEED_DOWN_KEYS
            + [SAVE_KEY, HOME_KEY, OFF_KEY, LIST_KEY]
            + LOAD_SLOT_KEYS
        )
        self.last_status_time = 0.0

    def initialize_pose(self):
        """Starts all servos at a known home position on setup."""
        send_home_all(self.ser)
        self.last_sent[:] = [int(round(START_ANGLE))] * SERVO_COUNT

    def process_one_shot_keys(self):
        """Processes key presses that trigger immediate one-shot actions (non-held)."""
        # Speed controls
        for k in SPEED_UP_KEYS:
            if self.edges.pressed(k):
                self.speed = clamp(self.speed + SPEED_STEP, SPEED_MIN, SPEED_MAX)
        for k in SPEED_DOWN_KEYS:
            if self.edges.pressed(k):
                self.speed = clamp(self.speed - SPEED_STEP, SPEED_MIN, SPEED_MAX)

        # Home trigger
        if self.edges.pressed(HOME_KEY):
            self.target = None
            self.angles = [START_ANGLE] * SERVO_COUNT
            send_home_all(self.ser)
            self.last_sent[:] = [int(round(START_ANGLE))] * SERVO_COUNT

        # Off trigger
        if self.edges.pressed(OFF_KEY):
            self.target = None
            send_release_all(self.ser)
            self.last_sent[:] = [None] * SERVO_COUNT

        # Save trigger
        if self.edges.pressed(SAVE_KEY):
            path = save_angles(self.angles, self.speed)
            print(f"\n[saved] {path}")

        # List files trigger
        if self.edges.pressed(LIST_KEY):
            print_saved_sets()

        # Load slot pose triggers (1-9)
        for slot in LOAD_SLOT_KEYS:
            if self.edges.pressed(slot):
                loaded = load_angle_set(slot)
                if loaded is None:
                    print(f"\n[set {slot}] angles_{slot}.txt not found / unreadable")
                else:
                    self.target = loaded
                    print(f"\n[set {slot}] moving to {[int(round(v)) for v in loaded]}")
                    # Offload non-blocking speed move to Arduino
                    for i in range(SERVO_COUNT):
                        send_move_servo(self.ser, i, self.target[i], self.speed)

    def update_motion(self, dt):
        """Integrates manual inputs and simulates auto-ramping targets."""
        manual_held = any(
            keyboard.is_pressed(CW_KEYS[i]) or keyboard.is_pressed(CCW_KEYS[i])
            for i in range(SERVO_COUNT)
        )
        
        # Intercept and cancel auto-movement if user presses manual buttons
        if self.target is not None and manual_held:
            self.target = None
            send_set_all_angles(self.ser, self.angles)
            self.last_sent[:] = [int(round(a)) for a in self.angles]

        if self.target is not None:
            # Local pose tracking of active target ramp
            step = self.speed * dt
            reached = True
            for i in range(SERVO_COUNT):
                diff = self.target[i] - self.angles[i]
                if abs(diff) <= step:
                    self.angles[i] = self.target[i]
                else:
                    self.angles[i] += step if diff > 0 else -step
                    reached = False
            if reached:
                self.target = None
        else:
            # Manual keyboard controls integration
            for i in range(SERVO_COUNT):
                delta = 0.0
                if keyboard.is_pressed(CW_KEYS[i]):
                    delta += self.speed * dt
                if keyboard.is_pressed(CCW_KEYS[i]):
                    delta -= self.speed * dt
                if delta != 0.0:
                    self.angles[i] = clamp(self.angles[i] + delta, ANGLE_MIN, ANGLE_MAX)
            
            # Send updated poses immediately
            rounded = [int(round(a)) for a in self.angles]
            if rounded != self.last_sent:
                send_set_all_angles(self.ser, rounded)
                self.last_sent[:] = rounded

    def print_status(self, now):
        """Prints live controller status in stdout."""
        if now - self.last_status_time > 0.15:
            name_w = max(len(motor_name(i)) for i in range(SERVO_COUNT))
            shown = " | ".join(
                f"{motor_name(i):<{name_w}}:{self.angles[i]:6.1f}" for i in range(SERVO_COUNT)
            )
            tag = "  [-> set]" if self.target is not None else ""
            line = f"speed {self.speed:5.0f} deg/s | {shown}{tag}"
            print("\r" + line.ljust(118), end="", flush=True)
            self.last_status_time = now


# =============================================================================
# MAIN RUNTIME SKELETON
# =============================================================================

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT

    # Serial interface initialization
    try:
        ser = serial.Serial(port, BAUD, timeout=1)
    except serial.SerialException as exc:
        sys.exit(f"Could not open serial port {port}: {exc}")

    time.sleep(2.0)  # Wait for the Arduino reset
    ser.reset_input_buffer()

    # Create app instance
    app = KeyboardControllerApp(ser)
    
    # UI display
    print_banner(port, app.speed)
    print_saved_sets()
    print("")

    # Initialize motor position
    app.initialize_pose()

    period = 1.0 / LOOP_HZ
    last_time = time.perf_counter()

    # Main Execution Loop
    try:
        while True:
            now = time.perf_counter()
            dt = now - last_time
            last_time = now

            if keyboard.is_pressed(QUIT_KEY):
                break

            app.process_one_shot_keys()
            app.update_motion(dt)
            app.print_status(now)

            # Keep frequency regulated to LOOP_HZ
            elapsed = time.perf_counter() - now
            if elapsed < period:
                time.sleep(period - elapsed)

    except KeyboardInterrupt:
        pass
    finally:
        print("\nReleasing motors and closing port...")
        try:
            send_release_all(ser)
            time.sleep(0.1)
        finally:
            ser.close()


if __name__ == "__main__":
    main()
