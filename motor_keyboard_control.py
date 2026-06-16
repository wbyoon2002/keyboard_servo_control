"""
Keyboard-driven 5-servo motor control.

Pairs with the firmware in:
    arduino_motor_control/keyboard_motor_control/keyboard_motor_control.ino

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

This script owns all motion logic: at a fixed rate it integrates the held
keys into target angles and streams them to the Arduino as absolute
positions, so the firmware stays a thin, safe actuator.

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
# Configuration
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
#
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


def clamp(value, low, high):
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
        f.write(f"# Saved motor angles\n")
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
    name_width = max(len(motor_name(i)) for i in range(SERVO_COUNT))
    print("=" * 60)
    print(f" {SERVO_COUNT}-Servo Keyboard Control")
    print("=" * 60)
    print(f" Port: {port}   Baud: {BAUD}")
    print("")
    print(" Hold a key to rotate; hold several at once to move several motors.")
    print(f"   #  {'name':<{name_width}}   CW(+)   CCW(-)")
    for i in range(SERVO_COUNT):
        print(f"   {i}  {motor_name(i):<{name_width}}     {CW_KEYS[i]}       {CCW_KEYS[i]}")
    print("")
    print(f" {'/'.join(SPEED_UP_KEYS)} : speed up      {'/'.join(SPEED_DOWN_KEYS)} : speed down")
    print(f" {SAVE_KEY:<5}: save angles to a new file")
    print(f" 1-9  : move to saved set saved_angles/angles_<n>.txt")
    print(f" {LIST_KEY:<5}: list the saved sets")
    print(f" {HOME_KEY:<5}: home all motors to center (90 deg)")
    print(f" {OFF_KEY:<5}: release (power off) all motors")
    print(f" {QUIT_KEY:<5}: quit")
    print("=" * 60)
    print(f" Speed: {speed:.0f} deg/s")
    print("")


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT

    try:
        ser = serial.Serial(port, BAUD, timeout=1)
    except serial.SerialException as exc:
        sys.exit(f"Could not open serial port {port}: {exc}")

    time.sleep(2.0)  # wait for the Arduino to reset after the port opens
    ser.reset_input_buffer()

    angles = [START_ANGLE] * SERVO_COUNT
    last_sent = [None] * SERVO_COUNT
    speed = SPEED_DEG_PER_S
    target = None  # when set, the saved pose we are ramping toward

    edges = EdgeDetector(
        SPEED_UP_KEYS + SPEED_DOWN_KEYS
        + [SAVE_KEY, HOME_KEY, OFF_KEY, LIST_KEY]
        + LOAD_SLOT_KEYS
    )

    def send_angles():
        rounded = [int(round(a)) for a in angles]
        if rounded != last_sent:
            ser.write(("S " + " ".join(str(a) for a in rounded) + "\n").encode())
            last_sent[:] = rounded

    print_banner(port, speed)
    print_saved_sets()
    print("")

    # Start from a known pose so software angles match the hardware.
    ser.write(b"H\n")
    last_sent[:] = [int(round(START_ANGLE))] * SERVO_COUNT

    period = 1.0 / LOOP_HZ
    last_time = time.perf_counter()
    last_status = 0.0

    try:
        while True:
            now = time.perf_counter()
            dt = now - last_time
            last_time = now

            if keyboard.is_pressed(QUIT_KEY):
                break

            # --- one-shot keys ------------------------------------------------
            for k in SPEED_UP_KEYS:
                if edges.pressed(k):
                    speed = clamp(speed + SPEED_STEP, SPEED_MIN, SPEED_MAX)
            for k in SPEED_DOWN_KEYS:
                if edges.pressed(k):
                    speed = clamp(speed - SPEED_STEP, SPEED_MIN, SPEED_MAX)

            if edges.pressed(HOME_KEY):
                target = None
                angles = [START_ANGLE] * SERVO_COUNT
                ser.write(b"H\n")
                last_sent[:] = [int(round(START_ANGLE))] * SERVO_COUNT

            if edges.pressed(OFF_KEY):
                target = None
                ser.write(b"O\n")
                last_sent[:] = [None] * SERVO_COUNT  # force a resend on next move

            if edges.pressed(SAVE_KEY):
                path = save_angles(angles, speed)
                print(f"\n[saved] {path}")

            if edges.pressed(LIST_KEY):
                print_saved_sets()

            # --- load a saved angle set (number keys) ------------------------
            for slot in LOAD_SLOT_KEYS:
                if edges.pressed(slot):
                    loaded = load_angle_set(slot)
                    if loaded is None:
                        print(f"\n[set {slot}] angles_{slot}.txt not found / unreadable")
                    else:
                        target = loaded
                        print(f"\n[set {slot}] moving to {[int(round(v)) for v in loaded]}")

            # --- motion: ramp toward a loaded set, else manual control -------
            manual_held = any(
                keyboard.is_pressed(CW_KEYS[i]) or keyboard.is_pressed(CCW_KEYS[i])
                for i in range(SERVO_COUNT)
            )
            if target is not None and manual_held:
                target = None  # touching any motor key cancels the auto-move

            if target is not None:
                step = speed * dt
                reached = True
                for i in range(SERVO_COUNT):
                    diff = target[i] - angles[i]
                    if abs(diff) <= step:
                        angles[i] = target[i]
                    else:
                        angles[i] += step if diff > 0 else -step
                        reached = False
                if reached:
                    target = None
            else:
                for i in range(SERVO_COUNT):
                    delta = 0.0
                    if keyboard.is_pressed(CW_KEYS[i]):
                        delta += speed * dt
                    if keyboard.is_pressed(CCW_KEYS[i]):
                        delta -= speed * dt
                    if delta != 0.0:
                        angles[i] = clamp(angles[i] + delta, ANGLE_MIN, ANGLE_MAX)

            send_angles()

            # --- live status line --------------------------------------------
            if now - last_status > 0.15:
                name_w = max(len(motor_name(i)) for i in range(SERVO_COUNT))
                shown = " | ".join(
                    f"{motor_name(i):<{name_w}}:{angles[i]:6.1f}" for i in range(SERVO_COUNT)
                )
                tag = "  [-> set]" if target is not None else ""
                line = f"speed {speed:5.0f} deg/s | {shown}{tag}"
                print("\r" + line.ljust(118), end="", flush=True)
                last_status = now

            elapsed = time.perf_counter() - now
            if elapsed < period:
                time.sleep(period - elapsed)

    except KeyboardInterrupt:
        pass
    finally:
        print("\nReleasing motors and closing port...")
        try:
            ser.write(b"O\n")
            time.sleep(0.1)
        finally:
            ser.close()


if __name__ == "__main__":
    main()
