"""
Terminal-driven Motor Control and Macro Execution Tool.

This script allows interactive control of the 5-servo robotic arm through
terminal input. It supports direct hardware commands, relative movements,
and extensible macros (sequences of movements with timing and speed control).

Pairs with the firmware in:
    firmware/keyboard_motor_control/keyboard_motor_control.ino

Usage:
    python terminal_control.py            # uses default port COM3
    python terminal_control.py COM5       # override the port
"""

import os
import sys
import time
import serial

# --------------------------------------------------------------------------
# Configuration
# --------------------------------------------------------------------------
DEFAULT_PORT = "COM3"
BAUD = 115200
# Motor Channel Constants
ATOP = 0
WHEEL = 1
ABOT = 2
ALEFT = 3
ARIGHT = 4

SERVO_COUNT = 5
MOTOR_NAMES = ["arm_top", "arm_wheel", "arm_bottom", "arm_left", "arm_right"]
START_ANGLE = 90.0
ANGLE_MIN = 0.0
ANGLE_MAX = 180.0

# =============================================================================
# PREDEFINED MACROS
# =============================================================================
MACROS = {
    # "wave": [
    #     {"type": "move", "index": ATOP, "angle": 50, "speed": 120},
    #     {"type": "delay", "seconds": 0.8},
    #     {"type": "move", "index": ATOP, "angle": 130, "speed": 120},
    #     {"type": "delay", "seconds": 0.8},
    #     {"type": "move", "index": ATOP, "angle": 90, "speed": 90},
    #     {"type": "delay", "seconds": 0.5},
    #     {"type": "home"}
    # ],
    # "nod": [
    #     {"type": "move", "index": ALEFT, "angle": 60, "speed": 80},
    #     {"type": "move", "index": ARIGHT, "angle": 120, "speed": 80},
    #     {"type": "delay", "seconds": 1.0},
    #     {"type": "move", "index": ALEFT, "angle": 120, "speed": 80},
    #     {"type": "move", "index": ARIGHT, "angle": 60, "speed": 80},
    #     {"type": "delay", "seconds": 1.0},
    #     {"type": "home"}
    # ],
    # "salute": [
    #     {"type": "set_all", "angles": [90, 45, 135, 90, 90]},
    #     {"type": "delay", "seconds": 1.5},
    #     {"type": "home"}
    # ],
    "aleft_hold": [
        {"type": "move", "index": ALEFT, "angle": 135, "speed": 10}
    ],
    "aleft_release": [
        {"type": "move", "index": ALEFT, "angle": 90, "speed": 10}
    ],
    "aright_hold": [
        {"type": "move", "index": ARIGHT, "angle": 45, "speed": 10}
    ],
    "aright_release": [
        {"type": "move", "index": ARIGHT, "angle": 90, "speed": 10}
    ],
    "roll_home": [
        {"type": "move", "index": ABOT, "angle": 0, "speed": 15},
        {"type": "move", "index": ATOP, "angle": 90, "speed": 15},
        {"type": "move", "index": WHEEL, "angle": 180, "speed": 15},
    ],
    "roll_right": [
        {"type": "move", "index": ABOT, "angle": 0, "speed": 15},
        {"type": "move", "index": ATOP, "angle": 135, "speed": 15},
        {"type": "move", "index": WHEEL, "angle": 0, "speed": 15},
        {"type": "delay", "seconds": 1.0},
        {"type": "relative", "index": ATOP, "angle": 15, "speed": 15},
        {"type": "move", "index": WHEEL, "angle": 90, "speed": 10},
        {"type": "delay", "seconds": 1.0},
        {"type": "move", "index": ABOT, "angle": 60, "speed": 15},
    ],
    "flip_right": [
        {"type": "move", "index": ALEFT, "angle": 90, "speed": 15},
        {"type": "relative", "index": ATOP, "angle": -15, "speed": 15},
        {"type": "relative", "index": ABOT, "angle": 120, "speed": 15},
        {"type": "delay", "seconds": 1.0},
        {"type": "move", "index": ALEFT, "angle": 135, "speed": 15}
    ],
    "roll": [
        # initialize
        {"type": "run", "macro": "aleft_hold"},
        {"type": "run", "macro": "aright_hold"},
        # {"type": "move", "index": ARIGHT, "angle": 52, "speed": 20},
        {"type": "move", "index": ABOT, "angle": 0, "speed": 20},
        {"type": "move", "index": WHEEL, "angle": 180, "speed": 20},
        {"type": "delay", "seconds": 5.0},


        # ready
        {"type": "relative", "index": ATOP, "angle": 55, "speed": 20},
        {"type": "run", "macro": "aleft_release"},
        # {"type": "run", "macro": "aright_release"},
        {"type": "delay", "seconds": 5.0},


        # catch
        {"type": "relative", "index": WHEEL, "angle": -60, "speed": 20},
        {"type": "delay", "seconds": 5.0},
        {"type": "relative", "index": ABOT, "angle": 60, "speed": 20},
        {"type": "delay", "seconds": 5.0},
        {"type": "relative", "index": ATOP, "angle": -5, "speed": 20},
        {"type": "relative", "index": WHEEL, "angle": -90, "speed": 20},
        {"type": "relative", "index": ABOT, "angle": 90, "speed": 30},
        {"type": "delay", "seconds": 5.0},
        {"type": "run", "macro": "aleft_hold"},
        {"type": "run", "macro": "roll_home"},


    ]
}


# =============================================================================
# ANGLE OFFSET CALIBRATION
# =============================================================================

class AngleOffsetManager:
    """Manages offset mapping between logical user angles and physical servo angles.
    Physical Angle = Logical Angle + Offset.
    For motor 0 (ATOP), logical 90 deg maps to physical 55 deg.
    So, offset = 55 - 90 = -35.
    """
    def __init__(self):
        self.offsets = [80.0 - 90.0, 0.0, 0.0, 0.0, 0.0]

    def to_physical(self, index, logical_angle):
        return logical_angle + self.offsets[index]

    def to_logical(self, index, physical_angle):
        return physical_angle - self.offsets[index]

offset_manager = AngleOffsetManager()


# =============================================================================
# SERIAL COMMUNICATIONS COMMANDS (Request-Response Pattern)
# =============================================================================

def send_raw_command(ser, cmd):
    """Sends a raw command string to the Arduino, clears the input buffer first
    to prevent synchronization mismatch, and returns the response line.
    """
    if not cmd.endswith('\n'):
        cmd += '\n'
    ser.reset_input_buffer()
    ser.write(cmd.encode())
    
    # Read response line (blocks up to serial timeout, which is 1s)
    response = ser.readline().decode().strip()
    return response


def send_set_all_angles(ser, angles):
    """Sends 'S a0 a1 a2 a3 a4' command to set all servo angles immediately."""
    physical_angles = [clamp(offset_manager.to_physical(i, a), ANGLE_MIN, ANGLE_MAX) for i, a in enumerate(angles)]
    cmd = f"S {' '.join(str(int(round(a))) for a in physical_angles)}"
    print(f"-> Sent: {cmd}")
    resp = send_raw_command(ser, cmd)
    print(f"<- Arduino: {resp}")


def send_move_servo(ser, index, angle, speed=None):
    """Sends 'M i a [v]' command to move a single servo."""
    phys_angle = clamp(offset_manager.to_physical(index, angle), ANGLE_MIN, ANGLE_MAX)
    if speed is not None and speed > 0:
        cmd = f"M {index} {int(round(phys_angle))} {int(round(speed))}"
    else:
        cmd = f"M {index} {int(round(phys_angle))}"
    print(f"-> Sent: {cmd}")
    resp = send_raw_command(ser, cmd)
    print(f"<- Arduino: {resp}")


def send_home_all(ser):
    """Sends 'H' command to home all servos to center (90 deg) immediately."""
    print("-> Sent: Home (H)")
    resp = send_raw_command(ser, "H")
    print(f"<- Arduino: {resp}")


def send_release_all(ser):
    """Sends 'O' command to release (de-energize) all servos."""
    print("-> Sent: Off/Release (O)")
    resp = send_raw_command(ser, "O")
    print(f"<- Arduino: {resp}")


def send_query_angles(ser):
    """Sends 'Q' command to query current angles from the Arduino."""
    print("-> Sent: Query (Q)")
    resp = send_raw_command(ser, "Q")
    if resp.startswith("A "):
        try:
            parts = resp.split()[1:]
            logical = [int(round(offset_manager.to_logical(i, float(p)))) for i, p in enumerate(parts)]
            print(f"<- Arduino (Logical): A {' '.join(str(l) for l in logical)}")
        except Exception:
            print(f"<- Arduino: {resp}")
    else:
        print(f"<- Arduino: {resp}")


def send_print_calibration(ser):
    """Sends 'P' command to query active calibration."""
    print("-> Sent: Print Cal (P)")
    ser.reset_input_buffer()
    ser.write(b"P\n")
    # Calibration print returns exactly 3 lines of output
    for _ in range(3):
        line = ser.readline().decode().strip()
        print(f"<- Arduino: {line}")


# =============================================================================
# TERMINAL APP STATE & LOGIC
# =============================================================================

class TerminalControllerApp:
    """Manages serial execution, active target angles tracking, and macro execution."""

    def __init__(self, ser):
        self.ser = ser
        # Track estimated current angles locally to compute relative moves
        self.angles = [START_ANGLE] * SERVO_COUNT

    def move_absolute(self, index, target_angle, speed=None):
        """Moves a single servo to an absolute target angle (clamped)."""
        target = clamp(target_angle, ANGLE_MIN, ANGLE_MAX)
        send_move_servo(self.ser, index, target, speed)
        self.angles[index] = target

    def move_relative(self, index, delta_angle, speed=None):
        """Moves a single servo relative to its current tracked position (clamped)."""
        target = clamp(self.angles[index] + delta_angle, ANGLE_MIN, ANGLE_MAX)
        print(f"   [Relative] Current: {self.angles[index]:.1f}° | Delta: {delta_angle:+.1f}° -> Target: {target:.1f}°")
        send_move_servo(self.ser, index, target, speed)
        self.angles[index] = target

    def set_all(self, target_angles):
        """Sets all servo angles to target values immediately (clamped)."""
        clamped_angles = [clamp(a, ANGLE_MIN, ANGLE_MAX) for a in target_angles]
        send_set_all_angles(self.ser, clamped_angles)
        self.angles[:] = clamped_angles

    def home(self):
        """Homes all servos to logical center (respecting physical offsets)."""
        self.set_all([START_ANGLE] * SERVO_COUNT)

    def release(self):
        """Power off/de-energize all servos."""
        send_release_all(self.ser)

    def run_macro(self, macro_name):
        """Executes a pre-defined macro sequence of motions and delays, updating state."""
        if macro_name not in MACROS:
            print(f"Error: Macro '{macro_name}' not found.")
            return

        print(f"\n[*] Executing macro: '{macro_name}'")
        steps = MACROS[macro_name]
        
        for idx, step in enumerate(steps):
            stype = step.get("type")
            print(f"  [Step {idx+1}/{len(steps)}] {stype.upper()}...", end="", flush=True)

            if stype == "move":
                idx_val = step["index"]
                angle_val = clamp(step["angle"], ANGLE_MIN, ANGLE_MAX)
                send_move_servo(self.ser, idx_val, angle_val, step.get("speed"))
                self.angles[idx_val] = angle_val
            elif stype == "relative":
                idx_val = step["index"]
                angle_val = clamp(self.angles[idx_val] + step.get("angle", 0.0), ANGLE_MIN, ANGLE_MAX)
                send_move_servo(self.ser, idx_val, angle_val, step.get("speed"))
                self.angles[idx_val] = angle_val
            elif stype == "run":
                sub_macro = step.get("macro") or step.get("name")
                if sub_macro:
                    print(f" invoking macro '{sub_macro}'")
                    self.run_macro(sub_macro)
                continue
            elif stype == "set_all":
                target_angles = [clamp(a, ANGLE_MIN, ANGLE_MAX) for a in step["angles"]]
                send_set_all_angles(self.ser, target_angles)
                self.angles[:] = target_angles
            elif stype == "home":
                send_home_all(self.ser)
                self.angles[:] = [START_ANGLE] * SERVO_COUNT
            elif stype == "release":
                send_release_all(self.ser)
            elif stype == "delay":
                delay_sec = step["seconds"]
                print(f" delaying {delay_sec}s")
                time.sleep(delay_sec)
                continue
            
            print(" done")
            time.sleep(0.02)
            
        print(f"[*] Macro '{macro_name}' execution completed.\n")


# =============================================================================
# USER INTERFACE & HELPERS
# =============================================================================

def clamp(value, low, high):
    """Clamps a numeric value within the range [low, high]."""
    return max(low, min(high, value))


def print_help():
    """Prints single comprehensive help screen with all command options."""
    macros_list = ", ".join(f"'{m}'" for m in MACROS.keys())
    help_text = f"""
======================================================================
                     Terminal Servo Controller
======================================================================
Command Options:
  H                   - Home all servos to center (90 degrees)
  O                   - Off/Release all servos (de-energize)
  Q                   - Query current angles from Arduino
  P                   - Print active calibration arrays
  M <i> <a> [<v>]     - Move single servo index <i> (0-4) to absolute angle <a>
                        with optional speed <v> (degrees/second)
  R <i> <a> [<v>]     - Move single servo index <i> (0-4) relatively by <a> degrees
                        with optional speed <v> (degrees/second)
  S <a0>..<a4>        - Set all 5 servo angles immediately (space separated)
  
Macro Commands:
  run <macro_name>    - Runs a macro. Available macros: {macros_list}
  list                - Displays all available macros
  
System Commands:
  help / ?            - Show this help menu
  exit / quit         - Exit the application
======================================================================
"""
    print(help_text)


def print_available_macros():
    """Prints details of all currently configured macros."""
    print("\nAvailable Macros:")
    for name, steps in MACROS.items():
        print(f"  - '{name}' ({len(steps)} steps)")
    print()


# =============================================================================
# MAIN INTERACTIVE CLI
# =============================================================================

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT

    try:
        ser = serial.Serial(port, BAUD, timeout=1)
    except serial.SerialException as exc:
        sys.exit(f"Could not open serial port {port}: {exc}")

    print("Connecting to Arduino...")
    time.sleep(2.0)  # Wait for Arduino auto-reset
    ser.reset_input_buffer()
    
    # Check if we can read READY message
    if ser.in_waiting > 0:
        while ser.in_waiting > 0:
            print(f"<- Arduino: {ser.readline().decode().strip()}")

    # Initialize Controller App State
    app = TerminalControllerApp(ser)

    print_help()

    try:
        while True:
            # Prompt user for command input
            user_input = input("ServoCMD> ").strip()
            if not user_input:
                continue

            parts = user_input.split()
            cmd = parts[0].upper()

            # Direct exit commands
            if cmd in ["EXIT", "QUIT"]:
                break

            # Help commands
            elif cmd in ["HELP", "?"]:
                print_help()
                continue

            # Predefined Macro display
            elif cmd == "LIST":
                print_available_macros()
                continue

            # Running macros
            elif cmd == "RUN":
                if len(parts) < 2:
                    print("Error: Specify macro name (e.g., 'run wave')")
                    continue
                app.run_macro(parts[1].lower())
                continue

            # Command 'H'
            elif cmd == "H":
                app.home()

            # Command 'O'
            elif cmd == "O":
                app.release()

            # Command 'Q'
            elif cmd == "Q":
                send_query_angles(ser)

            # Command 'P'
            elif cmd == "P":
                send_print_calibration(ser)

            # Command 'M': Single servo absolute move
            elif cmd == "M":
                if len(parts) < 3:
                    print("Error: Command format is 'M <i> <a> [<v>]'")
                    continue
                try:
                    idx = int(parts[1])
                    ang = float(parts[2])
                    speed = float(parts[3]) if len(parts) > 3 else None
                    if idx < 0 or idx >= SERVO_COUNT:
                        raise ValueError("Index out of bounds")
                except ValueError as e:
                    print(f"Error: Invalid parameters for M. {e}")
                    continue
                app.move_absolute(idx, ang, speed)

            # Command 'R': Single servo relative move
            elif cmd == "R":
                if len(parts) < 3:
                    print("Error: Command format is 'R <i> <a> [<v>]'")
                    continue
                try:
                    idx = int(parts[1])
                    delta = float(parts[2])
                    speed = float(parts[3]) if len(parts) > 3 else None
                    if idx < 0 or idx >= SERVO_COUNT:
                        raise ValueError("Index out of bounds")
                except ValueError as e:
                    print(f"Error: Invalid parameters for R. {e}")
                    continue
                app.move_relative(idx, delta, speed)

            # Command 'S': Set all servos
            elif cmd == "S":
                if len(parts) < 6:
                    print("Error: S requires 5 target angles (e.g., 'S 90 90 90 90 90')")
                    continue
                try:
                    angles = [float(a) for a in parts[1:6]]
                except ValueError:
                    print("Error: All angles must be valid numbers")
                    continue
                app.set_all(angles)

            else:
                print(f"Unknown command: '{parts[0]}'. Type 'help' or '?' for options.")

    except (KeyboardInterrupt, EOFError):
        print("\nExiting terminal control...")
    finally:
        # Safe cleanup
        print("Releasing motors and closing serial port...")
        try:
            send_release_all(ser)
            time.sleep(0.1)
        finally:
            ser.close()


if __name__ == "__main__":
    main()
