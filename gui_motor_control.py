"""
GUI-driven 5-servo motor control (on-screen buttons -- no keyboard needed).

Pairs with the firmware in:
    firmware/keyboard_motor_control/keyboard_motor_control.ino

This is the on-screen-button replacement for the live keyboard tool
(motor_keyboard_control.py). Instead of holding keys, you drive the arm from a
window:

1. Tick the motors you want to control, pick a DIRECTION (+/-) and a SPEED for
   each, then HOLD the big "MOVE" button. Every selected motor moves at the
   same time, each in its own direction at its own speed. Release to stop.

2. Or set a TARGET angle per motor and press "GO TO TARGET" -- all selected
   motors travel to their targets simultaneously, with the smooth speed
   interpolation handled on the Arduino.

3. Home / Release / Stop-hold, save the current pose to a new file, and recall
   saved poses (slots 1-9) -- same features as the keyboard tool.

Requirements
------------
    pip install pyserial
(Tkinter ships with the standard Python installer, so nothing else is needed.
 Unlike motor_keyboard_control.py this tool does NOT need the `keyboard`
 library or administrator rights.)

Usage
-----
    python gui_motor_control.py            # uses default port COM3
    python gui_motor_control.py COM5       # override the port
"""

import os
import sys
import time
import json
from datetime import datetime

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext, simpledialog

try:
    import serial  # pyserial
except ImportError:
    serial = None


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

# What the "+" direction does in real life for each motor (motors 0..4). "+"
# always increases the logical angle; the physical effect depends on the build.
DIRECTION_HINTS = [
    "+ CW / - CCW",
    "+ CW / - CCW",
    "+ CW / - CCW",
    "+ drop / - lift",
    "+ drop / - lift",
]

# --- per-motor angle calibration (logical GUI angle -> physical servo angle) ---
# Two tunable knobs, applied in order: first DIRECTION, then OFFSET.
#
#   physical = (INVERT[i] ? 180 - logical : logical) + OFFSET[i]
#
# INVERT mirrors a servo that spins the "wrong" way so it obeys the same +/-
# convention as the rest (motors 1, 2 and 4 are reversed on this build).
# OFFSET is a zero-point shift in degrees, exactly like terminal_control.py's
# AngleOffsetManager (physical = logical + offset). For example, set
# OFFSET[0] = -10 to reproduce terminal_control.py's motor-0 calibration.
INVERT = [False, True, True, False, True]
OFFSET = [-10.0, 0.0, 0.0, 0.0, 0.0]

START_ANGLE = 90.0             # center; matches the firmware "H" command
ANGLE_MIN = 0.0
ANGLE_MAX = 180.0

SPEED_DEG_PER_S = 60.0         # initial per-motor speed
SPEED_MIN = 5.0
SPEED_MAX = 300.0
SPEED_STEP = 5.0

NUDGE_STEP = 5.0               # degrees moved by the per-motor +/- nudge buttons

LOOP_MS = 25                   # control / streaming period (~40 Hz)
SAVE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "saved_angles")
SAVE_PREFIX = "angles_"        # saved poses are <SAVE_PREFIX><name>.txt

# Macros are sequences of steps, using the same step types as terminal_control.py
# (move / relative / set_all / home / release / delay / run). They are stored as
# JSON in MACROS_FILE so they persist between sessions.
MACROS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "macros.json")
STEP_TYPES = ["move", "relative", "set_all", "delay", "home", "release", "run"]

# Seeded on first run (delete/edit freely in the editor). These are the macros
# from terminal_control.py. Because the GUI mirrors motors 1/2/4 (INVERT) while
# terminal_control.py does not, the move/relative values on motors 1, 2 and 4
# are flipped here (move: 180-angle, relative: -angle) so the GUI reproduces the
# same physical motion. Motor 0 still uses terminal's logical values -- set
# OFFSET[0] = -10 above if you also want its zero-point to match.
# Motor indices: 0=arm_top 1=arm_wheel 2=arm_bottom 3=arm_left 4=arm_right.
DEFAULT_MACROS = {
    "aleft_hold":     [{"type": "move", "index": 3, "angle": 135, "speed": 10}],
    "aleft_release":  [{"type": "move", "index": 3, "angle": 90, "speed": 10}],
    "aright_hold":    [{"type": "move", "index": 4, "angle": 135, "speed": 10}],
    "aright_release": [{"type": "move", "index": 4, "angle": 90, "speed": 10}],
    "roll_home": [
        {"type": "move", "index": 2, "angle": 180, "speed": 15},
        {"type": "move", "index": 0, "angle": 90, "speed": 15},
        {"type": "move", "index": 1, "angle": 0, "speed": 15},
    ],
    "roll_right": [
        {"type": "move", "index": 2, "angle": 180, "speed": 15},
        {"type": "move", "index": 0, "angle": 135, "speed": 15},
        {"type": "move", "index": 1, "angle": 180, "speed": 15},
        {"type": "delay", "seconds": 1.0},
        {"type": "relative", "index": 0, "angle": 15, "speed": 15},
        {"type": "move", "index": 1, "angle": 90, "speed": 10},
        {"type": "delay", "seconds": 1.0},
        {"type": "move", "index": 2, "angle": 120, "speed": 15},
    ],
    "flip_right": [
        {"type": "move", "index": 3, "angle": 90, "speed": 15},
        {"type": "relative", "index": 0, "angle": -15, "speed": 15},
        {"type": "relative", "index": 2, "angle": -120, "speed": 15},
        {"type": "delay", "seconds": 1.0},
        {"type": "move", "index": 3, "angle": 135, "speed": 15},
    ],
    "roll": [
        {"type": "run", "macro": "aleft_hold"},
        {"type": "run", "macro": "aright_hold"},
        {"type": "move", "index": 2, "angle": 180, "speed": 20},
        {"type": "move", "index": 1, "angle": 0, "speed": 20},
        {"type": "delay", "seconds": 5.0},
        {"type": "relative", "index": 0, "angle": 55, "speed": 20},
        {"type": "run", "macro": "aleft_release"},
        {"type": "delay", "seconds": 5.0},
        {"type": "relative", "index": 1, "angle": 60, "speed": 20},
        {"type": "delay", "seconds": 5.0},
        {"type": "relative", "index": 2, "angle": -60, "speed": 20},
        {"type": "delay", "seconds": 5.0},
        {"type": "relative", "index": 0, "angle": -5, "speed": 20},
        {"type": "relative", "index": 1, "angle": 90, "speed": 20},
        {"type": "relative", "index": 2, "angle": -90, "speed": 30},
        {"type": "delay", "seconds": 5.0},
        {"type": "run", "macro": "aleft_hold"},
        {"type": "run", "macro": "roll_home"},
    ],
    "demo_sequence": [
        {"type": "home"},
        {"type": "delay", "seconds": 1.0},
        {"type": "move", "index": 0, "angle": 135, "speed": 30},
        {"type": "move", "index": 2, "angle": 45, "speed": 30},
        {"type": "delay", "seconds": 1.5},
        {"type": "relative", "index": 0, "angle": -45, "speed": 20},
        {"type": "delay", "seconds": 1.0},
        {"type": "home"},
    ],
}


# =============================================================================
# SERIAL COMMUNICATIONS COMMAND FUNCTIONS
# =============================================================================

def to_physical(index, angle):
    """Map a GUI (logical) angle to the physical servo angle.
    Applies the per-motor direction (INVERT mirrors within [MIN, MAX]) and then
    the tunable OFFSET, matching terminal_control.py's offset calibration."""
    if 0 <= index < len(INVERT) and INVERT[index]:
        base = (ANGLE_MIN + ANGLE_MAX) - angle
    else:
        base = angle
    if 0 <= index < len(OFFSET):
        base += OFFSET[index]
    return base


def to_logical(index, angle):
    """Inverse of to_physical: physical servo angle -> logical GUI angle."""
    if 0 <= index < len(OFFSET):
        angle = angle - OFFSET[index]
    if 0 <= index < len(INVERT) and INVERT[index]:
        return (ANGLE_MIN + ANGLE_MAX) - angle
    return angle


def send_set_all_angles(ser, angles):
    """Sends 'S a0 a1 a2 a3 a4' to set all servo angles immediately (logical in)."""
    rounded = [int(round(to_physical(i, a))) for i, a in enumerate(angles)]
    cmd = f"S {' '.join(str(a) for a in rounded)}\n"
    ser.write(cmd.encode())


def send_move_servo(ser, index, angle, speed=None):
    """Sends 'M i a [v]' to move a single servo to a target angle (logical in).
    If speed (v) is provided, the Arduino interpolates the move at v deg/s.
    """
    angle_val = int(round(to_physical(index, angle)))
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


def _slugify(name):
    """Turn a user-typed pose name into a safe filename slug.
    Letters/digits are kept, spaces become underscores, other punctuation is
    dropped. Returns "" if nothing usable remains."""
    out = []
    for ch in name.strip():
        if ch.isalnum() or ch in "-_":
            out.append(ch)
        elif ch.isspace():
            out.append("_")
    return "".join(out).strip("_")


def save_angles(angles, speed, name=None):
    """Write the current angles to a pose file and return its path.

    If `name` is given it is saved as saved_angles/angles_<slug>.txt so it can
    be recalled by that name (re-saving the same name overwrites it). With no
    name a timestamped file is created instead.
    """
    os.makedirs(SAVE_DIR, exist_ok=True)
    display_name = (name or "").strip()
    slug = _slugify(display_name)
    if not slug:
        slug = datetime.now().strftime("%Y%m%d_%H%M%S")
        display_name = slug
    path = os.path.join(SAVE_DIR, f"{SAVE_PREFIX}{slug}.txt")

    rounded = [int(round(a)) for a in angles]
    name_width = max(len(motor_name(i)) for i in range(len(rounded)))
    with open(path, "w", encoding="utf-8") as f:
        f.write("# Saved motor angles\n")
        f.write(f"# name      : {display_name}\n")
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


def load_pose_file(path):
    """Read a saved pose file -> list of SERVO_COUNT angles, or None."""
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


def _pose_display_name(path):
    """The saved '# name :' label for a pose file, or its filename stem."""
    try:
        with open(path, encoding="utf-8") as f:
            for line in f:
                s = line.strip()
                if s.startswith("# name"):
                    parts = s.split(":", 1)
                    if len(parts) == 2 and parts[1].strip():
                        return parts[1].strip()
                # Once the data lines start, there is no name header.
                if s.startswith(("motor", "names", "angles", "S ")):
                    break
    except OSError:
        pass
    stem = os.path.basename(path)
    if stem.startswith(SAVE_PREFIX):
        stem = stem[len(SAVE_PREFIX):]
    if stem.endswith(".txt"):
        stem = stem[:-4]
    return stem


def saved_poses():
    """List (display_name, path) for every saved pose, sorted by name."""
    if not os.path.isdir(SAVE_DIR):
        return []
    poses = []
    for fname in os.listdir(SAVE_DIR):
        if fname.startswith(SAVE_PREFIX) and fname.endswith(".txt"):
            path = os.path.join(SAVE_DIR, fname)
            poses.append((_pose_display_name(path), path))
    poses.sort(key=lambda p: p[0].lower())
    return poses


# =============================================================================
# MACRO STORAGE
# =============================================================================

def save_macros(macros):
    """Write the macros dict to MACROS_FILE as JSON."""
    with open(MACROS_FILE, "w", encoding="utf-8") as f:
        json.dump(macros, f, indent=2, ensure_ascii=False)


def load_macros():
    """Load macros from MACROS_FILE, seeding an example file on first run."""
    if os.path.isfile(MACROS_FILE):
        try:
            with open(MACROS_FILE, encoding="utf-8") as f:
                data = json.load(f)
            if isinstance(data, dict):
                # Keep only name -> list-of-steps entries.
                return {str(k): list(v) for k, v in data.items() if isinstance(v, list)}
        except (OSError, ValueError):
            pass
        return {}
    try:
        save_macros(DEFAULT_MACROS)
    except OSError:
        pass
    return {name: list(steps) for name, steps in DEFAULT_MACROS.items()}


def _fmt_num(value):
    """Render a number without a trailing '.0' when it is whole."""
    try:
        f = float(value)
    except (TypeError, ValueError):
        return str(value)
    return str(int(f)) if f == int(f) else str(f)


def step_repr(step):
    """One-line human-readable rendering of a macro step (for the editor list)."""
    t = step.get("type", "?")
    if t in ("move", "relative"):
        idx = int(step.get("index", 0))
        return (f"{t:<8} m{idx} {motor_name(idx):<10} "
                f"angle={_fmt_num(step.get('angle', 0))} speed={_fmt_num(step.get('speed', 0))}")
    if t == "set_all":
        angs = " ".join(_fmt_num(a) for a in step.get("angles", []))
        return f"set_all  angles=[{angs}]"
    if t == "delay":
        return f"delay    seconds={_fmt_num(step.get('seconds', 0))}"
    if t == "run":
        return f"run      macro=\"{step.get('macro', step.get('name', ''))}\""
    return f"{t}"


# =============================================================================
# GUI APPLICATION
# =============================================================================

class ServoGuiApp:
    """Tkinter window that drives the 5 servos with on-screen controls.

    Two ways to move several motors at once:
      * JOG  -- hold the MOVE button; every enabled motor integrates in its
                chosen direction at its speed while the PC streams 'S' frames.
      * GOTO -- press GO TO TARGET; each enabled motor is sent 'M i a v' and
                the Arduino interpolates to the target (we track locally for
                the live readout).
    """

    def __init__(self, root, port):
        self.root = root
        self.port = port
        self.ser = None
        self.connected = False

        # Motion state. self.angles is our best estimate of where each motor is.
        self.angles = [START_ANGLE] * SERVO_COUNT
        self.last_sent = [None] * SERVO_COUNT
        self.jogging = False                 # True while the MOVE button is held
        self.target = None                   # list of SERVO_COUNT targets, or None
        self.last_tick = time.perf_counter()
        self._rx = ""                        # serial receive line buffer
        self._pose_paths = {}                # display name -> saved file path

        # Macro storage + non-blocking runner state.
        self.macros = load_macros()          # name -> list of step dicts
        self._macro_win = None               # the editor Toplevel, if open
        self._cur_macro = None               # name selected in the editor
        self._macro_running = False
        self._macro_steps = []               # flattened steps of the running macro
        self._macro_index = 0
        self._macro_wait_until = 0.0         # perf_counter() deadline for a delay step
        self._macro_name = ""                # name of the running macro (for logs)
        self._pose_ref = []                  # [(name, angles)] shown in the macro window

        # Per-motor Tk variables.
        self.enable_var = [tk.BooleanVar(value=(i == 0)) for i in range(SERVO_COUNT)]
        self.dir_var = [tk.IntVar(value=1) for _ in range(SERVO_COUNT)]      # +1 / -1
        self.speed_var = [tk.DoubleVar(value=SPEED_DEG_PER_S) for _ in range(SERVO_COUNT)]
        self.target_var = [tk.DoubleVar(value=START_ANGLE) for _ in range(SERVO_COUNT)]
        self.cur_var = [tk.StringVar(value="--") for _ in range(SERVO_COUNT)]
        self.conn_var = tk.StringVar(value="disconnected")
        self.pose_name_var = tk.StringVar()      # name typed in the Save box
        self.selected_pose_var = tk.StringVar()  # name chosen in the recall box

        # Macro step-editor field variables (used by the Macros window).
        self.mv_type = tk.StringVar(value="move")
        self.mv_index = tk.IntVar(value=0)
        self.mv_index_name = tk.StringVar(value=motor_name(0))
        self.mv_angle = tk.DoubleVar(value=90.0)
        self.mv_speed = tk.DoubleVar(value=30.0)
        self.mv_seconds = tk.DoubleVar(value=1.0)
        self.mv_setall = tk.StringVar(value="90 90 90 90 90")
        self.mv_runmacro = tk.StringVar(value="")
        self.mv_index.trace_add("write", lambda *_: self._update_index_name())

        self._build_ui()
        self.refresh_poses()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        # Start the periodic control/serial loop.
        self.root.after(LOOP_MS, self._tick)

    # ------------------------------------------------------------------ UI ---
    def _build_ui(self):
        self.root.title("Servo Motor Control (GUI)")
        self.root.minsize(720, 560)
        pad = {"padx": 4, "pady": 3}

        # --- connection bar -------------------------------------------------
        bar = ttk.Frame(self.root)
        bar.pack(fill="x", **pad)
        ttk.Label(bar, text="Port:").pack(side="left")
        self.port_entry = ttk.Entry(bar, width=10)
        self.port_entry.insert(0, self.port)
        self.port_entry.pack(side="left", padx=(2, 8))
        self.connect_btn = ttk.Button(bar, text="Connect", command=self.toggle_connect)
        self.connect_btn.pack(side="left")
        ttk.Label(bar, text="  Status:").pack(side="left")
        ttk.Label(bar, textvariable=self.conn_var, foreground="#b00").pack(side="left")

        ttk.Label(
            self.root,
            text=("Tick the motors to control, set direction + speed, then HOLD the "
                  "MOVE button (release to stop).\nOr set target angles and press GO "
                  "TO TARGET to drive all selected motors there at once."),
            justify="left", foreground="#444",
        ).pack(fill="x", **pad)

        # --- per-motor table ------------------------------------------------
        motors = ttk.LabelFrame(self.root, text="Motors")
        motors.pack(fill="x", **pad)

        header = ["On", "Motor", "Direction", "Speed (deg/s)", "Target (deg)", "Now", "Nudge"]
        for col, text in enumerate(header):
            ttk.Label(motors, text=text, font=("TkDefaultFont", 9, "bold")).grid(
                row=0, column=col, padx=5, pady=(4, 2))

        for i in range(SERVO_COUNT):
            self._build_motor_row(motors, i, row=i + 1)

        sel = ttk.Frame(self.root)
        sel.pack(fill="x", padx=4)
        ttk.Label(sel, text="Select:").pack(side="left")
        ttk.Button(sel, text="All", width=5,
                   command=lambda: self.set_all_enabled(True)).pack(side="left", padx=2)
        ttk.Button(sel, text="None", width=5,
                   command=lambda: self.set_all_enabled(False)).pack(side="left", padx=2)

        # --- big action buttons --------------------------------------------
        actions = ttk.Frame(self.root)
        actions.pack(fill="x", **pad)

        self.move_btn = tk.Button(
            actions, text="▶  HOLD TO MOVE  ▶",
            bg="#2e8b57", fg="white", activebackground="#246b43", activeforeground="white",
            font=("TkDefaultFont", 12, "bold"), relief="raised", height=2,
        )
        self.move_btn.pack(side="left", fill="x", expand=True, padx=(0, 4))
        # Press-and-hold: jog while the mouse button is down on this widget.
        self.move_btn.bind("<ButtonPress-1>", self._start_jog)
        self.move_btn.bind("<ButtonRelease-1>", self._stop_jog)

        self.goto_btn = tk.Button(
            actions, text="GO TO TARGET", bg="#2f6fb0", fg="white",
            activebackground="#245688", activeforeground="white",
            font=("TkDefaultFont", 11, "bold"), height=2, command=self.go_to_targets,
        )
        self.goto_btn.pack(side="left", fill="x", expand=True, padx=4)

        self.stop_btn = tk.Button(
            actions, text="■ STOP / HOLD", bg="#b03030", fg="white",
            activebackground="#882424", activeforeground="white",
            font=("TkDefaultFont", 11, "bold"), height=2, command=self.stop_hold,
        )
        self.stop_btn.pack(side="left", fill="x", expand=True, padx=(4, 0))

        # --- system buttons -------------------------------------------------
        sysrow = ttk.Frame(self.root)
        sysrow.pack(fill="x", **pad)
        ttk.Button(sysrow, text="Home (90°)", command=self.home_all).pack(side="left", padx=2)
        ttk.Button(sysrow, text="Release (torque off)", command=self.release_all).pack(side="left", padx=2)
        ttk.Button(sysrow, text="Query angles", command=self.query_angles).pack(side="left", padx=2)
        ttk.Button(sysrow, text="Macros…", command=self.open_macro_window).pack(side="left", padx=2)

        # --- save / recall poses by name -----------------------------------
        poses = ttk.LabelFrame(self.root, text="Save / recall poses by name")
        poses.pack(fill="x", **pad)

        saverow = ttk.Frame(poses)
        saverow.pack(fill="x", padx=4, pady=2)
        ttk.Label(saverow, text="Name:").pack(side="left")
        self.name_entry = ttk.Entry(saverow, textvariable=self.pose_name_var, width=26)
        self.name_entry.pack(side="left", padx=4)
        self.name_entry.bind("<Return>", lambda _e: self.save_pose())
        ttk.Button(saverow, text="Save pose", command=self.save_pose).pack(side="left", padx=2)
        ttk.Label(saverow, text="(blank = timestamped)", foreground="#888").pack(side="left", padx=4)

        loadrow = ttk.Frame(poses)
        loadrow.pack(fill="x", padx=4, pady=2)
        ttk.Label(loadrow, text="Saved:").pack(side="left")
        self.pose_combo = ttk.Combobox(loadrow, textvariable=self.selected_pose_var,
                                       state="readonly", width=26)
        self.pose_combo.pack(side="left", padx=4)
        self.pose_combo.bind("<Double-Button-1>", lambda _e: self.load_selected_pose())
        ttk.Button(loadrow, text="Move there", command=self.load_selected_pose).pack(side="left", padx=2)
        ttk.Button(loadrow, text="Delete", command=self.delete_selected_pose).pack(side="left", padx=2)
        ttk.Button(loadrow, text="Refresh", command=self.refresh_poses).pack(side="left", padx=2)

        # --- serial log -----------------------------------------------------
        logframe = ttk.LabelFrame(self.root, text="Serial log")
        logframe.pack(fill="both", expand=True, **pad)
        self.log = scrolledtext.ScrolledText(logframe, height=8, state="disabled",
                                             font=("Consolas", 9), wrap="word")
        self.log.pack(fill="both", expand=True)

    def _build_motor_row(self, parent, i, row):
        ttk.Checkbutton(parent, variable=self.enable_var[i]).grid(row=row, column=0)

        ttk.Label(parent, text=f"{i}  {motor_name(i)}").grid(
            row=row, column=1, sticky="w", padx=5)

        dirframe = ttk.Frame(parent)
        dirframe.grid(row=row, column=2, padx=5)
        ttk.Radiobutton(dirframe, text="+", variable=self.dir_var[i], value=1).pack(side="left")
        ttk.Radiobutton(dirframe, text="−", variable=self.dir_var[i], value=-1).pack(side="left")
        ttk.Label(dirframe, text=DIRECTION_HINTS[i] if i < len(DIRECTION_HINTS) else "",
                  foreground="#888", font=("TkDefaultFont", 8)).pack(side="left", padx=(6, 0))

        ttk.Spinbox(parent, from_=SPEED_MIN, to=SPEED_MAX, increment=SPEED_STEP,
                    textvariable=self.speed_var[i], width=7).grid(row=row, column=3, padx=5)

        ttk.Spinbox(parent, from_=ANGLE_MIN, to=ANGLE_MAX, increment=1,
                    textvariable=self.target_var[i], width=7).grid(row=row, column=4, padx=5)

        ttk.Label(parent, textvariable=self.cur_var[i], width=6, anchor="e").grid(
            row=row, column=5, padx=5)

        nudge = ttk.Frame(parent)
        nudge.grid(row=row, column=6, padx=5)
        ttk.Button(nudge, text="−", width=3,
                   command=lambda idx=i: self.nudge(idx, -1)).pack(side="left")
        ttk.Button(nudge, text="+", width=3,
                   command=lambda idx=i: self.nudge(idx, +1)).pack(side="left")

    # ------------------------------------------------------------ logging ---
    def _log(self, msg):
        self.log.configure(state="normal")
        self.log.insert("end", msg + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    # --------------------------------------------------------- connection ---
    def toggle_connect(self):
        if self.connected:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        if serial is None:
            messagebox.showerror("Missing dependency",
                                 "pyserial is not installed.\n\npip install pyserial")
            return
        self.port = self.port_entry.get().strip() or DEFAULT_PORT
        try:
            # timeout=0 -> non-blocking reads so the GUI never freezes.
            self.ser = serial.Serial(self.port, BAUD, timeout=0)
        except serial.SerialException as exc:
            messagebox.showerror("Serial error", f"Could not open {self.port}:\n{exc}")
            return

        self.connected = True
        self._rx = ""
        self.conn_var.set(f"connected ({self.port})")
        self.connect_btn.configure(text="Disconnect")
        self._log(f"-- opened {self.port} @ {BAUD} baud")
        # Wait for the Arduino auto-reset, then home to a known pose.
        self.root.after(2000, self._after_reset)

    def _after_reset(self):
        if not self.connected:
            return
        # Home to the offset-adjusted center once the Arduino has reset.
        self.home_all()

    def disconnect(self):
        if self.ser is not None:
            try:
                send_release_all(self.ser)
                time.sleep(0.05)
            except Exception:
                pass
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.connected = False
        self.jogging = False
        self.target = None
        self._macro_running = False
        self.conn_var.set("disconnected")
        self.connect_btn.configure(text="Connect")
        self._log("-- closed serial port")

    def _require_connection(self):
        if not self.connected:
            self._log("!! not connected -- press Connect first")
            return False
        return True

    # ---------------------------------------------------------- main loop ---
    def _tick(self):
        now = time.perf_counter()
        dt = now - self.last_tick
        self.last_tick = now

        self._drain_serial()

        if self.connected:
            if self.jogging:
                self._jog_step(dt)
            elif self.target is not None:
                self._target_step(dt)
            self._refresh_angles()

        self.root.after(LOOP_MS, self._tick)

    def _jog_step(self, dt):
        """Integrate every enabled motor in its direction and stream 'S'."""
        any_enabled = False
        for i in range(SERVO_COUNT):
            if not self.enable_var[i].get():
                continue
            any_enabled = True
            speed = self._read_speed(i)
            delta = self.dir_var[i].get() * speed * dt
            self.angles[i] = clamp(self.angles[i] + delta, ANGLE_MIN, ANGLE_MAX)
        if not any_enabled:
            return
        rounded = [int(round(a)) for a in self.angles]
        if rounded != self.last_sent:
            send_set_all_angles(self.ser, rounded)
            self.last_sent = rounded

    def _target_step(self, dt):
        """Track the GO-TO-TARGET ramp locally for the live readout.
        The actual motion is done by the Arduino (it received 'M i a v')."""
        reached = True
        for i in range(SERVO_COUNT):
            diff = self.target[i] - self.angles[i]
            step = self._read_speed(i) * dt
            if abs(diff) <= step:
                self.angles[i] = self.target[i]
            else:
                self.angles[i] += step if diff > 0 else -step
                reached = False
        if reached:
            self.target = None

    def _drain_serial(self):
        """Read whatever the Arduino has sent and append complete lines to log."""
        if not self.connected or self.ser is None:
            return
        try:
            waiting = self.ser.in_waiting
        except (OSError, serial.SerialException):
            self._log("!! serial disconnected")
            self.disconnect()
            return
        if not waiting:
            return
        try:
            data = self.ser.read(waiting).decode(errors="replace")
        except (OSError, serial.SerialException):
            self._log("!! serial read error")
            self.disconnect()
            return
        self._rx += data
        while "\n" in self._rx:
            line, self._rx = self._rx.split("\n", 1)
            line = line.strip()
            if line:
                self._log(f"<- {line}")
                self._maybe_sync(line)

    def _maybe_sync(self, line):
        """If the Arduino reported angles ('A a0..a4'), sync our local estimate."""
        if not line.startswith("A "):
            return
        nums = _parse_numbers(line[2:])
        if len(nums) >= SERVO_COUNT:
            self.angles = [clamp(to_logical(i, n), ANGLE_MIN, ANGLE_MAX)
                           for i, n in enumerate(nums[:SERVO_COUNT])]
            self.last_sent = [None] * SERVO_COUNT

    # --------------------------------------------------------- actions ------
    def _start_jog(self, _event=None):
        if not self._require_connection():
            return
        self._abort_macro()
        self.target = None
        self.jogging = True
        self.move_btn.configure(relief="sunken", text="▶  MOVING…  ▶")

    def _stop_jog(self, _event=None):
        self.jogging = False
        self.move_btn.configure(relief="raised", text="▶  HOLD TO MOVE  ▶")
        # The last streamed 'S' frame already holds the motors in place.

    def go_to_targets(self):
        if not self._require_connection():
            return
        targets = list(self.angles)            # disabled motors keep their position
        moved = []
        for i in range(SERVO_COUNT):
            if self.enable_var[i].get():
                tgt = self._read_target(i)
                spd = self._read_speed(i)
                targets[i] = tgt
                send_move_servo(self.ser, i, tgt, spd)
                moved.append(i)
        if not moved:
            self._log("!! GO TO TARGET: no motors selected")
            return
        self.jogging = False
        self.target = targets
        self.last_sent = [None] * SERVO_COUNT
        self._log("-> GO " + ", ".join(
            f"m{i}->{int(round(targets[i]))}@{int(round(self._read_speed(i)))}" for i in moved))

    def nudge(self, i, sign):
        """Move a single motor by NUDGE_STEP degrees at its speed (firmware-smoothed)."""
        if not self._require_connection():
            return
        self.jogging = False
        self.target = None
        target = clamp(self.angles[i] + sign * NUDGE_STEP, ANGLE_MIN, ANGLE_MAX)
        spd = self._read_speed(i)
        send_move_servo(self.ser, i, target, spd)
        self.angles[i] = target
        self.last_sent = [None] * SERVO_COUNT
        self._log(f"-> M {i} {int(round(target))} {int(round(spd))}")

    def stop_hold(self):
        """Freeze every motor at its current estimated position."""
        if not self._require_connection():
            return
        self._abort_macro()
        self.jogging = False
        self.target = None
        send_set_all_angles(self.ser, self.angles)
        self.last_sent = [int(round(a)) for a in self.angles]
        self._log("-> STOP (hold current pose)")

    def home_all(self):
        if not self._require_connection():
            return
        self._abort_macro()
        self.jogging = False
        self.target = None
        self.angles = [START_ANGLE] * SERVO_COUNT
        # Use 'S' (not the firmware 'H') so each motor goes to its offset-adjusted
        # center: physical = to_physical(i, 90) = 90 + OFFSET[i] (e.g. 80 if -10).
        send_set_all_angles(self.ser, self.angles)
        self.last_sent = [int(round(START_ANGLE))] * SERVO_COUNT
        phys = [int(round(to_physical(i, START_ANGLE))) for i in range(SERVO_COUNT)]
        self._log(f"-> home all to 90° logical (physical {phys})")

    def release_all(self):
        if not self._require_connection():
            return
        self._abort_macro()
        self.jogging = False
        self.target = None
        send_release_all(self.ser)
        self.last_sent = [None] * SERVO_COUNT
        for i in range(SERVO_COUNT):
            self.cur_var[i].set("off")
        self._log("-> O (release / torque off)")

    def query_angles(self):
        if not self._require_connection():
            return
        send_query_angles(self.ser)
        self._log("-> Q (query angles)")

    def save_pose(self):
        # Use the first selected motor's speed as the recorded speed (display only).
        name = self.pose_name_var.get().strip()
        speed = self._read_speed(self._first_enabled())
        existed = bool(name) and name in self._pose_paths
        path = save_angles(self.angles, speed, name=name or None)
        saved_name = _pose_display_name(path)
        self.pose_name_var.set("")
        self.refresh_poses()
        self.selected_pose_var.set(saved_name)       # pre-select what we just saved
        note = " (overwritten)" if existed else ""
        self._log(f"-- saved pose '{saved_name}'{note} -> {os.path.basename(path)}")

    def load_selected_pose(self):
        if not self._require_connection():
            return
        name = self.selected_pose_var.get()
        path = self._pose_paths.get(name)
        if not path:
            self._log("!! no saved pose selected")
            return
        loaded = load_pose_file(path)
        if loaded is None:
            self._log(f"!! pose '{name}': could not read angles from {os.path.basename(path)}")
            return
        self.jogging = False
        self.target = list(loaded)
        spd = self._read_speed(self._first_enabled())
        for i in range(SERVO_COUNT):
            send_move_servo(self.ser, i, loaded[i], spd)
            self.target_var[i].set(int(round(loaded[i])))
        self.last_sent = [None] * SERVO_COUNT
        self._log(f"-> pose '{name}': move to {[int(round(v)) for v in loaded]} @ {int(round(spd))} deg/s")

    def delete_selected_pose(self):
        name = self.selected_pose_var.get()
        path = self._pose_paths.get(name)
        if not path:
            self._log("!! no saved pose selected")
            return
        if not messagebox.askyesno("Delete pose",
                                   f"Delete saved pose '{name}'?\n\n{os.path.basename(path)}"):
            return
        try:
            os.remove(path)
            self._log(f"-- deleted pose '{name}'")
        except OSError as exc:
            self._log(f"!! could not delete '{name}': {exc}")
        self.selected_pose_var.set("")
        self.refresh_poses()

    # =====================================================================
    # MACROS  --  editor window + non-blocking runner
    # =====================================================================

    # Which form fields are relevant for each step type (controls show/hide).
    _STEP_FIELDS = {
        "move":     ["index", "angle", "speed"],
        "relative": ["index", "angle", "speed"],
        "set_all":  ["setall"],
        "delay":    ["seconds"],
        "home":     [],
        "release":  [],
        "run":      ["runmacro"],
    }

    def open_macro_window(self):
        """Open (or focus) the macro editor/runner window."""
        if self._macro_win is not None:
            try:
                self._macro_win.lift()
                self._macro_win.focus_force()
                return
            except tk.TclError:
                self._macro_win = None
        self._build_macro_window()

    def _build_macro_window(self):
        win = tk.Toplevel(self.root)
        self._macro_win = win
        win.title("Macros")
        win.minsize(760, 480)
        win.protocol("WM_DELETE_WINDOW", self._close_macro_window)

        ttk.Label(
            win, justify="left", foreground="#444",
            text=("Build a sequence of steps (move / relative / set_all / delay / home / "
                  "release / run) -- same format as terminal_control.py.\nSelect a macro, "
                  "arrange its steps and parameters, then Run. Changes are saved "
                  "automatically to macros.json."),
        ).pack(fill="x", padx=6, pady=(6, 0))

        # --- bottom: saved-pose reference (pinned below the editor) ---------
        poseref = ttk.LabelFrame(win, text="Saved poses (reference — logical angles)")
        poseref.pack(side="bottom", fill="x", padx=6, pady=(0, 6))
        pr_inner = ttk.Frame(poseref)
        pr_inner.pack(fill="x", padx=4, pady=4)
        self._pose_ref_listbox = tk.Listbox(pr_inner, height=5, exportselection=False,
                                            font=("Consolas", 9))
        self._pose_ref_listbox.pack(side="left", fill="x", expand=True)
        pr_sb = ttk.Scrollbar(pr_inner, orient="vertical",
                              command=self._pose_ref_listbox.yview)
        pr_sb.pack(side="left", fill="y")
        self._pose_ref_listbox.configure(yscrollcommand=pr_sb.set)
        pr_btns = ttk.Frame(poseref)
        pr_btns.pack(fill="x", padx=4, pady=(0, 4))
        ttk.Button(pr_btns, text="Use in set_all field",
                   command=self._pose_ref_to_setall).pack(side="left", padx=2)
        ttk.Button(pr_btns, text="Refresh", command=self._refresh_pose_ref).pack(side="left", padx=2)

        body = ttk.Frame(win)
        body.pack(fill="both", expand=True, padx=6, pady=6)

        # --- left: macro list ----------------------------------------------
        left = ttk.LabelFrame(body, text="Macros")
        left.pack(side="left", fill="y")
        self._macro_listbox = tk.Listbox(left, width=20, height=14, exportselection=False)
        self._macro_listbox.pack(fill="y", expand=True, padx=4, pady=4)
        self._macro_listbox.bind("<<ListboxSelect>>", lambda _e: self._on_macro_select())
        mbtns = ttk.Frame(left)
        mbtns.pack(fill="x", padx=4)
        ttk.Button(mbtns, text="New", width=6, command=self._macro_new).grid(row=0, column=0, padx=1, pady=1)
        ttk.Button(mbtns, text="Rename", width=6, command=self._macro_rename).grid(row=0, column=1, padx=1, pady=1)
        ttk.Button(mbtns, text="Dup", width=6, command=self._macro_duplicate).grid(row=1, column=0, padx=1, pady=1)
        ttk.Button(mbtns, text="Delete", width=6, command=self._macro_delete).grid(row=1, column=1, padx=1, pady=1)
        runrow = ttk.Frame(left)
        runrow.pack(fill="x", padx=4, pady=(6, 4))
        tk.Button(runrow, text="▶ Run", bg="#2e8b57", fg="white", width=8,
                  command=self._macro_run_selected).pack(side="left", padx=2)
        tk.Button(runrow, text="■ Stop", bg="#b03030", fg="white", width=8,
                  command=self._abort_macro).pack(side="left", padx=2)

        # --- right: step editor --------------------------------------------
        right = ttk.LabelFrame(body, text="Steps")
        right.pack(side="left", fill="both", expand=True, padx=(6, 0))

        self._step_listbox = tk.Listbox(right, height=12, exportselection=False,
                                        font=("Consolas", 9))
        self._step_listbox.pack(fill="both", expand=True, padx=4, pady=4)
        self._step_listbox.bind("<<ListboxSelect>>", lambda _e: self._on_step_select())

        order = ttk.Frame(right)
        order.pack(fill="x", padx=4)
        ttk.Button(order, text="↑ Up", command=lambda: self._step_move(-1)).pack(side="left", padx=2)
        ttk.Button(order, text="↓ Down", command=lambda: self._step_move(1)).pack(side="left", padx=2)
        ttk.Button(order, text="Delete step", command=self._step_delete).pack(side="left", padx=2)

        # --- step parameter form -------------------------------------------
        form = ttk.LabelFrame(right, text="Add / edit step")
        form.pack(fill="x", padx=4, pady=6)

        typerow = ttk.Frame(form)
        typerow.pack(fill="x", pady=2)
        ttk.Label(typerow, text="Type:").pack(side="left")
        type_combo = ttk.Combobox(typerow, textvariable=self.mv_type, values=STEP_TYPES,
                                  state="readonly", width=12)
        type_combo.pack(side="left", padx=4)
        type_combo.bind("<<ComboboxSelected>>", lambda _e: self._on_type_change())

        # Build each (re-usable) field frame once; show/hide per type.
        self._field_frames = {}

        fi = ttk.Frame(form)
        ttk.Label(fi, text="motor (0-4):").pack(side="left")
        sp = ttk.Spinbox(fi, from_=0, to=SERVO_COUNT - 1, textvariable=self.mv_index, width=5,
                         command=self._update_index_name)
        sp.pack(side="left", padx=4)
        ttk.Label(fi, textvariable=self.mv_index_name, foreground="#666").pack(side="left")
        self._field_frames["index"] = fi

        fa = ttk.Frame(form)
        ttk.Label(fa, text="angle (deg; relative = delta):").pack(side="left")
        ttk.Spinbox(fa, from_=-180, to=180, textvariable=self.mv_angle, width=7).pack(side="left", padx=4)
        self._field_frames["angle"] = fa

        fs = ttk.Frame(form)
        ttk.Label(fs, text="speed (deg/s; 0 = immediate):").pack(side="left")
        ttk.Spinbox(fs, from_=0, to=SPEED_MAX, textvariable=self.mv_speed, width=7).pack(side="left", padx=4)
        self._field_frames["speed"] = fs

        fsec = ttk.Frame(form)
        ttk.Label(fsec, text="seconds:").pack(side="left")
        ttk.Spinbox(fsec, from_=0, to=120, increment=0.1, textvariable=self.mv_seconds,
                    width=7).pack(side="left", padx=4)
        self._field_frames["seconds"] = fsec

        fall = ttk.Frame(form)
        ttk.Label(fall, text="angles (5, space-separated):").pack(side="left")
        ttk.Entry(fall, textvariable=self.mv_setall, width=22).pack(side="left", padx=4)
        self._field_frames["setall"] = fall

        frun = ttk.Frame(form)
        ttk.Label(frun, text="run macro:").pack(side="left")
        self._runmacro_combo = ttk.Combobox(frun, textvariable=self.mv_runmacro,
                                            state="readonly", width=18)
        self._runmacro_combo.pack(side="left", padx=4)
        self._field_frames["runmacro"] = frun

        addrow = ttk.Frame(form)
        addrow.pack(fill="x", pady=(4, 2))
        ttk.Button(addrow, text="Add step", command=lambda: self._step_add(insert=False)).pack(side="left", padx=2)
        ttk.Button(addrow, text="Insert above", command=lambda: self._step_add(insert=True)).pack(side="left", padx=2)
        ttk.Button(addrow, text="Update selected", command=self._step_update).pack(side="left", padx=2)

        self._refresh_macro_list()
        self._on_type_change()
        self._refresh_pose_ref()

    def _close_macro_window(self):
        if self._macro_win is not None:
            try:
                self._macro_win.destroy()
            except tk.TclError:
                pass
        self._macro_win = None

    # ----------------------------------------------------- editor: fields ---
    def _on_type_change(self, *_):
        """Show only the form fields relevant to the chosen step type."""
        show = set(self._STEP_FIELDS.get(self.mv_type.get(), []))
        # Pack in a fixed order so the layout is stable.
        for key in ["index", "angle", "speed", "seconds", "setall", "runmacro"]:
            frame = self._field_frames.get(key)
            if frame is None:
                continue
            if key in show:
                frame.pack(fill="x", pady=1)
            else:
                frame.pack_forget()

    def _update_index_name(self):
        try:
            self.mv_index_name.set(motor_name(int(self.mv_index.get())))
        except (tk.TclError, ValueError):
            self.mv_index_name.set("")

    def _build_step_from_form(self):
        """Assemble a step dict from the form, or None (with a popup) if invalid."""
        t = self.mv_type.get()
        if t in ("move", "relative"):
            return {"type": t,
                    "index": self._ivar(self.mv_index, 0),
                    "angle": self._dvar(self.mv_angle, 0.0),
                    "speed": self._dvar(self.mv_speed, 0.0)}
        if t == "set_all":
            nums = _parse_numbers(self.mv_setall.get())
            if len(nums) < SERVO_COUNT:
                messagebox.showerror("Step error", f"set_all needs {SERVO_COUNT} angles.",
                                     parent=self._macro_win)
                return None
            return {"type": "set_all",
                    "angles": [clamp(n, ANGLE_MIN, ANGLE_MAX) for n in nums[:SERVO_COUNT]]}
        if t == "delay":
            return {"type": "delay", "seconds": max(0.0, self._dvar(self.mv_seconds, 0.0))}
        if t == "run":
            name = self.mv_runmacro.get().strip()
            if not name:
                messagebox.showerror("Step error", "Choose a macro to run.", parent=self._macro_win)
                return None
            return {"type": "run", "macro": name}
        if t in ("home", "release"):
            return {"type": t}
        return None

    # ------------------------------------------------- editor: step list ----
    def _current_steps(self):
        """The step list of the selected macro, or None (with a popup)."""
        if self._cur_macro is None or self._cur_macro not in self.macros:
            messagebox.showinfo("No macro", "Select or create a macro first.",
                                parent=self._macro_win)
            return None
        return self.macros[self._cur_macro]

    def _step_add(self, insert=False):
        steps = self._current_steps()
        if steps is None:
            return
        step = self._build_step_from_form()
        if step is None:
            return
        sel = self._sel_index(self._step_listbox)
        if insert and sel is not None:
            steps.insert(sel, step)
            new_index = sel
        else:
            steps.append(step)
            new_index = len(steps) - 1
        self._commit_steps(new_index)

    def _step_update(self):
        steps = self._current_steps()
        if steps is None:
            return
        sel = self._sel_index(self._step_listbox)
        if sel is None:
            messagebox.showinfo("No step", "Select a step to update.", parent=self._macro_win)
            return
        step = self._build_step_from_form()
        if step is None:
            return
        steps[sel] = step
        self._commit_steps(sel)

    def _step_delete(self):
        steps = self._current_steps()
        if steps is None:
            return
        sel = self._sel_index(self._step_listbox)
        if sel is None:
            return
        del steps[sel]
        self._commit_steps(min(sel, len(steps) - 1) if steps else None)

    def _step_move(self, delta):
        steps = self._current_steps()
        if steps is None:
            return
        sel = self._sel_index(self._step_listbox)
        if sel is None:
            return
        j = sel + delta
        if 0 <= j < len(steps):
            steps[sel], steps[j] = steps[j], steps[sel]
            self._commit_steps(j)

    def _on_step_select(self):
        """Load the highlighted step's values into the form for editing."""
        if self._cur_macro is None:
            return
        steps = self.macros.get(self._cur_macro, [])
        i = self._sel_index(self._step_listbox)
        if i is None or i >= len(steps):
            return
        s = steps[i]
        t = s.get("type", "move")
        self.mv_type.set(t)
        self._on_type_change()
        if t in ("move", "relative"):
            self.mv_index.set(int(s.get("index", 0)))
            self._update_index_name()
            self.mv_angle.set(float(s.get("angle", 0) or 0))
            self.mv_speed.set(float(s.get("speed", 0) or 0))
        elif t == "set_all":
            self.mv_setall.set(" ".join(_fmt_num(a) for a in s.get("angles", [])))
        elif t == "delay":
            self.mv_seconds.set(float(s.get("seconds", 0) or 0))
        elif t == "run":
            self.mv_runmacro.set(s.get("macro", s.get("name", "")))

    def _commit_steps(self, select_index):
        """Persist macros and refresh the step list, reselecting select_index."""
        self._save_macros()
        self._refresh_steps()
        if select_index is not None and 0 <= select_index < self._step_listbox.size():
            self._step_listbox.selection_clear(0, "end")
            self._step_listbox.selection_set(select_index)
            self._step_listbox.see(select_index)

    def _refresh_steps(self):
        self._step_listbox.delete(0, "end")
        for i, step in enumerate(self.macros.get(self._cur_macro, [])):
            self._step_listbox.insert("end", f"{i + 1:2d}. {step_repr(step)}")

    # ----------------------------------------------- editor: macro list -----
    def _refresh_macro_list(self):
        names = sorted(self.macros)
        self._macro_listbox.delete(0, "end")
        for name in names:
            self._macro_listbox.insert("end", name)
        if hasattr(self, "_runmacro_combo"):
            self._runmacro_combo.configure(values=names)
        if self._cur_macro in names:
            idx = names.index(self._cur_macro)
            self._macro_listbox.selection_set(idx)
            self._macro_listbox.see(idx)
        elif names:
            self._cur_macro = names[0]
            self._macro_listbox.selection_set(0)
        else:
            self._cur_macro = None
        self._refresh_steps()

    def _on_macro_select(self):
        i = self._sel_index(self._macro_listbox)
        if i is None:
            return
        self._cur_macro = self._macro_listbox.get(i)
        self._refresh_steps()

    def _macro_new(self):
        name = simpledialog.askstring("New macro", "Macro name:", parent=self._macro_win)
        if not name or not name.strip():
            return
        name = name.strip()
        if name in self.macros:
            messagebox.showerror("New macro", f"'{name}' already exists.", parent=self._macro_win)
            return
        self.macros[name] = []
        self._cur_macro = name
        self._save_macros()
        self._refresh_macro_list()

    def _macro_rename(self):
        if self._cur_macro is None:
            return
        old = self._cur_macro
        name = simpledialog.askstring("Rename macro", "New name:", initialvalue=old,
                                      parent=self._macro_win)
        if not name or not name.strip() or name.strip() == old:
            return
        name = name.strip()
        if name in self.macros:
            messagebox.showerror("Rename", f"'{name}' already exists.", parent=self._macro_win)
            return
        self.macros[name] = self.macros.pop(old)
        self._cur_macro = name
        self._save_macros()
        self._refresh_macro_list()

    def _macro_duplicate(self):
        if self._cur_macro is None:
            return
        base = f"{self._cur_macro}_copy"
        name, n = base, 2
        while name in self.macros:
            name = f"{base}{n}"
            n += 1
        self.macros[name] = [dict(s) for s in self.macros[self._cur_macro]]
        self._cur_macro = name
        self._save_macros()
        self._refresh_macro_list()

    def _macro_delete(self):
        if self._cur_macro is None:
            return
        name = self._cur_macro
        if not messagebox.askyesno("Delete macro", f"Delete macro '{name}'?",
                                   parent=self._macro_win):
            return
        self.macros.pop(name, None)
        self._cur_macro = None
        self._save_macros()
        self._refresh_macro_list()

    def _save_macros(self):
        try:
            save_macros(self.macros)
        except OSError as exc:
            self._log(f"!! could not save macros: {exc}")

    # ------------------------------------------------- saved-pose viewer ----
    def _refresh_pose_ref(self):
        """Fill the macro window's saved-pose reference list with name + angles."""
        self._pose_ref = []
        self._pose_ref_listbox.delete(0, "end")
        name_w = max((len(n) for n, _ in saved_poses()), default=0)
        for name, path in saved_poses():
            angles = load_pose_file(path)
            self._pose_ref.append((name, angles))
            if angles:
                shown = " ".join(f"{int(round(a)):3d}" for a in angles)
                self._pose_ref_listbox.insert("end", f"{name:<{name_w}}  [{shown}]")
            else:
                self._pose_ref_listbox.insert("end", f"{name:<{name_w}}  (unreadable)")
        if not self._pose_ref:
            self._pose_ref_listbox.insert("end", "(no saved poses in saved_angles/)")

    def _pose_ref_to_setall(self):
        """Load the selected saved pose's angles into the set_all step field."""
        i = self._sel_index(self._pose_ref_listbox)
        if i is None or i >= len(self._pose_ref):
            return
        name, angles = self._pose_ref[i]
        if not angles:
            return
        self.mv_type.set("set_all")
        self._on_type_change()
        self.mv_setall.set(" ".join(str(int(round(a))) for a in angles))

    # ----------------------------------------------------- macro runner -----
    def _macro_run_selected(self):
        if self._cur_macro is None:
            messagebox.showinfo("Run macro", "Select a macro first.", parent=self._macro_win)
            return
        self.run_macro(self._cur_macro)

    def run_macro(self, name):
        """Flatten (expanding nested 'run' steps) and start the non-blocking runner."""
        if not self._require_connection():
            return
        if self._macro_running:
            self._log("!! a macro is already running (Stop it first)")
            return
        try:
            flat = self._flatten_macro(name, set())
        except ValueError as exc:
            self._log(f"!! macro '{name}': {exc}")
            return
        if not flat:
            self._log(f"!! macro '{name}' has no steps")
            return
        # The macro takes control away from manual jog/target.
        self.jogging = False
        self.target = None
        self._macro_steps = flat
        self._macro_index = 0
        self._macro_name = name
        self._macro_wait_until = 0.0
        self._macro_running = True
        self._log(f"[*] running macro '{name}' ({len(flat)} steps)")
        self._macro_step()

    def _flatten_macro(self, name, seen):
        """Return a flat step list, expanding 'run' steps; guards against cycles."""
        if name in seen:
            raise ValueError(f"recursive 'run' loop via '{name}'")
        if name not in self.macros:
            raise ValueError(f"macro '{name}' not found")
        flat = []
        for step in self.macros[name]:
            if step.get("type") == "run":
                sub = step.get("macro") or step.get("name")
                if sub:
                    flat.extend(self._flatten_macro(sub, seen | {name}))
            else:
                flat.append(step)
        return flat

    def _macro_step(self):
        if not self._macro_running:
            return
        now = time.perf_counter()
        if now < self._macro_wait_until:               # still inside a delay
            self.root.after(20, self._macro_step)
            return
        if self._macro_index >= len(self._macro_steps):
            self._macro_running = False
            self._log(f"[*] macro '{self._macro_name}' done")
            return
        step = self._macro_steps[self._macro_index]
        self._macro_index += 1
        try:
            self._exec_macro_step(step, now)
        except Exception as exc:                        # serial dropped, bad data, ...
            self._macro_running = False
            self._log(f"!! macro aborted at step {self._macro_index}: {exc}")
            return
        self.root.after(15, self._macro_step)

    def _exec_macro_step(self, step, now):
        t = step.get("type")
        n = len(self._macro_steps)
        if t == "move":
            i = int(step["index"])
            a = clamp(float(step["angle"]), ANGLE_MIN, ANGLE_MAX)
            send_move_servo(self.ser, i, a, step.get("speed"))
            self.angles[i] = a
            self.last_sent = [None] * SERVO_COUNT
        elif t == "relative":
            i = int(step["index"])
            a = clamp(self.angles[i] + float(step.get("angle", 0.0)), ANGLE_MIN, ANGLE_MAX)
            send_move_servo(self.ser, i, a, step.get("speed"))
            self.angles[i] = a
            self.last_sent = [None] * SERVO_COUNT
        elif t == "set_all":
            angs = [clamp(float(x), ANGLE_MIN, ANGLE_MAX) for x in step.get("angles", [])]
            while len(angs) < SERVO_COUNT:
                angs.append(self.angles[len(angs)])
            angs = angs[:SERVO_COUNT]
            send_set_all_angles(self.ser, angs)
            self.angles[:] = angs
            self.last_sent = [None] * SERVO_COUNT
        elif t == "home":
            send_home_all(self.ser)
            self.angles = [START_ANGLE] * SERVO_COUNT
            self.last_sent = [int(round(START_ANGLE))] * SERVO_COUNT
        elif t == "release":
            send_release_all(self.ser)
            self.last_sent = [None] * SERVO_COUNT
        elif t == "delay":
            self._macro_wait_until = now + max(0.0, float(step.get("seconds", 0.0)))
        else:
            self._log(f"   [{self._macro_index}/{n}] skipped unknown step '{t}'")
            return
        self._log(f"   [{self._macro_index}/{n}] {step_repr(step)}")

    def _abort_macro(self):
        if self._macro_running:
            self._macro_running = False
            self._log("[*] macro stopped")

    # ----------------------------------------------------- small helpers ----
    def _sel_index(self, listbox):
        sel = listbox.curselection()
        return sel[0] if sel else None

    def _ivar(self, var, default=0):
        try:
            return int(var.get())
        except (tk.TclError, ValueError):
            return default

    def _dvar(self, var, default=0.0):
        try:
            return float(var.get())
        except (tk.TclError, ValueError):
            return default

    # --------------------------------------------------------- helpers ------
    def set_all_enabled(self, value):
        for v in self.enable_var:
            v.set(value)

    def _first_enabled(self):
        for i in range(SERVO_COUNT):
            if self.enable_var[i].get():
                return i
        return 0

    def _read_speed(self, i):
        try:
            return clamp(float(self.speed_var[i].get()), SPEED_MIN, SPEED_MAX)
        except (tk.TclError, ValueError):
            return SPEED_DEG_PER_S

    def _read_target(self, i):
        try:
            return clamp(float(self.target_var[i].get()), ANGLE_MIN, ANGLE_MAX)
        except (tk.TclError, ValueError):
            return self.angles[i]

    def _refresh_angles(self):
        for i in range(SERVO_COUNT):
            self.cur_var[i].set(f"{self.angles[i]:.0f}°")

    def refresh_poses(self):
        """Repopulate the recall dropdown from the saved_angles/ folder."""
        self._pose_paths = {name: path for name, path in saved_poses()}
        names = list(self._pose_paths.keys())
        self.pose_combo.configure(values=names)
        if self.selected_pose_var.get() not in self._pose_paths:
            self.selected_pose_var.set(names[0] if names else "")
        # Keep the macro window's reference list in sync if it is open.
        if self._macro_win is not None:
            try:
                self._refresh_pose_ref()
            except tk.TclError:
                pass

    # ----------------------------------------------------------- cleanup ----
    def on_close(self):
        try:
            self.disconnect()
        finally:
            self.root.destroy()


# =============================================================================
# MAIN
# =============================================================================

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    root = tk.Tk()
    ServoGuiApp(root, port)
    root.mainloop()


if __name__ == "__main__":
    main()
