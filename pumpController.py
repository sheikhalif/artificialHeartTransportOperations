#!/usr/bin/env python3
"""
Direct USB Pump Controller — GUI, live plotting, and data logging.

Key design choices
------------------
* Arduino streams DATA,p1,p2,mode,p1_duty,p2_duty,bpm_set[,bpm_meas] at 50 Hz.
* Display smoothing is mode-aware:
    HEART -> 3-point rolling median (sharp EKG-like peaks preserved)
    other -> 2 s rolling mean (stable steady-state readout)
* X-axis window is mode-aware:
    HEART -> 5 cardiac cycles (scales with BPM)
    other -> fixed 25 s rolling window
* Y-axis is fixed to 0..3.5 PSI so the trace stays visually anchored.
* Pump strength: 0-100% slider plus a typeable Entry box per pump
  that accepts decimal percentages. Slider value maps directly to
  PWM duty 0..255.
* Calibration sequence (run on demand from the side panel):
    1. Prime: both pumps at 100% for 4 s,
    2. Settle: pumps off for 2 s,
    3. Baseline: capture each sensor's zero offset for 3 s,
    4. Verify: both pumps at 100% for 2 s for visual confirmation.
  The captured offset is subtracted from all subsequent readings.
* BPM sweep study (right-side panel): runs an automated experimental
  campaign over a configurable BPM range. For each trial it records
  for the configured duration, detects pressure peaks, saves a per-
  trial CSV+PNG, then produces a summary CSV+PNG with BPM vs. mean
  peak pressure for both pumps.
* Origin-style plotting throughout.

Requirements:
    pip install pyserial matplotlib numpy
    pip install scipy   # optional — used for peak detection if available
"""

import csv
import os
import queue
import sys
import threading
import time
import tkinter as tk
from collections import deque
from datetime import datetime
from tkinter import messagebox, scrolledtext, ttk

import numpy as np
import serial
import serial.tools.list_ports
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib.ticker import AutoMinorLocator

try:
    from scipy.signal import find_peaks as _scipy_find_peaks
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

# === Configuration ===
BAUD_RATE = 115200
SERIAL_TIMEOUT = 0.1
DATA_POLL_INTERVAL_MS = 30
PLOT_UPDATE_INTERVAL_MS = 100      # ~10 Hz screen refresh
PLOT_WINDOW_SECONDS = 25           # default rolling window for non-HEART modes
HEART_CYCLES_VISIBLE = 5           # number of heartbeat cycles to show in HEART mode
ARDUINO_REPORT_HZ = 50             # must match Arduino REPORT_INTERVAL_MS
MAX_PLOT_POINTS = PLOT_WINDOW_SECONDS * ARDUINO_REPORT_HZ
ROLLING_MEAN_SECONDS = 2.0         # window for CONT-mode display smoothing
HEART_MEDIAN_WINDOW = 3            # tiny median to kill single-sample spikes
Y_AXIS_MAX_PSI = 3.5               # fixed y-axis ceiling for the live plot
Y_AXIS_MIN_PSI = 0.0               # fixed y-axis floor for the live plot

# Calibration sequence timing (seconds)
CAL_PRIME_DURATION_S    = 4.0   # run pumps at full power to prime the system
CAL_SETTLE_DURATION_S   = 2.0   # let pressure decay to baseline after stopping
CAL_BASELINE_DURATION_S = 3.0   # average sensor readings to capture zero offset
CAL_PRIME_DUTY          = 255   # pump duty during prime phase (0-255)
CAL_VERIFY_DURATION_S   = 2.0   # final verification: run pumps at full power this long

# Heart-rate matching parameters
HR_MATCH_DEBOUNCE_MS    = 1500     # min interval between BPM= updates when matching
HR_MATCH_DELTA_BPM      = 2        # only re-send if measured HR differs by > this
HR_SMOOTH_WINDOW        = 5        # rolling-mean window for HR display

# BPM sweep study defaults
SWEEP_BPM_START         = 50
SWEEP_BPM_END           = 150
SWEEP_BPM_STEP          = 10
SWEEP_TRIAL_DURATION_S  = 60.0
SWEEP_STABILIZE_S       = 5.0
SWEEP_PEAK_PROMINENCE_FRAC = 0.30  # peak must rise this fraction of (max-min)
SWEEP_PEAK_HEIGHT_FRAC  = 0.50     # peak must exceed mean + this*(max-mean)
SWEEP_PEAK_DISTANCE_FRAC = 0.60    # min spacing as fraction of expected period

# Pump-power sweep study defaults
PSWEEP_POWER_START      = 0
PSWEEP_POWER_END        = 100
PSWEEP_POWER_STEP       = 10
PSWEEP_TRIAL_DURATION_S = 30.0
PSWEEP_STABILIZE_S      = 5.0


# === Origin-style matplotlib defaults =====================================
# Applied once at module load; every figure created afterward inherits these.
ORIGIN_RC = {
    "font.family": "serif",
    "font.serif": ["Times New Roman", "DejaVu Serif", "Liberation Serif"],
    "font.size": 11,
    "axes.linewidth": 1.4,
    "axes.edgecolor": "black",
    "axes.labelsize": 12,
    "axes.labelweight": "bold",
    "axes.titlesize": 12,
    "axes.titleweight": "bold",
    "axes.spines.top": True,
    "axes.spines.right": True,
    "xtick.direction": "in",
    "ytick.direction": "in",
    "xtick.top": True,
    "ytick.right": True,
    "xtick.major.size": 6,
    "ytick.major.size": 6,
    "xtick.minor.size": 3,
    "ytick.minor.size": 3,
    "xtick.major.width": 1.2,
    "ytick.major.width": 1.2,
    "xtick.minor.width": 1.0,
    "ytick.minor.width": 1.0,
    "xtick.minor.visible": True,
    "ytick.minor.visible": True,
    "legend.frameon": True,
    "legend.framealpha": 1.0,
    "legend.edgecolor": "black",
    "legend.fancybox": False,
    "legend.fontsize": 10,
    "figure.facecolor": "white",
    "axes.facecolor": "white",
    "savefig.facecolor": "white",
}

# Color palette — high-contrast, publication-grade
COLOR_P1 = "#1f3a93"  # navy blue
COLOR_P2 = "#b22222"  # firebrick red

import matplotlib as mpl
mpl.rcParams.update(ORIGIN_RC)


# === Serial Manager =======================================================
class SerialManager:
    def __init__(self, message_queue):
        self.serial = None
        self.connected = False
        self.message_queue = message_queue
        self.read_thread = None
        self.stop_flag = threading.Event()

    def list_ports(self):
        ports = serial.tools.list_ports.comports()
        result = [(p.device, p.description) for p in ports]
        if sys.platform == "darwin":
            # On macOS each USB-CDC device shows up twice: /dev/cu.usbmodemXXXX
            # and /dev/tty.usbmodemXXXX. The tty.* version blocks on a DCD
            # signal that USB-CDC doesn't assert, and that's where the
            # "termios.error: (22, 'Invalid argument')" crash comes from.
            # Hide tty.* duplicates whose cu.* counterpart is present.
            cu_set = {p for p, _ in result if p.startswith("/dev/cu.")}
            filtered = []
            for p, d in result:
                if p.startswith("/dev/tty."):
                    cu_alt = "/dev/cu." + p[len("/dev/tty."):]
                    if cu_alt in cu_set:
                        continue
                filtered.append((p, d))
            result = filtered
        return result

    def connect(self, port):
        # Retry once on failure — macOS sometimes needs a beat after an
        # earlier session before the port is fully released. Catch a broad
        # Exception because termios.error is *not* a subclass of OSError
        # in Python 3.13, so a narrower handler lets it crash the GUI.
        last_error = None
        for attempt in range(2):
            try:
                self.serial = serial.Serial(
                    port, BAUD_RATE, timeout=SERIAL_TIMEOUT)
                time.sleep(2.0)
                self.connected = True
                self.stop_flag.clear()
                self.read_thread = threading.Thread(
                    target=self._read_loop, daemon=True)
                self.read_thread.start()
                return True, None
            except Exception as e:                          # noqa: BLE001
                last_error = f"{type(e).__name__}: {e}"
                if self.serial is not None:
                    try:
                        self.serial.close()
                    except Exception:
                        pass
                    self.serial = None
                time.sleep(0.4)
        return False, last_error

    def disconnect(self):
        if self.connected and self.serial:
            try:
                self.serial.write(b"OFF\n")
                self.serial.flush()
                time.sleep(0.1)
            except Exception:
                pass
        self.stop_flag.set()
        # Wait for the read thread to actually exit before closing the
        # port, otherwise the next connect() can race with a pending read.
        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join(timeout=1.0)
        self.read_thread = None
        if self.serial:
            try:
                self.serial.close()
            except Exception:
                pass
        self.serial = None
        self.connected = False
        # Brief pause so the OS releases the port before any reconnect
        time.sleep(0.2)

    def send_command(self, command):
        if not self.connected or not self.serial:
            return False
        try:
            self.serial.write((command + "\n").encode())
            self.serial.flush()
            self.message_queue.put(("SENT", command))
            return True
        except Exception as e:                              # noqa: BLE001
            self.message_queue.put(
                ("ERROR", f"Send failed: {type(e).__name__}: {e}"))
            self.connected = False
            return False

    def _read_loop(self):
        buffer = ""
        while not self.stop_flag.is_set():
            try:
                if self.serial and self.serial.in_waiting:
                    data = self.serial.read(self.serial.in_waiting).decode(
                        errors="replace")
                    buffer += data
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        line = line.strip()
                        if line:
                            self._parse_line(line)
                else:
                    time.sleep(0.005)
            except Exception as e:                          # noqa: BLE001
                self.message_queue.put(
                    ("ERROR", f"Read failed: {type(e).__name__}: {e}"))
                break
        self.connected = False

    def _parse_line(self, line):
        # Format: DATA,p1,p2,mode,p1_duty,p2_duty,bpm_set,bpm_meas
        if line.startswith("DATA,"):
            parts = line[5:].split(",")
            if len(parts) >= 6:
                try:
                    self.message_queue.put(("DATA", {
                        "p1": float(parts[0]),
                        "p2": float(parts[1]),
                        "mode": parts[2],
                        "pump1_duty": int(parts[3]),
                        "pump2_duty": int(parts[4]),
                        "bpm": int(parts[5]),
                        "hr_measured": int(parts[6]) if len(parts) > 6 else 0,
                    }))
                    return
                except ValueError:
                    pass
            self.message_queue.put(("RAW", line))
        elif line.startswith("OK,"):
            self.message_queue.put(("OK", line[3:]))
        elif line.startswith("ERR,"):
            self.message_queue.put(("ERR", line[4:]))
        else:
            self.message_queue.put(("RAW", line))


# === Main App =============================================================
class PumpControllerApp:
    BG_COLOR        = "#f4f4f7"
    PANEL_COLOR     = "#ffffff"
    BORDER_COLOR    = "#d4d4d9"
    TEXT_COLOR      = "#1a1a1a"
    MUTED_COLOR     = "#666666"
    SUCCESS_COLOR   = "#1e8449"   # green
    DANGER_COLOR    = "#c0392b"   # red
    ACCENT_COLOR    = "#1f3a93"   # blue (matches P1)

    def __init__(self, root):
        self.root = root
        self.root.title("Pump Controller — EvapoFlex Cardiac Sim")
        self.root.geometry("1280x880")
        self.root.minsize(1100, 760)
        self.root.configure(bg=self.BG_COLOR)

        self.message_queue = queue.Queue()
        self.serial = SerialManager(self.message_queue)

        # UI state
        self.master_on   = tk.BooleanVar(value=False)
        self.mode_var    = tk.StringVar(value="CONT")
        self.pump1_var   = tk.IntVar(value=255)
        self.pump2_var   = tk.IntVar(value=255)
        self.bpm_var     = tk.IntVar(value=60)
        self._last_bpm_sent = 60     # for change detection

        # Plotting & recording state
        self.is_recording = tk.BooleanVar(value=False)
        self.csv_file = None
        self.csv_writer = None
        self.csv_path = None
        self.recording_started_at = None
        self.recording_params_snapshot = None
        self.start_time = time.time()

        # Calibration state — zero-offset correction subtracted from readings
        self.cal_offset_p1 = 0.0
        self.cal_offset_p2 = 0.0
        self.cal_done = False
        self._cal_in_progress = False
        self._cal_collecting = False
        self._cal_samples_p1 = []
        self._cal_samples_p2 = []

        # Heart-rate matching state
        self.match_hr = tk.BooleanVar(value=False)
        self._hr_recent = deque(maxlen=HR_SMOOTH_WINDOW)
        self._last_hr_send_ms = 0
        self._last_hr_sent_bpm = 0

        # BPM sweep study — runs a structured experimental campaign:
        # for each BPM in a range, take a fixed-duration recording,
        # detect peaks, and produce per-trial + summary plots.
        self.sweep_bpm_start = tk.IntVar(value=SWEEP_BPM_START)
        self.sweep_bpm_end   = tk.IntVar(value=SWEEP_BPM_END)
        self.sweep_bpm_step  = tk.IntVar(value=SWEEP_BPM_STEP)
        self.sweep_trial_s   = tk.IntVar(value=int(SWEEP_TRIAL_DURATION_S))
        self.sweep_stab_s    = tk.IntVar(value=int(SWEEP_STABILIZE_S))

        self._sweep_in_progress = False
        self._sweep_state = "IDLE"   # IDLE | STABILIZE | RECORDING
        self._sweep_state_until_ms = 0
        self._sweep_bpm_list = []
        self._sweep_trial_idx = 0
        self._sweep_trial_results = []
        self._sweep_dir = None
        self._sweep_started_at = None
        self._sweep_csv_file = None
        self._sweep_csv_writer = None
        self._sweep_trial_t  = []
        self._sweep_trial_p1 = []
        self._sweep_trial_p2 = []

        # Pump-power sweep study — varies CONT-mode pump power and
        # measures the resulting steady-state pressure for each pump.
        self.psweep_pwr_start = tk.IntVar(value=PSWEEP_POWER_START)
        self.psweep_pwr_end   = tk.IntVar(value=PSWEEP_POWER_END)
        self.psweep_pwr_step  = tk.IntVar(value=PSWEEP_POWER_STEP)
        self.psweep_trial_s   = tk.IntVar(value=int(PSWEEP_TRIAL_DURATION_S))
        self.psweep_stab_s    = tk.IntVar(value=int(PSWEEP_STABILIZE_S))

        self._psweep_in_progress = False
        self._psweep_state = "IDLE"  # IDLE | STABILIZE | RECORDING
        self._psweep_state_until_ms = 0
        self._psweep_pwr_list = []
        self._psweep_trial_idx = 0
        self._psweep_trial_results = []
        self._psweep_dir = None
        self._psweep_started_at = None
        self._psweep_csv_file = None
        self._psweep_csv_writer = None
        self._psweep_trial_t  = []
        self._psweep_trial_p1 = []
        self._psweep_trial_p2 = []
        self._psweep_trial_started_at = None

        # Buffers — sized for the rolling window
        self.plot_times = deque(maxlen=MAX_PLOT_POINTS)
        self.plot_p1_raw = deque(maxlen=MAX_PLOT_POINTS)
        self.plot_p2_raw = deque(maxlen=MAX_PLOT_POINTS)
        self.last_mode_seen = "CONT"

        self._configure_styles()
        self._build_ui()
        self._set_controls_enabled(False)

        self.root.after(DATA_POLL_INTERVAL_MS, self._poll_messages)
        self.root.after(PLOT_UPDATE_INTERVAL_MS, self._update_plot)
        # Re-run the port refresh now that the log widget exists, so the
        # raw-port diagnostic ends up where the user can actually see it.
        self.root.after(50, self._refresh_ports)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # === Styling ==========================================================
    def _configure_styles(self):
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        style.configure(
            "TFrame", background=self.BG_COLOR,
        )
        style.configure(
            "Panel.TLabelframe",
            background=self.PANEL_COLOR,
            relief="solid",
            borderwidth=1,
            bordercolor=self.BORDER_COLOR,
        )
        style.configure(
            "Panel.TLabelframe.Label",
            background=self.PANEL_COLOR,
            foreground=self.TEXT_COLOR,
            font=("Helvetica", 10, "bold"),
        )
        style.configure(
            "TLabel", background=self.PANEL_COLOR, foreground=self.TEXT_COLOR,
        )
        style.configure(
            "Muted.TLabel",
            background=self.PANEL_COLOR, foreground=self.MUTED_COLOR,
        )
        style.configure(
            "Success.TLabel",
            background=self.PANEL_COLOR, foreground=self.SUCCESS_COLOR,
            font=("Helvetica", 10, "bold"),
        )
        style.configure(
            "Danger.TLabel",
            background=self.PANEL_COLOR, foreground=self.DANGER_COLOR,
            font=("Helvetica", 10, "bold"),
        )
        style.configure(
            "Big.TLabel",
            background=self.PANEL_COLOR, foreground=self.TEXT_COLOR,
            font=("Helvetica", 18, "bold"),
        )
        style.configure(
            "Unit.TLabel",
            background=self.PANEL_COLOR, foreground=self.MUTED_COLOR,
            font=("Helvetica", 10),
        )
        style.configure("TButton", padding=6)
        style.configure(
            "Accent.TButton",
            foreground="white", background=self.ACCENT_COLOR,
            font=("Helvetica", 10, "bold"),
        )
        style.map(
            "Accent.TButton",
            background=[("active", "#16306f"), ("disabled", "#9aa3c1")],
        )
        style.configure(
            "Slim.TRadiobutton",
            background=self.PANEL_COLOR, foreground=self.TEXT_COLOR,
        )

    # === Layout ===========================================================
    def _build_ui(self):
        outer = ttk.Frame(self.root, style="TFrame")
        outer.pack(fill="both", expand=True, padx=12, pady=12)

        main_pane = ttk.PanedWindow(outer, orient="horizontal")
        main_pane.pack(fill="both", expand=True)

        left_frame  = ttk.Frame(main_pane, style="TFrame")
        right_frame = ttk.Frame(main_pane, style="TFrame")
        main_pane.add(left_frame,  weight=0)
        main_pane.add(right_frame, weight=1)

        self._build_connection(left_frame)
        self._build_calibration(left_frame)
        self._build_master(left_frame)
        self._build_mode(left_frame)
        self._build_strength(left_frame)
        self._build_bpm(left_frame)
        self._build_data(left_frame)

        self._build_recording(right_frame)
        self._build_sweep(right_frame)
        self._build_power_sweep(right_frame)
        self._build_graph(right_frame)
        self._build_log(right_frame)

    def _panel(self, parent, title):
        frame = ttk.LabelFrame(parent, text=f"  {title}  ",
                               style="Panel.TLabelframe", padding=12)
        frame.pack(fill="x", pady=(0, 10))
        return frame

    def _build_connection(self, parent):
        frame = self._panel(parent, "Connection")
        ttk.Label(frame, text="Port", style="TLabel").pack(anchor="w")
        row = tk.Frame(frame, bg=self.PANEL_COLOR)
        row.pack(fill="x", pady=(2, 6))

        self.port_combo = ttk.Combobox(row, width=22, state="readonly")
        self.port_combo.pack(side="left", fill="x", expand=True)
        ttk.Button(row, text="↻", width=3, command=self._refresh_ports
                   ).pack(side="left", padx=(4, 0))
        ttk.Button(row, text="?", width=3,
                   command=self._show_port_help
                   ).pack(side="left", padx=(2, 0))

        self.port_count_label = ttk.Label(
            frame, text="—", style="Muted.TLabel",
            font=("Menlo", 9))
        self.port_count_label.pack(anchor="w", pady=(0, 4))

        self.connect_button = ttk.Button(
            frame, text="Connect", style="Accent.TButton",
            command=self._toggle_connection)
        self.connect_button.pack(fill="x")

        self.status_label = ttk.Label(
            frame, text="● Disconnected", style="Danger.TLabel")
        self.status_label.pack(anchor="w", pady=(8, 0))

        self._refresh_ports()

    def _build_calibration(self, parent):
        frame = self._panel(parent, "Calibration")

        self.cal_status_label = ttk.Label(
            frame, text="Not calibrated.", style="Muted.TLabel")
        self.cal_status_label.pack(anchor="w", pady=(0, 4))

        self.cal_offsets_label = ttk.Label(
            frame, text="P1 offset: —    P2 offset: —", style="TLabel",
            font=("Menlo", 9))
        self.cal_offsets_label.pack(anchor="w", pady=(0, 6))

        self.cal_button = ttk.Button(
            frame, text="Run Calibration", style="Accent.TButton",
            command=self._start_calibration)
        self.cal_button.pack(fill="x")
        self.cal_button.state(["disabled"])

    def _build_master(self, parent):
        frame = self._panel(parent, "Master")
        self.master_button = tk.Button(
            frame, text="⬤   PUMPS  OFF",
            font=("Helvetica", 16, "bold"),
            bg=self.DANGER_COLOR, fg="white",
            activebackground="#992d22", activeforeground="white",
            command=self._toggle_master,
            cursor="hand2", padx=20, pady=14, relief="flat",
            borderwidth=0, highlightthickness=0,
        )
        self.master_button.pack(fill="x")

    def _build_mode(self, parent):
        frame = self._panel(parent, "Pump Mode")
        modes = [("Continuous", "CONT"),
                 ("Heartbeat",  "HEART"),
                 ("Alternating","ALT")]
        for label, value in modes:
            rb = ttk.Radiobutton(
                frame, text=label, variable=self.mode_var, value=value,
                style="Slim.TRadiobutton", command=self._mode_changed,
            )
            rb.pack(anchor="w", pady=2)

    def _build_strength(self, parent):
        frame = self._panel(parent, "Pump Strength")

        # Per-pump StringVars hold the displayed percentage. Bidirectional
        # sync with pumpN_var (the IntVar used by the slider): slider drag
        # updates the entry text, entry commit (Enter or focus-out) updates
        # the slider position. The entry accepts decimals.
        self.pump1_pct_var = tk.StringVar(value="100.0")
        self.pump2_pct_var = tk.StringVar(value="100.0")

        for row, pump_num in enumerate([1, 2]):
            ttk.Label(frame, text=f"Pump {pump_num}", style="TLabel"
                      ).grid(row=row, column=0, sticky="w",
                             padx=(0, 10), pady=4)

            var = self.pump1_var if pump_num == 1 else self.pump2_var
            scale = ttk.Scale(frame, from_=0, to=255, orient="horizontal",
                              variable=var, length=180)
            scale.grid(row=row, column=1, sticky="we", padx=4, pady=4)
            scale.bind("<ButtonRelease-1>",
                       self._pump1_released if pump_num == 1
                       else self._pump2_released)

            pct_var = (self.pump1_pct_var if pump_num == 1
                       else self.pump2_pct_var)
            entry = ttk.Entry(frame, textvariable=pct_var,
                              width=6, justify="right",
                              font=("Helvetica", 10))
            entry.grid(row=row, column=2, padx=(4, 0), pady=4)
            entry.bind("<Return>",
                       lambda e, n=pump_num: self._on_pump_pct_committed(n))
            entry.bind("<FocusOut>",
                       lambda e, n=pump_num: self._on_pump_pct_committed(n))
            ttk.Label(frame, text="%", style="Muted.TLabel"
                      ).grid(row=row, column=3, sticky="w", padx=(2, 4))

            if pump_num == 1:
                self.pump1_scale = scale
                self.pump1_entry = entry
                self.pump1_var.trace_add(
                    "write", lambda *a: self._sync_pct_from_duty(1))
            else:
                self.pump2_scale = scale
                self.pump2_entry = entry
                self.pump2_var.trace_add(
                    "write", lambda *a: self._sync_pct_from_duty(2))

        frame.columnconfigure(1, weight=1)
        self._sync_pct_from_duty(1)
        self._sync_pct_from_duty(2)

    def _build_bpm(self, parent):
        frame = self._panel(parent, "Heart Rate")
        ttk.Label(frame, text="BPM", style="TLabel").grid(
            row=0, column=0, sticky="w", padx=(0, 10))
        self.bpm_scale = ttk.Scale(
            frame, from_=30, to=180, orient="horizontal",
            variable=self.bpm_var, length=200)
        self.bpm_scale.grid(row=0, column=1, sticky="we", padx=4)
        self.bpm_scale.bind("<ButtonRelease-1>", self._bpm_released)

        self.bpm_label = ttk.Label(frame, text="60 BPM", width=8,
                                   style="TLabel")
        self.bpm_label.grid(row=0, column=2, padx=4)
        self.bpm_var.trace_add("write", lambda *a: self._update_bpm_label())

        # Live sensor display
        ttk.Label(frame, text="Sensor", style="TLabel").grid(
            row=1, column=0, sticky="w", padx=(0, 10), pady=(8, 0))
        self.hr_sensor_label = ttk.Label(
            frame, text="—  no signal", style="Muted.TLabel")
        self.hr_sensor_label.grid(row=1, column=1, columnspan=2,
                                  sticky="w", pady=(8, 0))

        # Match-to-sensor toggle
        self.match_hr_check = ttk.Checkbutton(
            frame, text="Match BPM to sensor",
            variable=self.match_hr,
            command=self._on_match_hr_toggle,
            style="Slim.TRadiobutton",
        )
        self.match_hr_check.grid(row=2, column=0, columnspan=3,
                                 sticky="w", pady=(6, 0))

        frame.columnconfigure(1, weight=1)

    def _build_data(self, parent):
        frame = self._panel(parent, "Live Pressure")
        self.p1_value = ttk.Label(frame, text="—", style="Big.TLabel")
        self.p2_value = ttk.Label(frame, text="—", style="Big.TLabel")

        for i, (lbl_txt, val_widget, color) in enumerate([
            ("Pressure 1", self.p1_value, COLOR_P1),
            ("Pressure 2", self.p2_value, COLOR_P2),
        ]):
            tk.Frame(frame, width=6, bg=color).grid(
                row=i, column=0, sticky="ns", padx=(0, 8), pady=4)
            ttk.Label(frame, text=lbl_txt, style="TLabel"
                      ).grid(row=i, column=1, sticky="w", pady=4)
            val_widget.grid(row=i, column=2, sticky="e", padx=10)
            ttk.Label(frame, text="PSI", style="Unit.TLabel"
                      ).grid(row=i, column=3, sticky="w")
        frame.columnconfigure(2, weight=1)

    def _build_recording(self, parent):
        frame = self._panel(parent, "Data Logging")
        frame.pack(fill="x", padx=(10, 0))

        row = tk.Frame(frame, bg=self.PANEL_COLOR)
        row.pack(fill="x")

        self.record_button = tk.Button(
            row, text="⏺  Start Recording",
            font=("Helvetica", 11, "bold"),
            bg=self.ACCENT_COLOR, fg="white",
            activebackground="#16306f", activeforeground="white",
            command=self._toggle_recording,
            cursor="hand2", padx=14, pady=6, relief="flat",
            borderwidth=0, highlightthickness=0,
        )
        self.record_button.pack(side="left")

        ttk.Button(row, text="Clear graph", command=self._clear_graph
                   ).pack(side="left", padx=(8, 0))

        self.record_status = ttk.Label(
            frame, text="Not recording.", style="Muted.TLabel")
        self.record_status.pack(anchor="w", pady=(8, 0))

    def _build_sweep(self, parent):
        frame = ttk.LabelFrame(parent, text="  BPM Sweep Study  ",
                               style="Panel.TLabelframe", padding=10)
        frame.pack(fill="x", padx=(10, 0), pady=(0, 10))

        config_row = tk.Frame(frame, bg=self.PANEL_COLOR)
        config_row.pack(fill="x")

        def _label(parent_, text):
            return ttk.Label(parent_, text=text, style="TLabel")

        _label(config_row, "BPM").pack(side="left")
        tk.Spinbox(config_row, from_=30, to=180, width=4,
                   textvariable=self.sweep_bpm_start
                   ).pack(side="left", padx=(2, 0))
        _label(config_row, "→").pack(side="left", padx=2)
        tk.Spinbox(config_row, from_=30, to=180, width=4,
                   textvariable=self.sweep_bpm_end
                   ).pack(side="left", padx=(0, 6))
        _label(config_row, "step").pack(side="left")
        tk.Spinbox(config_row, from_=1, to=50, width=3,
                   textvariable=self.sweep_bpm_step
                   ).pack(side="left", padx=(2, 12))

        _label(config_row, "Trial").pack(side="left")
        tk.Spinbox(config_row, from_=10, to=600, width=4,
                   textvariable=self.sweep_trial_s
                   ).pack(side="left", padx=(2, 0))
        _label(config_row, "s").pack(side="left", padx=(1, 8))

        _label(config_row, "Stab").pack(side="left")
        tk.Spinbox(config_row, from_=1, to=60, width=3,
                   textvariable=self.sweep_stab_s
                   ).pack(side="left", padx=(2, 0))
        _label(config_row, "s").pack(side="left", padx=(1, 0))

        button_row = tk.Frame(frame, bg=self.PANEL_COLOR)
        button_row.pack(fill="x", pady=(8, 0))

        self.sweep_button = tk.Button(
            button_row, text="▶  Start Sweep Study",
            font=("Helvetica", 10, "bold"),
            bg=self.SUCCESS_COLOR, fg="white",
            activebackground="#196b3a", activeforeground="white",
            command=self._toggle_sweep_study,
            cursor="hand2", padx=12, pady=4, relief="flat",
            borderwidth=0, highlightthickness=0,
        )
        self.sweep_button.pack(side="left")
        self.sweep_button.config(state="disabled")

        self.sweep_status = ttk.Label(
            frame, text="Idle.", style="Muted.TLabel")
        self.sweep_status.pack(anchor="w", pady=(8, 0))

    def _build_power_sweep(self, parent):
        frame = ttk.LabelFrame(parent, text="  Pump Power Sweep  ",
                               style="Panel.TLabelframe", padding=10)
        frame.pack(fill="x", padx=(10, 0), pady=(0, 10))

        config_row = tk.Frame(frame, bg=self.PANEL_COLOR)
        config_row.pack(fill="x")

        def _label(parent_, text):
            return ttk.Label(parent_, text=text, style="TLabel")

        _label(config_row, "Power").pack(side="left")
        tk.Spinbox(config_row, from_=0, to=100, width=4,
                   textvariable=self.psweep_pwr_start
                   ).pack(side="left", padx=(2, 0))
        _label(config_row, "→").pack(side="left", padx=2)
        tk.Spinbox(config_row, from_=0, to=100, width=4,
                   textvariable=self.psweep_pwr_end
                   ).pack(side="left", padx=(0, 6))
        _label(config_row, "step").pack(side="left")
        tk.Spinbox(config_row, from_=1, to=50, width=3,
                   textvariable=self.psweep_pwr_step
                   ).pack(side="left", padx=(2, 12))

        _label(config_row, "Trial").pack(side="left")
        tk.Spinbox(config_row, from_=5, to=600, width=4,
                   textvariable=self.psweep_trial_s
                   ).pack(side="left", padx=(2, 0))
        _label(config_row, "s").pack(side="left", padx=(1, 8))

        _label(config_row, "Stab").pack(side="left")
        tk.Spinbox(config_row, from_=1, to=60, width=3,
                   textvariable=self.psweep_stab_s
                   ).pack(side="left", padx=(2, 0))
        _label(config_row, "s").pack(side="left", padx=(1, 0))

        button_row = tk.Frame(frame, bg=self.PANEL_COLOR)
        button_row.pack(fill="x", pady=(8, 0))

        self.psweep_button = tk.Button(
            button_row, text="▶  Start Power Sweep",
            font=("Helvetica", 10, "bold"),
            bg=self.SUCCESS_COLOR, fg="white",
            activebackground="#196b3a", activeforeground="white",
            command=self._toggle_power_sweep,
            cursor="hand2", padx=12, pady=4, relief="flat",
            borderwidth=0, highlightthickness=0,
        )
        self.psweep_button.pack(side="left")
        self.psweep_button.config(state="disabled")

        self.psweep_status = ttk.Label(
            frame, text="Idle.", style="Muted.TLabel")
        self.psweep_status.pack(anchor="w", pady=(8, 0))

    def _build_graph(self, parent):
        frame = ttk.LabelFrame(parent, text="  Live Pressure Plot  ",
                               style="Panel.TLabelframe", padding=10)
        frame.pack(fill="both", expand=True, pady=(0, 10), padx=(10, 0))

        self.fig = Figure(figsize=(6, 3.6), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self._style_axes(self.ax)
        self.fig.tight_layout()

        self.canvas = FigureCanvasTkAgg(self.fig, master=frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    def _build_log(self, parent):
        frame = ttk.LabelFrame(parent, text="  Communication Log  ",
                               style="Panel.TLabelframe", padding=10)
        frame.pack(fill="x", padx=(10, 0))

        self.log_text = scrolledtext.ScrolledText(
            frame, height=6, wrap="word",
            font=("Menlo", 9), bg="#fafafa",
            relief="flat", borderwidth=1, highlightthickness=0,
        )
        self.log_text.pack(fill="both", expand=True)
        self.log_text.tag_config("sent", foreground="#0066cc")
        self.log_text.tag_config("recv", foreground=self.SUCCESS_COLOR)
        self.log_text.tag_config("err",  foreground=self.DANGER_COLOR)

        ttk.Button(
            frame, text="Clear log",
            command=lambda: self.log_text.delete("1.0", "end"),
        ).pack(anchor="e", pady=(5, 0))

    # === Plot styling =====================================================
    def _style_axes(self, ax):
        """Apply Origin-style frame and ticks to a fresh axes."""
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Pressure (PSI)")
        ax.xaxis.set_minor_locator(AutoMinorLocator(5))
        ax.yaxis.set_minor_locator(AutoMinorLocator(5))
        ax.tick_params(which="both", direction="in",
                       top=True, right=True)
        for spine in ax.spines.values():
            spine.set_linewidth(1.4)
            spine.set_color("black")

    # === Core control logic ==============================================
    def _refresh_ports(self):
        # Get the raw pyserial list (unfiltered) for diagnostic logging
        try:
            raw = serial.tools.list_ports.comports()
        except Exception as e:                              # noqa: BLE001
            self._log(f"Port enumeration failed: "
                      f"{type(e).__name__}: {e}", "err")
            raw = []

        # Now the filtered list we'll actually offer to the user
        ports = self.serial.list_ports()
        self.port_combo["values"] = [f"{p}  ({d})" for p, d in ports]

        # Diagnostic dump to the log so the user can see exactly what
        # pyserial detected and what survived filtering
        self._log(
            f"Refresh: pyserial detected {len(raw)} device(s); "
            f"{len(ports)} usable after macOS tty.* filtering",
            "recv")
        for p in raw:
            note = ""
            if sys.platform == "darwin" and p.device.startswith("/dev/tty."):
                cu_alt = "/dev/cu." + p.device[len("/dev/tty."):]
                if any(rp.device == cu_alt for rp in raw):
                    note = "  [hidden — duplicate of cu.* entry]"
            self._log(f"  • {p.device}  ({p.description}){note}", "recv")

        # Update the visible count label
        if not ports:
            self.port_count_label.config(
                text="No serial ports found — click ? for help.",
                style="Danger.TLabel")
            return
        self.port_count_label.config(
            text=f"{len(ports)} port(s) found",
            style="Muted.TLabel")

        # Auto-select an Arduino-looking entry if we can identify one;
        # otherwise default to the first usable port so the user just
        # has to hit Connect.
        for i, (p, d) in enumerate(ports):
            tag = (p + " " + d).lower()
            if any(k in tag for k in
                   ["arduino", "usbmodem", "ch340", "ch341",
                    "cp210", "wchusbserial", "usbserial", "uno", "nano"]):
                if sys.platform == "darwin" and not p.startswith("/dev/cu."):
                    continue
                self.port_combo.current(i)
                return
        self.port_combo.current(0)

    def _show_port_help(self):
        """Display platform-aware troubleshooting info."""
        if sys.platform == "darwin":
            msg = (
                "Troubleshooting on macOS\n\n"
                "1. Cable check (most common!): make sure your USB-C cable "
                "supports DATA, not just power. Many short USB-C cables "
                "are charge-only — try the cable that came with your "
                "Arduino, or a known-good USB-C data cable.\n\n"
                "2. Open Terminal and run:\n"
                "       ls /dev/cu.*\n"
                "   You should see something like /dev/cu.usbmodem14201 "
                "when the Arduino is plugged in. If you don't, the issue "
                "is at the OS or driver level, not in this script.\n\n"
                "3. Driver check: Arduino UNO/Mega/R4 boards work without "
                "a driver. Boards with CH340/CH341 chips (most cheap "
                "Nano/Pro Mini clones) need the WCH driver from "
                "wch.cn — install the .pkg, then reboot.\n\n"
                "4. Try a different USB port. USB-C hubs sometimes drop "
                "data lines for serial devices."
            )
        else:
            msg = (
                "Troubleshooting\n\n"
                "1. Verify the cable supports data, not just power.\n"
                "2. Check OS device manager for the COM/tty device.\n"
                "3. Install the appropriate USB-Serial driver if needed "
                "(CH340/CH341 chips usually require it)."
            )
        messagebox.showinfo("Serial Port Help", msg)

    def _toggle_connection(self):
        (self._disconnect if self.serial.connected else self._connect)()

    def _connect(self):
        selection = self.port_combo.get()
        if not selection:
            messagebox.showerror("Error", "No port selected")
            return
        port = selection.split("  ")[0]
        success, error = self.serial.connect(port)
        if success:
            self.status_label.config(text="● Connected",
                                     style="Success.TLabel")
            self.connect_button.config(text="Disconnect")
            self._set_controls_enabled(True)
            self.start_time = time.time()
            self._reset_plot_buffers()
            self._log(f"Connected to {port}", "recv")
        else:
            messagebox.showerror("Connection failed", error)

    def _disconnect(self):
        if self._sweep_in_progress:
            self._cancel_sweep_study()
        if self._psweep_in_progress:
            self._cancel_power_sweep()
        if self._cal_in_progress:
            self._cancel_calibration()
        if self.is_recording.get():
            self._toggle_recording()
        self.serial.disconnect()
        self.status_label.config(text="● Disconnected",
                                 style="Danger.TLabel")
        self.connect_button.config(text="Connect")
        self._set_controls_enabled(False)
        self.master_on.set(False)
        self._update_master_button()
        # Reset calibration state — sensors may have moved between sessions
        self.cal_offset_p1 = 0.0
        self.cal_offset_p2 = 0.0
        self.cal_done = False
        self.cal_status_label.config(text="Not calibrated.",
                                     style="Muted.TLabel")
        self.cal_offsets_label.config(text="P1 offset: —    P2 offset: —")
        self.match_hr.set(False)
        self.hr_sensor_label.config(
            text="—  no signal", style="Muted.TLabel")
        self._log("Disconnected", "err")

    def _set_controls_enabled(self, enabled):
        state = "normal" if enabled else "disabled"
        ttk_state = ["!disabled"] if enabled else ["disabled"]
        self.master_button.config(state=state)
        self.record_button.config(state=state)
        self.pump1_scale.state(ttk_state)
        self.pump2_scale.state(ttk_state)
        self.pump1_entry.state(ttk_state)
        self.pump2_entry.state(ttk_state)
        self.bpm_scale.state(ttk_state)
        self.match_hr_check.state(ttk_state)
        self.sweep_button.config(state=state)
        self.psweep_button.config(state=state)
        if enabled:
            self.cal_button.state(["!disabled"])
        else:
            self.cal_button.state(["disabled"])

    def _toggle_master(self):
        if not self.serial.connected:
            return
        self.master_on.set(not self.master_on.get())
        self._update_master_button()
        self._reset_plot_buffers()
        if self.master_on.get():
            self._send_command(f"P1={int(self.pump1_var.get())}")
            self._send_command(f"P2={int(self.pump2_var.get())}")
            self._send_command(f"BPM={int(self.bpm_var.get())}")
            self._send_command(self.mode_var.get())
        else:
            self._send_command("OFF")

    def _update_master_button(self):
        if self.master_on.get():
            self.master_button.config(
                text="⬤   PUMPS  ON",
                bg=self.SUCCESS_COLOR, activebackground="#196b3a")
        else:
            self.master_button.config(
                text="⬤   PUMPS  OFF",
                bg=self.DANGER_COLOR, activebackground="#992d22")

    def _mode_changed(self):
        self._reset_plot_buffers()
        if self.master_on.get():
            self._send_command(self.mode_var.get())

    def _pump1_released(self, _evt=None):
        self._reset_plot_buffers()
        self._send_command(f"P1={int(self.pump1_var.get())}")

    def _pump2_released(self, _evt=None):
        self._reset_plot_buffers()
        self._send_command(f"P2={int(self.pump2_var.get())}")

    def _bpm_released(self, _evt=None):
        new_bpm = int(self.bpm_var.get())
        if new_bpm != self._last_bpm_sent:
            self._last_bpm_sent = new_bpm
            self._reset_plot_buffers()
            self._send_command(f"BPM={new_bpm}")

    def _sync_pct_from_duty(self, pump_num):
        """Slider/IntVar -> entry text. Keeps the entry in sync with the
        slider whenever the underlying duty value changes."""
        var = self.pump1_var if pump_num == 1 else self.pump2_var
        pct_var = (self.pump1_pct_var if pump_num == 1
                   else self.pump2_pct_var)
        try:
            duty = int(var.get())
        except (tk.TclError, ValueError):
            return
        pct = duty * 100.0 / 255.0
        pct_var.set(f"{pct:.1f}")

    def _on_pump_pct_committed(self, pump_num):
        """Entry committed (Enter or focus-out): parse, clamp, update
        slider, send to Arduino."""
        pct_var = (self.pump1_pct_var if pump_num == 1
                   else self.pump2_pct_var)
        var = self.pump1_var if pump_num == 1 else self.pump2_var
        try:
            pct = float(pct_var.get())
        except ValueError:
            # Invalid input — restore display from current slider value
            self._sync_pct_from_duty(pump_num)
            return
        pct = max(0.0, min(100.0, pct))
        duty = int(round(pct * 255.0 / 100.0))
        # Setting the IntVar fires the trace which re-syncs the entry
        # text to the actually-applied (quantized) percentage, so the
        # user sees what was committed.
        var.set(duty)
        if self.serial.connected:
            self._reset_plot_buffers()
            self._send_command(f"P{pump_num}={duty}")

    def _update_bpm_label(self):
        self.bpm_label.config(text=f"{int(self.bpm_var.get())} BPM")

    def _send_command(self, cmd):
        self.serial.send_command(cmd)

    # === Recording ========================================================
    def _toggle_recording(self):
        if not self.is_recording.get():
            filename = datetime.now().strftime(
                "pump_data_%Y%m%d_%H%M%S.csv")
            try:
                self.csv_file = open(filename, "w", newline="")
                self.csv_writer = csv.writer(self.csv_file)
                self.csv_writer.writerow([
                    "Timestamp", "SessionTime(s)", "P1(PSI)", "P2(PSI)",
                    "Mode", "P1_Duty", "P2_Duty", "BPM_Set", "HR_Measured"
                ])
                self.csv_path = os.path.abspath(filename)
                self.is_recording.set(True)
                self.recording_started_at = datetime.now()
                self.recording_params_snapshot = {
                    "mode":  self.mode_var.get(),
                    "p1_pct": int(self.pump1_var.get()) * 100 // 255,
                    "p2_pct": int(self.pump2_var.get()) * 100 // 255,
                    "p1_duty": int(self.pump1_var.get()),
                    "p2_duty": int(self.pump2_var.get()),
                    "bpm":   int(self.bpm_var.get()),
                }
                self._reset_plot_buffers()
                self.record_button.config(
                    text="⏹  Stop Recording",
                    bg=self.DANGER_COLOR, activebackground="#992d22")
                self.record_status.config(
                    text=f"Recording → {filename}",
                    style="Success.TLabel")
            except Exception as e:
                messagebox.showerror("Recording Error",
                                     f"Failed to open file: {e}")
        else:
            # Stop and save snapshot
            if self.csv_file:
                self.csv_file.close()
                self.csv_file = None
                self.csv_writer = None
            self.is_recording.set(False)
            self.record_button.config(
                text="⏺  Start Recording",
                bg=self.ACCENT_COLOR, activebackground="#16306f")

            png_path = self._save_session_snapshot()
            if png_path:
                self.record_status.config(
                    text=f"Saved: {os.path.basename(png_path)}",
                    style="Muted.TLabel")
            else:
                self.record_status.config(
                    text="Not recording.", style="Muted.TLabel")

    def _save_session_snapshot(self):
        """Render the current plot data as a publication-style PNG with a
        parameters footer, alongside the CSV."""
        if not self.csv_path or not self.plot_times:
            return None

        params = self.recording_params_snapshot or {}
        mode = params.get("mode", "?")
        duration = (datetime.now() - self.recording_started_at
                    ).total_seconds() if self.recording_started_at else 0

        # Build a fresh figure so the on-screen one is untouched
        snap_fig = Figure(figsize=(8, 5), dpi=150)
        snap_fig.subplots_adjust(left=0.10, right=0.97,
                                 top=0.92, bottom=0.26)
        ax = snap_fig.add_subplot(111)
        self._style_axes(ax)

        times = list(self.plot_times)
        p1_disp, p2_disp = self._displayed_series(mode)

        ax.plot(times, p1_disp, color=COLOR_P1, linewidth=1.6,
                label="Pressure 1")
        ax.plot(times, p2_disp, color=COLOR_P2, linewidth=1.6,
                label="Pressure 2")
        ax.set_xlim(times[0], times[-1])
        ax.set_ylim(Y_AXIS_MIN_PSI, Y_AXIS_MAX_PSI)
        ax.set_title(f"Pressure vs. Time — {self._mode_human(mode)} Mode")
        ax.legend(loc="upper right")

        # Parameters footer
        footer = self._build_footer_text(params, duration)
        snap_fig.text(
            0.5, 0.02, footer,
            ha="center", va="bottom",
            fontsize=9, family="serif",
            bbox=dict(boxstyle="round,pad=0.4",
                      facecolor="#fafafa",
                      edgecolor="#cccccc"),
        )

        png_path = os.path.splitext(self.csv_path)[0] + ".png"
        try:
            snap_fig.savefig(png_path, dpi=150, facecolor="white")
            self._log(f"Snapshot saved: {os.path.basename(png_path)}",
                      "recv")
            return png_path
        except Exception as e:
            self._log(f"Snapshot failed: {e}", "err")
            return None

    def _build_footer_text(self, params, duration_s):
        mode_h = self._mode_human(params.get("mode", "?"))
        bpm_part = (f"  ·  BPM: {params.get('bpm','?')}"
                    if params.get("mode") == "HEART" else "")
        ts = (self.recording_started_at.strftime("%Y-%m-%d %H:%M:%S")
              if self.recording_started_at else "—")
        if self.cal_done:
            cal_part = (f"Calibrated (P1 {self.cal_offset_p1:+.3f}, "
                        f"P2 {self.cal_offset_p2:+.3f} PSI)")
        else:
            cal_part = "Not calibrated"
        return (
            f"Mode: {mode_h}{bpm_part}  ·  "
            f"Pump 1: {params.get('p1_pct','?')}% "
            f"(duty {params.get('p1_duty','?')}/255)  ·  "
            f"Pump 2: {params.get('p2_pct','?')}% "
            f"(duty {params.get('p2_duty','?')}/255)\n"
            f"Duration: {duration_s:.1f} s  ·  "
            f"Sample rate: {ARDUINO_REPORT_HZ} Hz  ·  "
            f"Display smoothing: "
            f"{self._smoothing_label(params.get('mode','?'))}\n"
            f"{cal_part}  ·  Recorded: {ts}"
        )

    @staticmethod
    def _mode_human(mode):
        return {"CONT": "Continuous", "HEART": "Heartbeat",
                "ALT": "Alternating", "OFF": "Off"}.get(mode, mode)

    def _smoothing_label(self, mode):
        if mode == "HEART":
            return f"{HEART_MEDIAN_WINDOW}-pt rolling median"
        return f"{ROLLING_MEAN_SECONDS:.0f}-s rolling mean"

    # === Calibration ======================================================
    # Sequence: prime pumps -> stop -> let pressure settle -> capture baseline.
    # The captured P1/P2 averages become the zero offsets for all subsequent
    # readings. Each step is scheduled with root.after so the GUI stays
    # responsive and incoming DATA messages are still polled normally.
    def _start_calibration(self):
        if not self.serial.connected:
            return
        if self._cal_in_progress:
            return
        # If pumps are currently running, turn them off first via the master
        if self.master_on.get():
            self.master_on.set(False)
            self._update_master_button()

        self._cal_in_progress = True
        self._cal_collecting = False
        self._cal_samples_p1 = []
        self._cal_samples_p2 = []
        self.cal_done = False
        self._set_controls_during_cal(locked=True)
        self._reset_plot_buffers()
        self._log("Calibration started", "sent")

        # Make sure we begin from a known state: pumps off, then ramp
        # to 100% for the prime phase. This guarantees calibration
        # always starts the pumps at full power.
        self._send_command("OFF")
        self.cal_status_label.config(
            text=f"Priming pumps at 100% "
                 f"({CAL_PRIME_DURATION_S:.0f} s)…",
            style="Success.TLabel")
        self._send_command(f"P1={CAL_PRIME_DUTY}")
        self._send_command(f"P2={CAL_PRIME_DUTY}")
        self._send_command("CONT")
        self.root.after(int(CAL_PRIME_DURATION_S * 1000),
                        self._cal_step_settle)

    def _cal_step_settle(self):
        if not self._cal_in_progress:
            return
        self.cal_status_label.config(
            text=f"Stopping pumps, waiting to settle "
                 f"({CAL_SETTLE_DURATION_S:.0f} s)…",
            style="Success.TLabel")
        self._send_command("OFF")
        self.root.after(int(CAL_SETTLE_DURATION_S * 1000),
                        self._cal_step_collect)

    def _cal_step_collect(self):
        if not self._cal_in_progress:
            return
        self.cal_status_label.config(
            text=f"Capturing zero baseline "
                 f"({CAL_BASELINE_DURATION_S:.0f} s)…",
            style="Success.TLabel")
        # NOTE: when collecting, _poll_messages appends raw values to
        # self._cal_samples_p1/p2 instead of applying offsets.
        self._cal_samples_p1 = []
        self._cal_samples_p2 = []
        self._cal_collecting = True
        self.root.after(int(CAL_BASELINE_DURATION_S * 1000),
                        self._cal_step_finish)

    def _cal_step_finish(self):
        if not self._cal_in_progress:
            return
        self._cal_collecting = False
        n1, n2 = len(self._cal_samples_p1), len(self._cal_samples_p2)
        if n1 < 5 or n2 < 5:
            self.cal_status_label.config(
                text=f"Calibration failed (only {min(n1,n2)} samples).",
                style="Danger.TLabel")
            self._log(f"Calibration failed: too few samples ({n1}/{n2})",
                      "err")
            self._cal_in_progress = False
            self._set_controls_during_cal(locked=False)
            return

        self.cal_offset_p1 = sum(self._cal_samples_p1) / n1
        self.cal_offset_p2 = sum(self._cal_samples_p2) / n2

        self._log(
            f"Zero baseline captured: P1 {self.cal_offset_p1:+.3f}, "
            f"P2 {self.cal_offset_p2:+.3f} PSI", "recv")

        # Skip directly to verification — no deadband sweep
        self._cal_verify_step()

    def _cal_verify_step(self):
        """Run both pumps briefly at full power so the user can confirm
        visually that both pumps are responding before the calibration
        is declared complete."""
        if not self._cal_in_progress:
            return
        self.cal_status_label.config(
            text=f"Verification: running both pumps at 100% "
                 f"for {CAL_VERIFY_DURATION_S:.0f} s…",
            style="Success.TLabel")
        self._send_command("P1=255")
        self._send_command("P2=255")
        self._send_command("CONT")
        self.root.after(int(CAL_VERIFY_DURATION_S * 1000),
                        self._cal_finalize)

    def _cal_finalize(self):
        # End the verification run; pumps off
        self._send_command("OFF")
        self.cal_done = True
        self.cal_status_label.config(
            text="Calibration complete.", style="Success.TLabel")
        offsets = (f"P1 offset: {self.cal_offset_p1:+.3f} PSI    "
                   f"P2 offset: {self.cal_offset_p2:+.3f} PSI")
        self.cal_offsets_label.config(text=offsets)
        self._log("Calibration complete", "recv")
        self._cal_in_progress = False
        self._set_controls_during_cal(locked=False)
        self._reset_plot_buffers()

    def _cancel_calibration(self):
        """Abort an in-progress calibration (e.g. on disconnect)."""
        if not self._cal_in_progress:
            return
        self._cal_in_progress = False
        self._cal_collecting = False
        try:
            self._send_command("OFF")
        except Exception:
            pass
        self.cal_status_label.config(
            text="Calibration cancelled.", style="Danger.TLabel")
        self._set_controls_during_cal(locked=False)

    def _set_controls_during_cal(self, locked):
        """Lock or unlock user controls for the duration of calibration."""
        state = "disabled" if locked else "normal"
        ttk_state = ["disabled"] if locked else ["!disabled"]
        self.master_button.config(state=state)
        self.record_button.config(state=state)
        self.pump1_scale.state(ttk_state)
        self.pump2_scale.state(ttk_state)
        self.pump1_entry.state(ttk_state)
        self.pump2_entry.state(ttk_state)
        self.bpm_scale.state(ttk_state)
        if locked:
            self.cal_button.state(["disabled"])
        else:
            self.cal_button.state(["!disabled"])

    # === Heart-rate sensor handling ======================================
    def _handle_hr_reading(self, hr):
        """Update sensor display and (optionally) drive heartbeat BPM."""
        if hr <= 0 or hr < 30 or hr > 200:
            self._hr_recent.clear()
            self.hr_sensor_label.config(
                text="—  no signal", style="Muted.TLabel")
            return

        self._hr_recent.append(hr)
        # Show "acquiring" until we have enough samples to median-smooth,
        # rather than displaying whatever the first un-stabilized BPM is.
        if len(self._hr_recent) < 3:
            self.hr_sensor_label.config(
                text="…  acquiring", style="Muted.TLabel")
            return

        # Median is robust to occasional bad beats that slip past Arduino's
        # filter. With a 5-sample window, even 2 outliers can't move the
        # reported value.
        sorted_recent = sorted(self._hr_recent)
        smoothed_int = int(round(sorted_recent[len(sorted_recent) // 2]))
        self.hr_sensor_label.config(
            text=f"{smoothed_int} BPM   ●  live",
            style="Success.TLabel")

        if not self.match_hr.get():
            return
        now_ms = int(time.time() * 1000)
        if (now_ms - self._last_hr_send_ms) < HR_MATCH_DEBOUNCE_MS:
            return
        if abs(smoothed_int - self._last_hr_sent_bpm) < HR_MATCH_DELTA_BPM:
            return
        target = max(30, min(180, smoothed_int))
        self.bpm_var.set(target)
        self._last_bpm_sent = target
        self._last_hr_sent_bpm = target
        self._last_hr_send_ms = now_ms
        self._send_command(f"BPM={target}")

    def _on_match_hr_toggle(self):
        if self.match_hr.get():
            # Disable BPM slider — sensor is now in charge
            self.bpm_scale.state(["disabled"])
            self._log("BPM-match enabled (sensor drives heartbeat rate)",
                      "sent")
            # Reset debounce so a fresh value can fire immediately
            self._last_hr_send_ms = 0
        else:
            self.bpm_scale.state(["!disabled"])
            self._log("BPM-match disabled", "sent")

    # === BPM Sweep Study =================================================
    def _toggle_sweep_study(self):
        if self._sweep_in_progress:
            self._cancel_sweep_study()
        else:
            self._start_sweep_study()

    def _start_sweep_study(self):
        if not self.serial.connected:
            return
        if self._cal_in_progress:
            messagebox.showwarning(
                "Sweep Study",
                "Calibration is in progress. Please wait until it completes.")
            return
        # Validate config
        try:
            bpm_start = int(self.sweep_bpm_start.get())
            bpm_end   = int(self.sweep_bpm_end.get())
            bpm_step  = int(self.sweep_bpm_step.get())
            trial_s   = int(self.sweep_trial_s.get())
            stab_s    = int(self.sweep_stab_s.get())
        except (tk.TclError, ValueError):
            messagebox.showerror("Sweep Study",
                                 "Invalid configuration values.")
            return
        if bpm_step <= 0 or bpm_end < bpm_start or trial_s < 5:
            messagebox.showerror(
                "Sweep Study",
                "Invalid configuration: check BPM range, step, and "
                "trial duration.")
            return
        bpm_list = list(range(bpm_start, bpm_end + 1, bpm_step))
        n_trials = len(bpm_list)
        total_min = n_trials * (trial_s + stab_s) / 60.0

        # Capture pump strengths now so they're locked for the campaign
        p1_pct = int(self.pump1_var.get()) * 100 // 255
        p2_pct = int(self.pump2_var.get()) * 100 // 255

        confirm = messagebox.askyesno(
            "Sweep Study",
            f"Run BPM sweep study?\n\n"
            f"Trials: {n_trials}  ({bpm_start} → {bpm_end} BPM, "
            f"step {bpm_step})\n"
            f"Per trial: {trial_s} s recording + {stab_s} s settle\n"
            f"Pump 1: {p1_pct}%   Pump 2: {p2_pct}%\n"
            f"Estimated total time: ~{total_min:.1f} minutes\n\n"
            f"All other controls will be locked until the study completes "
            f"or is cancelled."
        )
        if not confirm:
            return

        # Build output directory
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._sweep_dir = os.path.abspath(f"bpm_sweep_{ts}")
        try:
            os.makedirs(self._sweep_dir, exist_ok=True)
        except OSError as e:
            messagebox.showerror("Sweep Study",
                                 f"Could not create output folder: {e}")
            return

        self._sweep_in_progress = True
        self._sweep_started_at = datetime.now()
        self._sweep_bpm_list = bpm_list
        self._sweep_trial_idx = 0
        self._sweep_trial_results = []

        # Lock UI; keep the sweep button enabled to allow cancel
        self._set_controls_during_sweep(locked=True)
        self.sweep_button.config(
            text="⏹  Cancel Study",
            bg=self.DANGER_COLOR, activebackground="#992d22")

        # Engage pumps in HEART mode at the user's chosen strengths
        self.master_on.set(True)
        self._update_master_button()
        d1 = int(self.pump1_var.get())
        d2 = int(self.pump2_var.get())
        self._send_command(f"P1={d1}")
        self._send_command(f"P2={d2}")
        self.mode_var.set("HEART")
        self._send_command("HEART")
        self._log(f"BPM sweep study started → {self._sweep_dir}", "sent")

        self._sweep_begin_trial()

    def _sweep_begin_trial(self):
        """Start the next trial: set BPM, enter STABILIZE state."""
        if not self._sweep_in_progress:
            return
        if self._sweep_trial_idx >= len(self._sweep_bpm_list):
            self._sweep_finish_study()
            return

        bpm = self._sweep_bpm_list[self._sweep_trial_idx]
        self.bpm_var.set(bpm)
        self._send_command(f"BPM={bpm}")
        self._reset_plot_buffers()

        self._sweep_state = "STABILIZE"
        stab_ms = int(SWEEP_STABILIZE_S * 1000)
        try:
            stab_ms = int(self.sweep_stab_s.get()) * 1000
        except (tk.TclError, ValueError):
            pass
        self._sweep_state_until_ms = int(time.time() * 1000) + stab_ms
        self._update_sweep_status()
        self.root.after(200, self._sweep_tick)

    def _sweep_tick(self):
        """Periodic state-machine tick (200 ms)."""
        if not self._sweep_in_progress:
            return
        now_ms = int(time.time() * 1000)

        if self._sweep_state == "STABILIZE":
            if now_ms >= self._sweep_state_until_ms:
                self._sweep_start_recording()
            else:
                self._update_sweep_status()
                self.root.after(200, self._sweep_tick)

        elif self._sweep_state == "RECORDING":
            if now_ms >= self._sweep_state_until_ms:
                self._sweep_stop_recording_and_advance()
            else:
                self._update_sweep_status()
                self.root.after(200, self._sweep_tick)

    def _sweep_start_recording(self):
        bpm = self._sweep_bpm_list[self._sweep_trial_idx]
        # Open a per-trial CSV
        path = os.path.join(self._sweep_dir, f"bpm_{bpm:03d}.csv")
        try:
            self._sweep_csv_file = open(path, "w", newline="")
            self._sweep_csv_writer = csv.writer(self._sweep_csv_file)
            self._sweep_csv_writer.writerow([
                "Timestamp", "TrialTime(s)", "P1(PSI)", "P2(PSI)",
                "Mode", "P1_Duty", "P2_Duty", "BPM"
            ])
        except OSError as e:
            self._log(f"Trial CSV open failed: {e}", "err")
            self._cancel_sweep_study()
            return

        self._sweep_trial_t  = []
        self._sweep_trial_p1 = []
        self._sweep_trial_p2 = []
        self._reset_plot_buffers()
        self._sweep_state = "RECORDING"
        try:
            trial_ms = int(self.sweep_trial_s.get()) * 1000
        except (tk.TclError, ValueError):
            trial_ms = int(SWEEP_TRIAL_DURATION_S * 1000)
        self._sweep_state_until_ms = int(time.time() * 1000) + trial_ms
        self._sweep_trial_started_at = time.time()
        self._update_sweep_status()
        self.root.after(200, self._sweep_tick)

    def _sweep_stop_recording_and_advance(self):
        bpm = self._sweep_bpm_list[self._sweep_trial_idx]
        # Close CSV
        if self._sweep_csv_file:
            try:
                self._sweep_csv_file.close()
            except Exception:
                pass
        self._sweep_csv_file = None
        self._sweep_csv_writer = None

        # Detect peaks and produce per-trial outputs
        result = self._sweep_analyze_trial(bpm)
        self._sweep_trial_results.append(result)
        self._save_sweep_trial_snapshot(bpm, result)

        self._log(
            f"Trial BPM {bpm} → mean peak P1={result['p1_mean']:.3f} ± "
            f"{result['p1_std']:.3f}, P2={result['p2_mean']:.3f} ± "
            f"{result['p2_std']:.3f} PSI "
            f"(n_p1={result['p1_n']}, n_p2={result['p2_n']})",
            "recv")

        self._sweep_trial_idx += 1
        # Brief settle between trials
        self.root.after(500, self._sweep_begin_trial)

    def _sweep_analyze_trial(self, bpm):
        """Detect peaks in the trial buffers and compute statistics."""
        p1_idx, p1_heights = self._find_pressure_peaks(
            self._sweep_trial_p1, bpm)
        p2_idx, p2_heights = self._find_pressure_peaks(
            self._sweep_trial_p2, bpm)

        def _mean_std(xs):
            if not xs:
                return float("nan"), float("nan")
            arr = np.asarray(xs, dtype=float)
            return float(arr.mean()), float(arr.std(ddof=0))

        p1_mean, p1_std = _mean_std(p1_heights)
        p2_mean, p2_std = _mean_std(p2_heights)
        return {
            "bpm": bpm,
            "p1_mean": p1_mean, "p1_std": p1_std, "p1_n": len(p1_heights),
            "p2_mean": p2_mean, "p2_std": p2_std, "p2_n": len(p2_heights),
            "p1_peak_idx": list(p1_idx),
            "p2_peak_idx": list(p2_idx),
            "p1_heights":  list(p1_heights),
            "p2_heights":  list(p2_heights),
            "t":  list(self._sweep_trial_t),
            "p1": list(self._sweep_trial_p1),
            "p2": list(self._sweep_trial_p2),
        }

    @staticmethod
    def _find_pressure_peaks(data, bpm, fs=ARDUINO_REPORT_HZ):
        """Detect heartbeat-related pressure peaks. Returns (indices, heights)."""
        if len(data) < 10:
            return [], []
        arr = np.asarray(data, dtype=float)
        amin, amax = float(arr.min()), float(arr.max())
        if amax - amin < 0.05:
            return [], []
        amean = float(arr.mean())

        expected_period_s = 60.0 / max(1, bpm)
        min_distance = max(2, int(SWEEP_PEAK_DISTANCE_FRAC * expected_period_s * fs))
        min_height = amean + SWEEP_PEAK_HEIGHT_FRAC * (amax - amean)
        prominence = SWEEP_PEAK_PROMINENCE_FRAC * (amax - amin)

        if HAS_SCIPY:
            idx, _ = _scipy_find_peaks(
                arr, distance=min_distance,
                height=min_height, prominence=prominence)
            return list(idx), [float(arr[i]) for i in idx]

        # Pure-numpy fallback: local maxima above threshold + min spacing
        candidates = []
        for i in range(1, len(arr) - 1):
            if arr[i] >= arr[i - 1] and arr[i] >= arr[i + 1] \
                    and arr[i] >= min_height:
                candidates.append(i)
        if not candidates:
            return [], []
        selected = [candidates[0]]
        for c in candidates[1:]:
            if c - selected[-1] >= min_distance:
                selected.append(c)
            elif arr[c] > arr[selected[-1]]:
                selected[-1] = c
        return selected, [float(arr[i]) for i in selected]

    def _save_sweep_trial_snapshot(self, bpm, result):
        """Render per-trial PNG with peak markers and mean-peak annotations."""
        if not result["t"]:
            return
        fig = Figure(figsize=(8, 5), dpi=150)
        fig.subplots_adjust(left=0.10, right=0.97, top=0.92, bottom=0.26)
        ax = fig.add_subplot(111)
        self._style_axes(ax)

        t = result["t"]
        # Apply the same EKG-style smoothing for display consistency
        p1_disp = self._rolling_median(result["p1"], HEART_MEDIAN_WINDOW)
        p2_disp = self._rolling_median(result["p2"], HEART_MEDIAN_WINDOW)

        ax.plot(t, p1_disp, color=COLOR_P1, linewidth=1.4,
                label="Pressure 1", zorder=2)
        ax.plot(t, p2_disp, color=COLOR_P2, linewidth=1.4,
                label="Pressure 2", zorder=2)

        # Peak markers (use raw values & raw indices)
        if result["p1_peak_idx"]:
            ax.plot([t[i] for i in result["p1_peak_idx"]],
                    [result["p1"][i] for i in result["p1_peak_idx"]],
                    "v", color=COLOR_P1, markersize=5,
                    markeredgecolor="black", markeredgewidth=0.5,
                    linestyle="None", zorder=3)
        if result["p2_peak_idx"]:
            ax.plot([t[i] for i in result["p2_peak_idx"]],
                    [result["p2"][i] for i in result["p2_peak_idx"]],
                    "v", color=COLOR_P2, markersize=5,
                    markeredgecolor="black", markeredgewidth=0.5,
                    linestyle="None", zorder=3)

        # Mean-peak horizontal lines
        if not np.isnan(result["p1_mean"]):
            ax.axhline(result["p1_mean"], color=COLOR_P1,
                       linestyle="--", linewidth=1.0, alpha=0.7, zorder=1)
        if not np.isnan(result["p2_mean"]):
            ax.axhline(result["p2_mean"], color=COLOR_P2,
                       linestyle="--", linewidth=1.0, alpha=0.7, zorder=1)

        ax.set_xlim(t[0], t[-1])
        ax.set_ylim(Y_AXIS_MIN_PSI, Y_AXIS_MAX_PSI)
        title = (f"BPM {bpm} Trial  —  Mean peak: "
                 f"P1 = {result['p1_mean']:.3f} ± {result['p1_std']:.3f},  "
                 f"P2 = {result['p2_mean']:.3f} ± {result['p2_std']:.3f} PSI")
        ax.set_title(title, fontsize=11)
        ax.legend(loc="upper right")

        p1_pct = int(self.pump1_var.get()) * 100 // 255
        p2_pct = int(self.pump2_var.get()) * 100 // 255
        cal_part = ("Calibrated" if self.cal_done else "Not calibrated")
        footer = (
            f"BPM Sweep Trial  ·  BPM: {bpm}  ·  "
            f"Pump 1: {p1_pct}%   ·  Pump 2: {p2_pct}%\n"
            f"Trial duration: {len(t)/ARDUINO_REPORT_HZ:.1f} s  ·  "
            f"Sample rate: {ARDUINO_REPORT_HZ} Hz  ·  "
            f"Smoothing: {HEART_MEDIAN_WINDOW}-pt rolling median\n"
            f"Peaks detected: P1 = {result['p1_n']}, P2 = {result['p2_n']}"
            f"  ·  {cal_part}"
        )
        fig.text(0.5, 0.02, footer, ha="center", va="bottom", fontsize=9,
                 family="serif",
                 bbox=dict(boxstyle="round,pad=0.4",
                           facecolor="#fafafa", edgecolor="#cccccc"))

        png_path = os.path.join(self._sweep_dir, f"bpm_{bpm:03d}.png")
        try:
            fig.savefig(png_path, dpi=150, facecolor="white")
        except Exception as e:
            self._log(f"Trial snapshot save failed: {e}", "err")

    def _save_sweep_summary(self):
        """Generate the BPM-vs-peak-pressure summary CSV and PNG."""
        if not self._sweep_trial_results:
            return None, None
        results = self._sweep_trial_results

        # Summary CSV
        csv_path = os.path.join(self._sweep_dir, "summary.csv")
        try:
            with open(csv_path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow([
                    "BPM",
                    "P1_mean_peak_PSI", "P1_std_peak_PSI", "P1_peak_count",
                    "P2_mean_peak_PSI", "P2_std_peak_PSI", "P2_peak_count",
                ])
                for r in results:
                    w.writerow([
                        r["bpm"],
                        f"{r['p1_mean']:.4f}", f"{r['p1_std']:.4f}", r['p1_n'],
                        f"{r['p2_mean']:.4f}", f"{r['p2_std']:.4f}", r['p2_n'],
                    ])
        except OSError as e:
            self._log(f"Summary CSV save failed: {e}", "err")
            csv_path = None

        # Summary PNG
        bpms = [r["bpm"] for r in results]
        p1_m  = [r["p1_mean"] for r in results]
        p1_s  = [r["p1_std"]  for r in results]
        p2_m  = [r["p2_mean"] for r in results]
        p2_s  = [r["p2_std"]  for r in results]

        fig = Figure(figsize=(8, 5.2), dpi=150)
        fig.subplots_adjust(left=0.11, right=0.97, top=0.92, bottom=0.22)
        ax = fig.add_subplot(111)
        self._style_axes(ax)
        ax.set_xlabel("Heart Rate (BPM)")
        ax.set_ylabel("Mean Peak Pressure (PSI)")

        ax.errorbar(bpms, p1_m, yerr=p1_s,
                    fmt="o-", color=COLOR_P1, markersize=6,
                    markerfacecolor=COLOR_P1, markeredgecolor="black",
                    markeredgewidth=0.6, linewidth=1.4, capsize=3,
                    label="Pump 1")
        ax.errorbar(bpms, p2_m, yerr=p2_s,
                    fmt="s-", color=COLOR_P2, markersize=6,
                    markerfacecolor=COLOR_P2, markeredgecolor="black",
                    markeredgewidth=0.6, linewidth=1.4, capsize=3,
                    label="Pump 2")

        ax.set_xlim(min(bpms) - 5, max(bpms) + 5)
        # Auto y-range with a tidy 0 floor when data is positive
        finite_y = [v for v in (p1_m + p2_m) if not np.isnan(v)]
        if finite_y:
            ymax = max(finite_y) + max(0.1, 1.2 * max(p1_s + p2_s))
            ax.set_ylim(0, ymax)

        ax.set_title("Peak Pressure vs. Heart Rate")
        ax.legend(loc="best")

        p1_pct = int(self.pump1_var.get()) * 100 // 255
        p2_pct = int(self.pump2_var.get()) * 100 // 255
        try:
            trial_s = int(self.sweep_trial_s.get())
        except (tk.TclError, ValueError):
            trial_s = int(SWEEP_TRIAL_DURATION_S)
        ts_started = (self._sweep_started_at.strftime("%Y-%m-%d %H:%M:%S")
                      if self._sweep_started_at else "—")
        cal_part = ("Calibrated" if self.cal_done else "Not calibrated")
        footer = (
            f"BPM Sweep Summary  ·  Trials: {len(results)}  ·  "
            f"BPM range: {min(bpms)} – {max(bpms)}  ·  "
            f"Pump 1: {p1_pct}%   ·  Pump 2: {p2_pct}%\n"
            f"Per-trial duration: {trial_s} s  ·  "
            f"Sample rate: {ARDUINO_REPORT_HZ} Hz  ·  "
            f"Error bars: ±1 σ of within-trial peaks  ·  {cal_part}\n"
            f"Started: {ts_started}"
        )
        fig.text(0.5, 0.02, footer, ha="center", va="bottom", fontsize=9,
                 family="serif",
                 bbox=dict(boxstyle="round,pad=0.4",
                           facecolor="#fafafa", edgecolor="#cccccc"))

        png_path = os.path.join(self._sweep_dir, "summary.png")
        try:
            fig.savefig(png_path, dpi=150, facecolor="white")
        except Exception as e:
            self._log(f"Summary plot save failed: {e}", "err")
            png_path = None

        return csv_path, png_path

    def _sweep_finish_study(self):
        """All trials complete; finalize."""
        self._send_command("OFF")
        self.master_on.set(False)
        self._update_master_button()

        csv_path, png_path = self._save_sweep_summary()

        self._sweep_in_progress = False
        self._sweep_state = "IDLE"
        self._set_controls_during_sweep(locked=False)
        self.sweep_button.config(
            text="▶  Start Sweep Study",
            bg=self.SUCCESS_COLOR, activebackground="#196b3a")

        n = len(self._sweep_trial_results)
        msg = (f"Sweep study complete — {n} trial(s) saved to "
               f"{os.path.basename(self._sweep_dir)}/")
        self.sweep_status.config(text=msg, style="Success.TLabel")
        self._log(msg, "recv")
        if png_path:
            self._log(f"Summary saved: {os.path.basename(png_path)}",
                      "recv")

    def _cancel_sweep_study(self):
        if not self._sweep_in_progress:
            return
        # Save partial summary if anything was recorded
        if self._sweep_csv_file:
            try:
                self._sweep_csv_file.close()
            except Exception:
                pass
            self._sweep_csv_file = None
            self._sweep_csv_writer = None

        self._send_command("OFF")
        self.master_on.set(False)
        self._update_master_button()

        if self._sweep_trial_results:
            try:
                self._save_sweep_summary()
            except Exception:
                pass

        self._sweep_in_progress = False
        self._sweep_state = "IDLE"
        self._set_controls_during_sweep(locked=False)
        self.sweep_button.config(
            text="▶  Start Sweep Study",
            bg=self.SUCCESS_COLOR, activebackground="#196b3a")
        self.sweep_status.config(text="Sweep cancelled.",
                                 style="Danger.TLabel")
        self._log("Sweep cancelled by user", "err")

    def _set_controls_during_sweep(self, locked):
        state = "disabled" if locked else "normal"
        ttk_state = ["disabled"] if locked else ["!disabled"]
        self.master_button.config(state=state)
        self.record_button.config(state=state)
        self.pump1_scale.state(ttk_state)
        self.pump2_scale.state(ttk_state)
        self.pump1_entry.state(ttk_state)
        self.pump2_entry.state(ttk_state)
        self.bpm_scale.state(ttk_state)
        self.match_hr_check.state(ttk_state)
        # Lock the OTHER sweep panel too
        if locked:
            self.cal_button.state(["disabled"])
            self.psweep_button.config(state="disabled")
        else:
            self.cal_button.state(["!disabled"])
            self.psweep_button.config(state="normal")

    def _update_sweep_status(self):
        if not self._sweep_in_progress:
            return
        idx = self._sweep_trial_idx + 1
        n_total = len(self._sweep_bpm_list)
        bpm = self._sweep_bpm_list[self._sweep_trial_idx]
        remaining_ms = max(0, self._sweep_state_until_ms
                           - int(time.time() * 1000))
        remaining_s = remaining_ms / 1000.0
        if self._sweep_state == "STABILIZE":
            phase = f"Stabilizing — {remaining_s:.0f}s left"
        elif self._sweep_state == "RECORDING":
            phase = f"Recording — {remaining_s:.0f}s left"
        else:
            phase = self._sweep_state
        self.sweep_status.config(
            text=f"Trial {idx}/{n_total}  ·  BPM {bpm}  ·  {phase}",
            style="Success.TLabel")

    # === Pump Power Sweep Study ==========================================
    # Same shape as the BPM sweep, but iterates over CONT-mode pump power
    # values and measures the time-averaged pressure at each step. With
    # the Arduino's fast slow-PWM in CONT mode (20 Hz cycle), the pumps
    # run continuously — rotor and fluid inertia carry through each OFF
    # window, so there's no visible cycle ripple and the time-averaged
    # pressure reflects the duty ratio directly.
    def _toggle_power_sweep(self):
        if self._psweep_in_progress:
            self._cancel_power_sweep()
        else:
            self._start_power_sweep()

    def _start_power_sweep(self):
        if not self.serial.connected:
            return
        if self._cal_in_progress or self._sweep_in_progress:
            messagebox.showwarning(
                "Power Sweep",
                "Another operation is in progress. Wait for it to finish.")
            return
        try:
            pwr_start = int(self.psweep_pwr_start.get())
            pwr_end   = int(self.psweep_pwr_end.get())
            pwr_step  = int(self.psweep_pwr_step.get())
            trial_s   = int(self.psweep_trial_s.get())
            stab_s    = int(self.psweep_stab_s.get())
        except (tk.TclError, ValueError):
            messagebox.showerror("Power Sweep",
                                 "Invalid configuration values.")
            return
        if (pwr_step <= 0 or pwr_end < pwr_start
                or pwr_start < 0 or pwr_end > 100 or trial_s < 5):
            messagebox.showerror(
                "Power Sweep",
                "Invalid configuration: check power range (0-100), "
                "step, and trial duration.")
            return

        pwr_list = list(range(pwr_start, pwr_end + 1, pwr_step))
        n_trials = len(pwr_list)
        total_min = n_trials * (trial_s + stab_s) / 60.0

        confirm = messagebox.askyesno(
            "Power Sweep",
            f"Run pump power sweep?\n\n"
            f"Trials: {n_trials}  ({pwr_start}% → {pwr_end}%, "
            f"step {pwr_step}%)\n"
            f"Per trial: {trial_s} s recording + {stab_s} s settle\n"
            f"Mode: CONT (continuous, 20 Hz duty cycle)\n"
            f"Estimated total time: ~{total_min:.1f} minutes\n\n"
            f"Both pumps will be set to the same power at each step. "
            f"Other controls will be locked until the sweep completes "
            f"or is cancelled."
        )
        if not confirm:
            return

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._psweep_dir = os.path.abspath(f"power_sweep_{ts}")
        try:
            os.makedirs(self._psweep_dir, exist_ok=True)
        except OSError as e:
            messagebox.showerror("Power Sweep",
                                 f"Could not create output folder: {e}")
            return

        self._psweep_in_progress = True
        self._psweep_started_at = datetime.now()
        self._psweep_pwr_list = pwr_list
        self._psweep_trial_idx = 0
        self._psweep_trial_results = []

        self._set_controls_during_power_sweep(locked=True)
        self.psweep_button.config(
            text="⏹  Cancel Sweep",
            bg=self.DANGER_COLOR, activebackground="#992d22")

        # CONT mode, pumps off initially. We'll set per-trial power and
        # bring up the master in _psweep_begin_trial.
        self.mode_var.set("CONT")
        self.master_on.set(True)
        self._update_master_button()
        self._send_command("CONT")
        self._log(f"Power sweep started → {self._psweep_dir}", "sent")

        self._psweep_begin_trial()

    def _psweep_begin_trial(self):
        if not self._psweep_in_progress:
            return
        if self._psweep_trial_idx >= len(self._psweep_pwr_list):
            self._psweep_finish()
            return
        pwr_pct = self._psweep_pwr_list[self._psweep_trial_idx]
        duty = int(round(pwr_pct * 255.0 / 100.0))
        # Set both pumps to the same power
        self.pump1_var.set(duty)
        self.pump2_var.set(duty)
        self._send_command(f"P1={duty}")
        self._send_command(f"P2={duty}")
        self._reset_plot_buffers()

        self._psweep_state = "STABILIZE"
        try:
            stab_ms = int(self.psweep_stab_s.get()) * 1000
        except (tk.TclError, ValueError):
            stab_ms = int(PSWEEP_STABILIZE_S * 1000)
        self._psweep_state_until_ms = int(time.time() * 1000) + stab_ms
        self._update_psweep_status()
        self.root.after(200, self._psweep_tick)

    def _psweep_tick(self):
        if not self._psweep_in_progress:
            return
        now_ms = int(time.time() * 1000)
        if self._psweep_state == "STABILIZE":
            if now_ms >= self._psweep_state_until_ms:
                self._psweep_start_recording()
            else:
                self._update_psweep_status()
                self.root.after(200, self._psweep_tick)
        elif self._psweep_state == "RECORDING":
            if now_ms >= self._psweep_state_until_ms:
                self._psweep_stop_recording_and_advance()
            else:
                self._update_psweep_status()
                self.root.after(200, self._psweep_tick)

    def _psweep_start_recording(self):
        pwr_pct = self._psweep_pwr_list[self._psweep_trial_idx]
        path = os.path.join(self._psweep_dir, f"power_{pwr_pct:03d}.csv")
        try:
            self._psweep_csv_file = open(path, "w", newline="")
            self._psweep_csv_writer = csv.writer(self._psweep_csv_file)
            self._psweep_csv_writer.writerow([
                "Timestamp", "TrialTime(s)", "P1(PSI)", "P2(PSI)",
                "Mode", "P1_Duty", "P2_Duty", "Power(%)"
            ])
        except OSError as e:
            self._log(f"Trial CSV open failed: {e}", "err")
            self._cancel_power_sweep()
            return

        self._psweep_trial_t  = []
        self._psweep_trial_p1 = []
        self._psweep_trial_p2 = []
        self._reset_plot_buffers()
        self._psweep_state = "RECORDING"
        try:
            trial_ms = int(self.psweep_trial_s.get()) * 1000
        except (tk.TclError, ValueError):
            trial_ms = int(PSWEEP_TRIAL_DURATION_S * 1000)
        self._psweep_state_until_ms = int(time.time() * 1000) + trial_ms
        self._psweep_trial_started_at = time.time()
        self._update_psweep_status()
        self.root.after(200, self._psweep_tick)

    def _psweep_stop_recording_and_advance(self):
        pwr_pct = self._psweep_pwr_list[self._psweep_trial_idx]
        if self._psweep_csv_file:
            try:
                self._psweep_csv_file.close()
            except Exception:
                pass
        self._psweep_csv_file = None
        self._psweep_csv_writer = None

        result = self._psweep_analyze_trial(pwr_pct)
        self._psweep_trial_results.append(result)
        self._save_psweep_trial_snapshot(pwr_pct, result)
        self._log(
            f"Trial Power {pwr_pct}% → mean P1={result['p1_mean']:.3f} ± "
            f"{result['p1_std']:.3f}, P2={result['p2_mean']:.3f} ± "
            f"{result['p2_std']:.3f} PSI (n={result['n']})",
            "recv")

        self._psweep_trial_idx += 1
        # Brief pause between trials, then proceed
        self.root.after(500, self._psweep_begin_trial)

    def _psweep_analyze_trial(self, pwr_pct):
        """Compute mean and std of the trial's pressure traces."""
        p1 = self._psweep_trial_p1
        p2 = self._psweep_trial_p2
        if not p1:
            return {
                "power": pwr_pct,
                "p1_mean": float("nan"), "p1_std": float("nan"),
                "p2_mean": float("nan"), "p2_std": float("nan"),
                "n": 0, "t": [], "p1": [], "p2": [],
            }
        arr1 = np.asarray(p1, dtype=float)
        arr2 = np.asarray(p2, dtype=float)
        return {
            "power": pwr_pct,
            "p1_mean": float(arr1.mean()),
            "p1_std":  float(arr1.std(ddof=0)),
            "p2_mean": float(arr2.mean()),
            "p2_std":  float(arr2.std(ddof=0)),
            "n": len(p1),
            "t":  list(self._psweep_trial_t),
            "p1": list(p1),
            "p2": list(p2),
        }

    def _save_psweep_trial_snapshot(self, pwr_pct, result):
        if not result["t"]:
            return
        fig = Figure(figsize=(8, 5), dpi=150)
        fig.subplots_adjust(left=0.10, right=0.97, top=0.92, bottom=0.26)
        ax = fig.add_subplot(111)
        self._style_axes(ax)

        t = result["t"]
        # CONT mode uses the 2-s rolling mean for display — matches the
        # live plot and gives a clean picture of the time-averaged pressure
        window_n = max(1, int(ROLLING_MEAN_SECONDS * ARDUINO_REPORT_HZ))
        p1_disp = self._rolling_mean(result["p1"], window_n)
        p2_disp = self._rolling_mean(result["p2"], window_n)
        ax.plot(t, p1_disp, color=COLOR_P1, linewidth=1.4,
                label="Pressure 1", zorder=2)
        ax.plot(t, p2_disp, color=COLOR_P2, linewidth=1.4,
                label="Pressure 2", zorder=2)
        if not np.isnan(result["p1_mean"]):
            ax.axhline(result["p1_mean"], color=COLOR_P1,
                       linestyle="--", linewidth=1.0, alpha=0.7, zorder=1)
        if not np.isnan(result["p2_mean"]):
            ax.axhline(result["p2_mean"], color=COLOR_P2,
                       linestyle="--", linewidth=1.0, alpha=0.7, zorder=1)
        ax.set_xlim(t[0], t[-1])
        ax.set_ylim(Y_AXIS_MIN_PSI, Y_AXIS_MAX_PSI)
        title = (f"Pump Power {pwr_pct}% Trial  —  Mean: "
                 f"P1 = {result['p1_mean']:.3f} ± {result['p1_std']:.3f},  "
                 f"P2 = {result['p2_mean']:.3f} ± {result['p2_std']:.3f} PSI")
        ax.set_title(title, fontsize=11)
        ax.legend(loc="upper right")

        cal_part = ("Calibrated" if self.cal_done else "Not calibrated")
        footer = (
            f"Power Sweep Trial  ·  Pump power: {pwr_pct}%  ·  "
            f"Mode: Continuous (20 Hz duty cycle)\n"
            f"Trial duration: {len(t)/ARDUINO_REPORT_HZ:.1f} s  ·  "
            f"Sample rate: {ARDUINO_REPORT_HZ} Hz  ·  "
            f"Smoothing: {ROLLING_MEAN_SECONDS:.0f}-s rolling mean\n"
            f"{cal_part}  ·  Samples: {result['n']}"
        )
        fig.text(0.5, 0.02, footer, ha="center", va="bottom", fontsize=9,
                 family="serif",
                 bbox=dict(boxstyle="round,pad=0.4",
                           facecolor="#fafafa", edgecolor="#cccccc"))
        png_path = os.path.join(self._psweep_dir, f"power_{pwr_pct:03d}.png")
        try:
            fig.savefig(png_path, dpi=150, facecolor="white")
        except Exception as e:
            self._log(f"Trial snapshot save failed: {e}", "err")

    def _save_psweep_summary(self):
        if not self._psweep_trial_results:
            return None, None
        results = self._psweep_trial_results
        csv_path = os.path.join(self._psweep_dir, "summary.csv")
        try:
            with open(csv_path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow([
                    "Power(%)",
                    "P1_mean_PSI", "P1_std_PSI",
                    "P2_mean_PSI", "P2_std_PSI",
                    "n_samples",
                ])
                for r in results:
                    w.writerow([
                        r["power"],
                        f"{r['p1_mean']:.4f}", f"{r['p1_std']:.4f}",
                        f"{r['p2_mean']:.4f}", f"{r['p2_std']:.4f}",
                        r['n'],
                    ])
        except OSError as e:
            self._log(f"Summary CSV save failed: {e}", "err")
            csv_path = None

        powers = [r["power"] for r in results]
        p1_m   = [r["p1_mean"] for r in results]
        p1_s   = [r["p1_std"]  for r in results]
        p2_m   = [r["p2_mean"] for r in results]
        p2_s   = [r["p2_std"]  for r in results]

        fig = Figure(figsize=(8, 5.2), dpi=150)
        fig.subplots_adjust(left=0.11, right=0.97, top=0.92, bottom=0.22)
        ax = fig.add_subplot(111)
        self._style_axes(ax)
        ax.set_xlabel("Pump Power (%)")
        ax.set_ylabel("Mean Pressure (PSI)")

        ax.errorbar(powers, p1_m, yerr=p1_s,
                    fmt="o-", color=COLOR_P1, markersize=6,
                    markerfacecolor=COLOR_P1, markeredgecolor="black",
                    markeredgewidth=0.6, linewidth=1.4, capsize=3,
                    label="Pump 1")
        ax.errorbar(powers, p2_m, yerr=p2_s,
                    fmt="s-", color=COLOR_P2, markersize=6,
                    markerfacecolor=COLOR_P2, markeredgecolor="black",
                    markeredgewidth=0.6, linewidth=1.4, capsize=3,
                    label="Pump 2")
        ax.set_xlim(min(powers) - 5, max(powers) + 5)
        finite_y = [v for v in (p1_m + p2_m) if not np.isnan(v)]
        if finite_y:
            ymax = max(finite_y) + max(0.1, 1.2 * max(p1_s + p2_s))
            ax.set_ylim(0, ymax)
        ax.set_title("Pressure vs. Pump Power")
        ax.legend(loc="best")

        ts_started = (self._psweep_started_at.strftime("%Y-%m-%d %H:%M:%S")
                      if self._psweep_started_at else "—")
        cal_part = ("Calibrated" if self.cal_done else "Not calibrated")
        try:
            trial_s = int(self.psweep_trial_s.get())
        except (tk.TclError, ValueError):
            trial_s = int(PSWEEP_TRIAL_DURATION_S)
        footer = (
            f"Pump Power Sweep Summary  ·  Trials: {len(results)}  ·  "
            f"Power range: {min(powers)} – {max(powers)}%  ·  "
            f"Mode: CONT (continuous, 20 Hz duty cycle)\n"
            f"Per-trial duration: {trial_s} s  ·  "
            f"Sample rate: {ARDUINO_REPORT_HZ} Hz  ·  "
            f"Error bars: ±1 σ within-trial  ·  {cal_part}\n"
            f"Started: {ts_started}"
        )
        fig.text(0.5, 0.02, footer, ha="center", va="bottom", fontsize=9,
                 family="serif",
                 bbox=dict(boxstyle="round,pad=0.4",
                           facecolor="#fafafa", edgecolor="#cccccc"))
        png_path = os.path.join(self._psweep_dir, "summary.png")
        try:
            fig.savefig(png_path, dpi=150, facecolor="white")
        except Exception as e:
            self._log(f"Summary plot save failed: {e}", "err")
            png_path = None
        return csv_path, png_path

    def _psweep_finish(self):
        self._send_command("OFF")
        self.master_on.set(False)
        self._update_master_button()
        csv_path, png_path = self._save_psweep_summary()
        self._psweep_in_progress = False
        self._psweep_state = "IDLE"
        self._set_controls_during_power_sweep(locked=False)
        self.psweep_button.config(
            text="▶  Start Power Sweep",
            bg=self.SUCCESS_COLOR, activebackground="#196b3a")
        n = len(self._psweep_trial_results)
        msg = (f"Power sweep complete — {n} trial(s) saved to "
               f"{os.path.basename(self._psweep_dir)}/")
        self.psweep_status.config(text=msg, style="Success.TLabel")
        self._log(msg, "recv")
        if png_path:
            self._log(f"Summary saved: {os.path.basename(png_path)}",
                      "recv")

    def _cancel_power_sweep(self):
        if not self._psweep_in_progress:
            return
        if self._psweep_csv_file:
            try:
                self._psweep_csv_file.close()
            except Exception:
                pass
            self._psweep_csv_file = None
            self._psweep_csv_writer = None
        self._send_command("OFF")
        self.master_on.set(False)
        self._update_master_button()
        if self._psweep_trial_results:
            try:
                self._save_psweep_summary()
            except Exception:
                pass
        self._psweep_in_progress = False
        self._psweep_state = "IDLE"
        self._set_controls_during_power_sweep(locked=False)
        self.psweep_button.config(
            text="▶  Start Power Sweep",
            bg=self.SUCCESS_COLOR, activebackground="#196b3a")
        self.psweep_status.config(text="Power sweep cancelled.",
                                  style="Danger.TLabel")
        self._log("Power sweep cancelled by user", "err")

    def _set_controls_during_power_sweep(self, locked):
        state = "disabled" if locked else "normal"
        ttk_state = ["disabled"] if locked else ["!disabled"]
        self.master_button.config(state=state)
        self.record_button.config(state=state)
        self.pump1_scale.state(ttk_state)
        self.pump2_scale.state(ttk_state)
        self.pump1_entry.state(ttk_state)
        self.pump2_entry.state(ttk_state)
        self.bpm_scale.state(ttk_state)
        self.match_hr_check.state(ttk_state)
        # Keep the OTHER sweep's button locked too, plus the cal button.
        if locked:
            self.cal_button.state(["disabled"])
            self.sweep_button.config(state="disabled")
        else:
            self.cal_button.state(["!disabled"])
            self.sweep_button.config(state="normal")

    def _update_psweep_status(self):
        if not self._psweep_in_progress:
            return
        idx = self._psweep_trial_idx + 1
        n_total = len(self._psweep_pwr_list)
        pwr = self._psweep_pwr_list[self._psweep_trial_idx]
        remaining_s = max(0, self._psweep_state_until_ms
                          - int(time.time() * 1000)) / 1000.0
        if self._psweep_state == "STABILIZE":
            phase = f"Stabilizing — {remaining_s:.0f}s left"
        elif self._psweep_state == "RECORDING":
            phase = f"Recording — {remaining_s:.0f}s left"
        else:
            phase = self._psweep_state
        self.psweep_status.config(
            text=f"Trial {idx}/{n_total}  ·  Power {pwr}%  ·  {phase}",
            style="Success.TLabel")

    # === Plotting =========================================================
    def _reset_plot_buffers(self):
        self.plot_times.clear()
        self.plot_p1_raw.clear()
        self.plot_p2_raw.clear()
        self.start_time = time.time()

    def _clear_graph(self):
        self._reset_plot_buffers()
        self.ax.clear()
        self._style_axes(self.ax)
        self.fig.tight_layout()
        self.canvas.draw_idle()

    def _displayed_series(self, mode):
        """Return (p1, p2) lists with display smoothing applied.

        HEART mode uses a tiny rolling median so heartbeat peaks remain
        sharp and EKG-like. All other modes use the 2-s rolling mean for
        stable steady-state readout.
        """
        p1 = list(self.plot_p1_raw)
        p2 = list(self.plot_p2_raw)
        if not p1:
            return p1, p2
        if mode == "HEART":
            return (self._rolling_median(p1, HEART_MEDIAN_WINDOW),
                    self._rolling_median(p2, HEART_MEDIAN_WINDOW))
        window_n = max(1, int(ROLLING_MEAN_SECONDS * ARDUINO_REPORT_HZ))
        return (self._rolling_mean(p1, window_n),
                self._rolling_mean(p2, window_n))

    @staticmethod
    def _rolling_mean(data, n):
        out = [0.0] * len(data)
        cum = 0.0
        from collections import deque as _dq
        window = _dq(maxlen=n)
        for i, v in enumerate(data):
            window.append(v)
            out[i] = sum(window) / len(window)
        return out

    @staticmethod
    def _rolling_median(data, n):
        if n <= 1:
            return list(data)
        out = [0.0] * len(data)
        from collections import deque as _dq
        window = _dq(maxlen=n)
        for i, v in enumerate(data):
            window.append(v)
            sw = sorted(window)
            out[i] = sw[len(sw) // 2]
        return out

    def _update_plot(self):
        if self.plot_times:
            mode = self.last_mode_seen
            p1_disp, p2_disp = self._displayed_series(mode)

            self.ax.clear()
            self._style_axes(self.ax)

            self.ax.plot(self.plot_times, p1_disp,
                         color=COLOR_P1, linewidth=1.6,
                         label="Pressure 1")
            self.ax.plot(self.plot_times, p2_disp,
                         color=COLOR_P2, linewidth=1.6,
                         label="Pressure 2")

            # Title gets mode hint so the user sees what's being shown
            self.ax.set_title(
                f"Live Pressure  —  {self._mode_human(mode)} mode")
            self.ax.legend(loc="upper right")

            # X axis: window depends on mode
            #   HEART -> ~5 cardiac cycles, scales with BPM
            #   other -> default rolling window
            window_s = self._x_window_seconds(mode)
            t_end = self.plot_times[-1]
            self.ax.set_xlim(max(0, t_end - window_s),
                             max(window_s, t_end))

            # Y axis: fixed range so the trace stays visually anchored
            self.ax.set_ylim(Y_AXIS_MIN_PSI, Y_AXIS_MAX_PSI)

            self.fig.tight_layout()
            self.canvas.draw_idle()

        self.root.after(PLOT_UPDATE_INTERVAL_MS, self._update_plot)

    def _x_window_seconds(self, mode):
        """How many seconds of history to show on the x-axis."""
        if mode == "HEART":
            bpm = max(30, int(self.bpm_var.get()))
            # 5 cycles fits comfortably while still showing peak + flat region
            return HEART_CYCLES_VISIBLE * 60.0 / bpm
        return float(PLOT_WINDOW_SECONDS)

    # === Message pump =====================================================
    def _poll_messages(self):
        try:
            while True:
                msg_type, data = self.message_queue.get_nowait()
                if msg_type == "DATA":
                    raw_p1 = data["p1"]
                    raw_p2 = data["p2"]

                    # During the baseline-capture step, store raw values
                    # so we can compute offsets without applying them yet.
                    if self._cal_collecting:
                        self._cal_samples_p1.append(raw_p1)
                        self._cal_samples_p2.append(raw_p2)

                    # Apply calibration offset to everything user-facing
                    p1 = raw_p1 - self.cal_offset_p1
                    p2 = raw_p2 - self.cal_offset_p2

                    # Heart-rate sensor handling
                    self._handle_hr_reading(data.get("hr_measured", 0))

                    self.p1_value.config(text=f"{p1:.3f}")
                    self.p2_value.config(text=f"{p2:.3f}")
                    self.last_mode_seen = data["mode"]

                    t = time.time() - self.start_time
                    self.plot_times.append(t)
                    self.plot_p1_raw.append(p1)
                    self.plot_p2_raw.append(p2)

                    if self.is_recording.get() and self.csv_writer:
                        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                        self.csv_writer.writerow([
                            ts, round(t, 3),
                            round(p1, 4), round(p2, 4),
                            data["mode"],
                            data["pump1_duty"], data["pump2_duty"],
                            data["bpm"],
                            data.get("hr_measured", 0),
                        ])

                    # Sweep-study trial recording
                    if (self._sweep_in_progress
                            and self._sweep_state == "RECORDING"):
                        trial_t = time.time() - self._sweep_trial_started_at
                        self._sweep_trial_t.append(trial_t)
                        self._sweep_trial_p1.append(p1)
                        self._sweep_trial_p2.append(p2)
                        if self._sweep_csv_writer:
                            ts = datetime.now().strftime(
                                "%H:%M:%S.%f")[:-3]
                            self._sweep_csv_writer.writerow([
                                ts, round(trial_t, 3),
                                round(p1, 4), round(p2, 4),
                                data["mode"],
                                data["pump1_duty"], data["pump2_duty"],
                                data["bpm"],
                            ])

                    # Power-sweep trial recording
                    if (self._psweep_in_progress
                            and self._psweep_state == "RECORDING"):
                        trial_t = time.time() - self._psweep_trial_started_at
                        pwr_pct = self._psweep_pwr_list[
                            self._psweep_trial_idx]
                        self._psweep_trial_t.append(trial_t)
                        self._psweep_trial_p1.append(p1)
                        self._psweep_trial_p2.append(p2)
                        if self._psweep_csv_writer:
                            ts = datetime.now().strftime(
                                "%H:%M:%S.%f")[:-3]
                            self._psweep_csv_writer.writerow([
                                ts, round(trial_t, 3),
                                round(p1, 4), round(p2, 4),
                                data["mode"],
                                data["pump1_duty"], data["pump2_duty"],
                                pwr_pct,
                            ])

                elif msg_type == "SENT":
                    self._log(f"→ {data}", "sent")
                elif msg_type == "OK":
                    self._log(f"✓ {data}", "recv")
                elif msg_type == "ERR":
                    self._log(f"✗ {data}", "err")
                elif msg_type == "RAW":
                    self._log(f"  {data}", "recv")
                elif msg_type == "ERROR":
                    self._log(f"!! {data}", "err")
                    if not self.serial.connected:
                        self.status_label.config(text="● Disconnected",
                                                 style="Danger.TLabel")
                        self.connect_button.config(text="Connect")
                        self._set_controls_enabled(False)
        except queue.Empty:
            pass
        self.root.after(DATA_POLL_INTERVAL_MS, self._poll_messages)

    def _log(self, msg, tag="recv"):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        # The log widget is built after the connection panel, so the very
        # first port refresh might run before log_text exists. Fall back
        # to stderr in that case so diagnostic info isn't lost.
        if not hasattr(self, "log_text") or self.log_text is None:
            print(f"[{ts}] {msg}", file=sys.stderr)
            return
        self.log_text.insert("end", f"[{ts}] {msg}\n", tag)
        self.log_text.see("end")
        lines = int(self.log_text.index("end").split(".")[0])
        if lines > 500:
            self.log_text.delete("1.0", f"{lines - 500}.0")

    def _on_close(self):
        if self.is_recording.get():
            self._toggle_recording()
        if self.serial.connected:
            try:
                self.serial.send_command("OFF")
                time.sleep(0.1)
            except Exception:
                pass
            self.serial.disconnect()
        self.root.destroy()


# === Entry point ==========================================================
def main():
    root = tk.Tk()
    PumpControllerApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()