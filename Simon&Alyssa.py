"""
Pulse monitor GUI fed by the ESP32 over USB serial.

Expected line from the ESP32:
    Raw:1930 Threshold:1964 HeartRate:78.9

    pip install PySimpleGUI matplotlib pyserial
(Install "pyserial", not "serial" - they are different packages.)
"""

import queue
import re
import threading
import time
from collections import deque

import PySimpleGUI as sg
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import serial
import serial.tools.list_ports
import time
from datetime import datetime

# --------------------------------------------------------------------------
# Settings
# --------------------------------------------------------------------------
PORT = "/dev/cu.usbserial-110"          # e.g. "/dev/cu.usbserial-0001"; None = auto-detect
BAUD = 115200        # must match Serial.begin(...) in the ESP32 sketch

LOW_LIMIT = 40     # alarm limits, unset for now (e.g. 50 and 120)
HIGH_LIMIT = 120

MIN_VALID_BPM, MAX_VALID_BPM = 20, 250   # outside this, treat as "no pulse"

REFRESH_INTERVAL_S = 0.2   # redraw target
MAX_ALLOWED_GAP_S = 1.0    # requirement: never more than 1 s between refreshes
STALE_AFTER_S = 2.0        # no serial data for this long -> signal lost
SELF_TEST = True
SELF_TEST_DURATION_S = 10

LINE_RE = re.compile(
    r"Raw:\s*(-?\d+)\s+Threshold:\s*(-?\d+)\s+HeartRate:\s*(-?\d+(?:\.\d+)?)"
)

# --------------------------------------------------------------------------
# Views, buffers, alarm styles
# --------------------------------------------------------------------------
VIEWS = {
    "pulse": {
        "button_text": "Showing: Pulse (BPM)  -  click for Waveform",
        "title": "Pulse Rate", "xlabel": "Time (s, 0 = now)",
        "ylabel": "Heart rate (beats per minute)",
        "window_s": 60, "ylim": (40, 140), "color": "#d6336c",
    },
    "waveform": {
        "button_text": "Showing: Waveform (PPG)  -  click for Pulse",
        "title": "Pulse Waveform (PPG)", "xlabel": "Time (s, 0 = now)",
        "ylabel": "ADC reading (0-4095)",
        "window_s": 10, "ylim": (0, 4095), "color": "#1f6f8b",
    },
}

# pulse rows: (t, bpm)    waveform rows: (t, raw, threshold)
data = {"pulse": deque(), "waveform": deque()}

ALARM_STYLES = {
    "idle": {"text": "ALARM: NO DATA", "bg": "#BDBDBD", "fg": "#222222"},
    "ok":   {"text": "PULSE OK",       "bg": "#4CAF50", "fg": "#FFFFFF"},
    "low":  {"text": "PULSE TOO LOW",  "bg": "#E53935", "fg": "#FFFFFF"},
    "high": {"text": "PULSE TOO HIGH", "bg": "#E53935", "fg": "#FFFFFF"},
}

# --------------------------------------------------------------------------
# Serial reader (runs in its own thread)
# --------------------------------------------------------------------------
def find_port():
    if PORT:
        return PORT
    for p in serial.tools.list_ports.comports():
        if p.device.startswith("/dev/cu.") and any(
                k in p.device for k in ("usbserial", "usbmodem", "SLAB", "wchusbserial")):
            return p.device
    return None


class SerialReader(threading.Thread):
    """Reads lines, parses them, and puts the results on a queue."""

    def __init__(self, out_q):
        super().__init__(daemon=True)
        self.out_q = out_q
        self.stop_event = threading.Event()

    def run(self):
        while not self.stop_event.is_set():
            port = find_port()
            if port is None:
                self.out_q.put(("status", "No ESP32 serial port found"))
                self.stop_event.wait(1.0)
                continue
            try:
                with serial.Serial(port, BAUD, timeout=0.5) as ser:
                    self.out_q.put(("status", f"Connected: {port}"))
                    ser.reset_input_buffer()
                    while not self.stop_event.is_set():
                        line = ser.readline().decode("utf-8", errors="ignore").strip()
                        if not line:
                            continue
                        m = LINE_RE.search(line)
                        if m:   # ignore boot messages and half-received lines
                            self.out_q.put(("data", time.monotonic(),
                                            int(m[1]), int(m[2]), float(m[3])))
            except (serial.SerialException, OSError) as e:
                self.out_q.put(("status", f"Serial error: {e}"))
                self.stop_event.wait(1.0)

# --------------------------------------------------------------------------
# Display helpers
# --------------------------------------------------------------------------
def trim_buffers(now_t):
    for key, buf in data.items():
        cutoff = now_t - VIEWS[key]["window_s"]
        while buf and buf[0][0] < cutoff:
            buf.popleft()


def refresh_plot(ax, canvas, view_key, now_t):
    view = VIEWS[view_key]
    ax.clear()
    ax.set_title(view["title"])
    ax.set_xlabel(view["xlabel"])
    ax.set_ylabel(view["ylabel"])
    ax.set_xlim(-view["window_s"], 0)
    ax.set_ylim(*view["ylim"])
    ax.grid(True, linestyle="--", alpha=0.4)

    rows = list(data[view_key])
    if rows:
        xs = [r[0] - now_t for r in rows]
        if view_key == "pulse":
            ax.plot(xs, [r[1] for r in rows], color=view["color"])
        else:
            raws = [r[1] for r in rows]
            thrs = [r[2] for r in rows]
            ax.plot(xs, raws, color=view["color"], label="Signal")
            ax.plot(xs, thrs, "--", color="grey", linewidth=1, label="Threshold")
            lo, hi = min(min(raws), min(thrs)), max(max(raws), max(thrs))
            pad = max(20, 0.1 * (hi - lo))
            ax.set_ylim(lo - pad, hi + pad)
            ax.legend(loc="upper right")
    else:
        ax.text(0.5, 0.5, "No data yet", transform=ax.transAxes,
                ha="center", va="center", color="grey", fontsize=14)
    canvas.draw_idle()


def update_bpm(window, bpm):
    window["-BPM-"].update("--" if bpm is None else f"{bpm:.1f}")


def get_alarm_state(bpm):
    if bpm is None:
        return "idle"
    if LOW_LIMIT is not None and bpm < LOW_LIMIT:
        return "low"
    if HIGH_LIMIT is not None and bpm > HIGH_LIMIT:
        return "high"
    if LOW_LIMIT is None and HIGH_LIMIT is None:
        return "idle"
    return "ok"


def update_alarm(window, bpm):
    s = ALARM_STYLES[get_alarm_state(bpm)]
    window["-ALARM-"].update(s["text"], background_color=s["bg"], text_color=s["fg"])


def refresh_display(window, ax, canvas, view_key, bpm, now_t):
    update_bpm(window, bpm)
    update_alarm(window, bpm)
    refresh_plot(ax, canvas, view_key, now_t)


def log_event(window, message):
    #Format as Day Mon DD HH:MM:SS YYYY using a 24-hour clock
    timestamp = datetime.now().strftime("%a %b %d %H:%M:%S %Y")

    #Combine the timestamp and event message
    log_message = f"{timestamp}: {message}"

    #Append the event to the system log without overwriting old entries
    window["-SYSTEM-LOG-"].update(
        value=log_message + "\n",
        append=True
    )

#NEW: Log changes in the pulse alarm state
def log_alarm_transition(window, previous_state, current_state):

    #NEW: Log entry when the pulse first enters the low state
    if current_state == "low" and previous_state != "low":
        log_event(window, "Pulse Low")

    #NEW: Log entry when the pulse first enters the high state
    elif current_state == "high" and previous_state != "high":
        log_event(window, "Pulse High")

    #NEW: Log recovery when a low or high pulse returns to normal
    elif current_state == "ok" and previous_state in ("low", "high"):
        log_event(window, "Pulse Normal")

    #NEW: Log when pulse data becomes unavailable after an alarm
    elif current_state == "idle" and previous_state in ("low", "high"):
        log_event(window, "Pulse Data Unavailable")
# END NEW


def build_layout():
    idle = ALARM_STYLES["idle"]
    bpm_panel = [
        [sg.Text(idle["text"], key="-ALARM-", size=(18, 2), justification="center",
                 font=("Any", 16, "bold"), background_color=idle["bg"],
                 text_color=idle["fg"], pad=(0, (0, 20)))],
        [sg.Text("Current pulse", font=("Any", 14))],
        [sg.Text("--", key="-BPM-", font=("Any", 72, "bold"), size=(6, 1),
                 justification="center")],
        [sg.Text("BPM", font=("Any", 18))],
    ]
    return [
        [sg.Text("Pulse Monitor", font=("Any", 20, "bold"))],
        [sg.Button(VIEWS["pulse"]["button_text"], key="-TOGGLE-", size=(42, 1))],
        [sg.Canvas(key="-CANVAS-"),
         sg.Column(bpm_panel, element_justification="center",
                   vertical_alignment="center", pad=(20, 0))],
        [sg.Text("Starting...", key="-STATUS-", font=("Any", 9), text_color="grey")],
        
        #NEW: System log heading
        [sg.Text("System Log", font=("Any", 12, "bold"))],
        #NEW: Read-only system log window at the bottom of the GUI
        [sg.Multiline(
            "",
            key="-SYSTEM-LOG-",
            size=(100, 12),
            disabled=True,          # so logs are read only
            autoscroll=True,        # keeps new entries visible
            expand_x=True,          # logs use available width
            expand_y=False
        )]
    ]

# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------
def main():
    window = sg.Window("Pulse Monitor", build_layout(), size = (1100,800), resizable = True, finalize=True)

    fig, ax = plt.subplots(figsize=(7, 4))
    fig.tight_layout()
    canvas = FigureCanvasTkAgg(fig, master=window["-CANVAS-"].TKCanvas)
    canvas.get_tk_widget().pack(side="top", fill="both", expand=1)

    current_view = "pulse"
    rx_q = queue.Queue()
    reader = SerialReader(rx_q)
    reader.start()

    latest_bpm = None
    last_data_time = None
    conn_status = "Connecting..."

    #NEW: Store the previous alarm state to detect state changes
    previous_alarm_state = "idle"

    #NEW: Record that the GUI has started
    log_event(window, "Pulse Monitor Started")
    #end NEW

    start = last_refresh = time.monotonic()
    worst_gap, missed = 0.0, 0
    self_test_pending = SELF_TEST

    while True:
        event, _ = window.read(timeout=100)
        if event == sg.WIN_CLOSED:
            break

        if event == "-TOGGLE-":
            current_view = "waveform" if current_view == "pulse" else "pulse"
            window["-TOGGLE-"].update(VIEWS[current_view]["button_text"])
            fig.tight_layout()
            last_refresh = 0          # redraw immediately

        # ---- drain everything the serial thread has received ----
        while True:
            try:
                item = rx_q.get_nowait()
            except queue.Empty:
                break
            if item[0] == "status":
                conn_status = item[1]
                continue
            _, t, raw, thr, bpm = item
            last_data_time = t
            data["waveform"].append((t, raw, thr))
            if MIN_VALID_BPM <= bpm <= MAX_VALID_BPM:
                latest_bpm = bpm
                data["pulse"].append((t, bpm))
            else:
                latest_bpm = None     # ESP32 reports 0 etc. when no finger/pulse

        # ---- timer-driven redraw: happens whether or not data arrived ----
        now = time.monotonic()
        if now - last_refresh >= REFRESH_INTERVAL_S:
            gap = now - last_refresh if last_refresh else REFRESH_INTERVAL_S
            stale = last_data_time is None or (now - last_data_time) > STALE_AFTER_S
            trim_buffers(now)

            #NEW: Use the current valid BPM, or None when the data is stale
            current_bpm = None if stale else latest_bpm

            #NEW: Determine the current alarm state using Simon's existing function
            current_alarm_state = get_alarm_state(current_bpm)

            #NEW: Log any transition into or out of a pulse alarm state
            log_alarm_transition(
                window,
                previous_alarm_state,
                current_alarm_state
            )
            #NEW: Save the current state for comparison at the next refresh
            previous_alarm_state = current_alarm_state

            #NEW: Preserve the existing BPM, alarm and graph updates
            refresh_display(window, ax, canvas, current_view,current_bpm,now) #end of new
            last_refresh = now

            worst_gap = max(worst_gap, gap)
            if gap > MAX_ALLOWED_GAP_S:
                missed += 1
            window["-STATUS-"].update(
                f"{conn_status}  |  refresh gap {gap:.2f} s, worst {worst_gap:.2f} s, "
                f"missed {missed}",
                text_color="red" if missed else "grey")

        if self_test_pending and now - start >= SELF_TEST_DURATION_S:
            verdict = "PASS" if worst_gap <= MAX_ALLOWED_GAP_S else "FAIL"
            print(f"SELF-TEST {verdict}: worst gap {worst_gap:.3f} s over "
                  f"{SELF_TEST_DURATION_S} s (limit {MAX_ALLOWED_GAP_S} s)")
            self_test_pending = False

    reader.stop_event.set()
    window.close()


if __name__ == "__main__":
    main()
