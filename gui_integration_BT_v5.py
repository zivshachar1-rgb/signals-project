"""
============================================================================
 PULSE MONITOR GUI  -  host application (Windows, Bluetooth SPP)
============================================================================
WHAT IT DOES
    Receives one packet per second from the ESP32 over Bluetooth, then shows
    the heart rate, pulse waveform, alarms and a system log in one window.
    The alarm limits set on the sliders are also sent back to the ESP32, so
    its alarm LED follows the same limits (two-way alarm).

ARCHITECTURE
    SerialReader thread --(rx queue)--> main() GUI loop
    SerialReader thread <--(tx queue)-- main() GUI loop   (alarm limits)
      - the thread does the slow, blocking Bluetooth work and reconnecting
      - the GUI loop only uses the queues and redraws, so it never freezes

PACKET FROM THE ESP32 (once per second, fixed length: 263 characters + newline)
    seq,bpm,s0,s1,...,s49,cmd
    e.g. 00123,078.9,1930,1931,...,1944,3
      seq: packet sequence number (wraps at 99999)
      bpm: pulse rate calculated on the ESP32 (0 = no valid reading)
      s0..s49: 50 raw ADC readings, 20 ms apart
      cmd: button action since the last packet
           0 none, 1 record start, 2 record stop, 3 event marker, 4 alarm silenced

MESSAGE TO THE ESP32 (alarm limits)
    LIM,low,high      e.g. LIM,055,120
      sent when a slider moves, after every (re)connect and every 5 s

REQUIREMENT MAP
    Req 6   one window ............................ build_layout(), make_window()
    Req 7   BPM and waveform graphs ............... VIEWS, refresh_plot(), toggle button
    Req 8   refresh at least once per second ...... timer-driven redraw in main()
    Req 9   large BPM text ........................ the -BPM- element (72 pt)
    Req 10  BPM to 1 decimal place ................ update_bpm()
    Req 11  visual alarm for high / low pulse ..... update_alarm(); sliders set the
                                                    limits and _send_limits() passes
                                                    them to the ESP32
    Req 12  display latency under 2 s ............. 1 s packets + 0.2 s redraw
    Req 13  system log with timestamp format ...... log_event()
    Req 14  alarm if no packet for 5 s ............ update_comm_alarm(), log_comm_transition()
    Req 15  Bluetooth SPP ......................... SerialReader
    Req 16  automatic reconnect ................... SerialReader.run()
    Req 17  connected within 10 s ................. SerialReader.run() ("Connected in X s")
    Req 18  packet check and sequence numbers ..... _parse_packet(), _check_sequence()
    Req 19  low-frequency / noise handling ........ bandpass() on the waveform view

    pip install PySimpleGUI matplotlib pyserial numpy scipy
(Install "pyserial", not "serial" - they are different packages.)
============================================================================
"""

import queue
import threading
import time
from collections import deque
from datetime import datetime

import numpy as np
import PySimpleGUI as sg
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import serial
from scipy.signal import butter, sosfiltfilt

# --------------------------------------------------------------------------
# Settings
# --------------------------------------------------------------------------
PORT_NAME = "COM7"         # Windows: the OUTGOING Bluetooth COM port of the paired ESP32AK
BAUD_RATE = 115200         # ignored by Bluetooth SPP, kept for pyserial
SERIAL_TIMEOUT_S = 2

LOW_LIMIT = 40     # alarm limits (changed live by the sliders, and sent to the ESP32)
HIGH_LIMIT = 120
LIMIT_RESEND_S = 5         # limits are re-sent this often, so a lost message or an ESP32 reboot heals itself

MIN_VALID_BPM, MAX_VALID_BPM = 20, 250   # outside this, treat as "no pulse"

REFRESH_INTERVAL_S = 0.2   # redraw target (Req 8 needs at least once per second)
MAX_ALLOWED_GAP_S = 1.0    # requirement: never more than 1 s between refreshes
STALE_AFTER_S = 2.0        # no packet for this long -> BPM shown as "--"
SELF_TEST = True           # measure the worst refresh gap at start-up
SELF_TEST_DURATION_S = 10

COMM_TIMEOUT_S = 5.0       # secs w/o a packet before the comms alarm shows (Req 14)
Themes = sg.theme_list()

# Packet format (Req 18)
SAMPLES_PER_PACKET = 50    # 1 s / 20 ms
SAMPLE_PERIOD_S = 0.020    # time between samples inside a packet
PKT_CHARS = 263            # fixed packet length, excluding the newline
CMD_TEXT = {1: "Recording started", 2: "Recording stopped",
            3: "Event marker", 4: "Alarm silenced on device"}   # cmd code -> log message

# Connection handling (Req 16, 17)
NO_DATA_TIMEOUT_S = 5      # silent failure: Bluetooth COM port can stay "open" after a drop
RETRY_DELAY_S = 1          # pause between open attempts
CONNECT_TARGET_S = 10      # comms should be established within this time

# --------------------------------------------------------------------------
# Filter settings (Req 19, display only)
# --------------------------------------------------------------------------
FILTER_ON = 1
LOW_CUT_HZ = 0.5      # removes baseline drift below this
HIGH_CUT_HZ = 4.0     # removes noise above this (240 BPM)
FILTER_ORDER = 2      # higher = steeper cutoff, but more edge distortion

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

# Rolling data shown on the graphs (old points are trimmed away)
# pulse rows: (t, bpm)    waveform rows: (t, raw, threshold)
data = {"pulse": deque(), "waveform": deque()}

# Colours and text for the pulse alarm panel (Req 11)
ALARM_STYLES = {
    "idle": {"text": "ALARM: NO DATA", "bg": "#BDBDBD", "fg": "#222222"},
    "ok":   {"text": "PULSE OK",       "bg": "#4CAF50", "fg": "#FFFFFF"},
    "low":  {"text": "PULSE TOO LOW",  "bg": "#E53935", "fg": "#FFFFFF"},
    "high": {"text": "PULSE TOO HIGH", "bg": "#E53935", "fg": "#FFFFFF"},
}

# Colours and text for the communication-loss box (Req 14)
COMM_ALARM_STYLES = {
    "ok":   {"text": "COMMUNICATION OK\nData packets arriving.",
             "bg": "#BDBDBD", "fg": "#222222"},
    "lost": {"text": "\u26A0\nWARNING\nData not received for 5 secs.\n"
                     "Communication interrupted.",
             "bg": "#FF9800", "fg": "#000000"},
}

REC_STYLES = {
    "off": {"text": "\u25CF  REC", "fg": "#9E9E9E"},   # grey, \u25CF is filled circle
    "on":  {"text": "\u25CF  REC", "fg": "#E53935"},   # red
}

# --------------------------------------------------------------------------
# Bluetooth serial reader (runs in its own thread)
# --------------------------------------------------------------------------
class SerialReader(threading.Thread):
    """
    SerialReader
    ------------------------------------------------------------------------
    Purpose : Open the Bluetooth COM port, read packets, check them, and hand
              them to the GUI. Also sends the alarm limits to the ESP32. Keeps
              retrying if the link never comes up or is lost.
    Req     : 15 (Bluetooth SPP), 16 (reconnect), 17 (connect within 10 s),
              18 (packet checking, sequence numbers), 11 (limits to the ESP32)
    Input   : tx_q holds (low, high) alarm limits from the GUI sliders
    Output  : messages on rx_q, each one of:
                ("status", text)                  connection information
                ("event", text)                   button action from the device
                ("data", t, raw, thr, bpm)        one waveform sample
    Why     : serial reads can block for seconds, so this runs in a separate
              thread. The queues are thread-safe, so the GUI never freezes and
              never has to wait for Bluetooth. Only this thread touches the
              port, so reads and writes never collide.
    """

    def __init__(self, rx_q, tx_q):
        super().__init__(daemon=True)                  # daemon: the thread stops when the program exits
        self.rx_q = rx_q                               # queue to the GUI loop
        self.tx_q = tx_q                               # queue from the GUI loop (alarm limits)
        self.stop_event = threading.Event()            # set by the GUI to stop the thread
        self.port = None                               # the open serial port, or None
        self.buffer = b""                              # bytes received but not yet split into packets
        self.last_data_time = time.monotonic()         # when the last valid packet arrived
        self.attempt_start = None                      # when the current connection attempt began
        self.warned = False                            # so the "still retrying" message appears once
        self.last_seq = None                           # last accepted sequence number
        self.limits = (LOW_LIMIT, HIGH_LIMIT)          # latest alarm limits to send to the ESP32
        self.last_limit_send = 0.0                     # when the limits were last sent

    def _open_port(self):
        """Try to open the Bluetooth COM port. Returns the port, or None on failure."""
        p = serial.Serial()
        p.port = PORT_NAME
        p.baudrate = BAUD_RATE
        p.bytesize = 8
        p.timeout = SERIAL_TIMEOUT_S
        p.write_timeout = SERIAL_TIMEOUT_S             # a write to a dropped link must not hang forever
        p.stopbits = serial.STOPBITS_ONE
        try:
            p.open()                                   # opening the outgoing port makes Windows connect to the ESP32
            return p
        except Exception:
            return None                                # device off or out of range: caller will retry

    def _close_port(self):
        """Close the port safely and reset the receive state so it can be reopened."""
        try:
            if self.port is not None:
                self.port.close()
        except Exception:
            pass
        self.port = None
        self.buffer = b""
        self.last_seq = None           # new connection -> new sequence baseline

    def _send_limits(self):
        """
        Send the alarm limits to the ESP32 as one short text line, e.g. LIM,055,120.
        Zero-padded to 3 digits so the ESP32 parser always sees the same shape.
        """
        low, high = self.limits
        self.port.write(f"LIM,{low:03d},{high:03d}\n".encode("ascii"))
        self.last_limit_send = time.monotonic()

    def _parse_packet(self, line):
        """
        Check and decode one packet line (Req 18).
        Returns (seq, bpm, samples, cmd), or None if the line is not a valid
        packet. A valid packet is exactly 263 characters with 53 numeric
        fields, so cut-off or corrupted lines are silently rejected.
        """
        text = line.decode("ascii", errors="replace").strip()
        if len(text) != PKT_CHARS:                     # fixed length check
            return None
        parts = text.split(",")
        if len(parts) != SAMPLES_PER_PACKET + 3:       # seq + bpm + 50 samples + cmd
            return None
        try:
            seq = int(parts[0])
            bpm = float(parts[1])
            samples = [int(x) for x in parts[2:2 + SAMPLES_PER_PACKET]]
            cmd = int(parts[-1])
        except ValueError:                             # a non-numeric field means a corrupted packet
            return None
        return seq, bpm, samples, cmd

    def _check_sequence(self, seq):
        """
        Check the sequence number (Req 18). Returns True if the packet should be used.
          first packet / seq 0 : accept (new connection, wrap-around, ESP32 reboot)
          seq <= last          : duplicate or out of order, so discard it
          seq > last + 1       : packets were missed, so report it but use this one
        """
        if self.last_seq is None or seq == 0:
            self.last_seq = seq        # first packet after connect, wrap-around, or ESP32 reboot
            return True
        if seq <= self.last_seq:
            self.rx_q.put(("status",
                f"Out-of-order/duplicate packet {seq} (last {self.last_seq}) - discarded"))
            return False
        if seq > self.last_seq + 1:
            self.rx_q.put(("status",
                f"{seq - self.last_seq - 1} packet(s) missed before #{seq}"))
        self.last_seq = seq
        return True

    def _publish(self, bpm, samples):
        """
        Turn one packet into 50 timestamped waveform points for the GUI.
        The packet arrives all at once, so the original 20 ms spacing is
        rebuilt by working backwards from the arrival time. The threshold line
        is only for display and is computed here on the host (60% of the
        packet's range); the ESP32 uses its own threshold for the BPM.
        """
        t_end = time.monotonic()
        lo, hi = min(samples), max(samples)
        thr = lo + 0.6 * (hi - lo)     # display threshold line, computed on the host
        n = len(samples)
        for i, v in enumerate(samples):
            t = t_end - (n - 1 - i) * SAMPLE_PERIOD_S    # spread over the last second
            self.rx_q.put(("data", t, v, thr, bpm))

    def run(self):
        """
        Thread main loop: connect -> send limits + read packets -> detect
        failure -> reconnect. The loop never exits until the program closes,
        which is how the host "continuously tries to reconnect" (Req 16).
        """
        while not self.stop_event.is_set():

            # no open port -> keep trying forever (Req 16, 17)
            if self.port is None or not self.port.is_open:
                if self.attempt_start is None:
                    self.attempt_start = time.monotonic()      # start timing this connection attempt
                    self.rx_q.put(("status", "Connecting..."))

                self.port = self._open_port()
                if self.port is None:                          # failed: wait, then try again
                    waited = time.monotonic() - self.attempt_start
                    if waited > CONNECT_TARGET_S and not self.warned:
                        self.warned = True                     # warn once, not every second
                        self.rx_q.put(("status",
                            f"Not connected after {CONNECT_TARGET_S} s - still retrying"))
                    self.stop_event.wait(RETRY_DELAY_S)
                    continue

                self.last_data_time = time.monotonic()         # connected: reset all receive state
                self.buffer = b""
                self.last_seq = None
                self.last_limit_send = 0.0                     # send the limits straight away on a new connection
                self.warned = False
                elapsed = time.monotonic() - self.attempt_start
                self.attempt_start = None
                self.rx_q.put(("status", f"Connected in {elapsed:.1f} s"))   # evidence for Req 17

            # send limits, then read bytes and split into packet lines
            try:
                # Two-way alarm: pick up the newest limits from the GUI, and send them
                # when they change, after connecting, and every LIMIT_RESEND_S seconds.
                while not self.tx_q.empty():
                    self.limits = self.tx_q.get_nowait()
                    self.last_limit_send = 0.0                 # force a send now
                if time.monotonic() - self.last_limit_send >= LIMIT_RESEND_S:
                    self._send_limits()

                n = self.port.in_waiting                       # bytes waiting to be read
                if n > 0:
                    self.buffer += self.port.read(n)
                    *lines, self.buffer = self.buffer.split(b"\n")   # complete lines; the last piece is a partial packet kept for next time
                    for line in lines:
                        pkt = self._parse_packet(line)
                        if pkt is None:                        # invalid packet: drop it
                            continue
                        seq, bpm, samples, cmd = pkt
                        if not self._check_sequence(seq):      # duplicate / out of order: drop it
                            continue
                        self.last_data_time = time.monotonic() # valid packet: the link is alive
                        self._publish(bpm, samples)
                        if cmd in CMD_TEXT:                    # button action on the device
                            self.rx_q.put(("event", CMD_TEXT[cmd]))
                else:
                    # silent failure (Bluetooth dropped, port still "open")
                    if time.monotonic() - self.last_data_time > NO_DATA_TIMEOUT_S:
                        self.rx_q.put(("status",
                            f"No data for {NO_DATA_TIMEOUT_S} s - reconnecting"))
                        self._close_port()                     # close it so the loop above reopens it
                        self.stop_event.wait(RETRY_DELAY_S)
                    else:
                        self.stop_event.wait(0.01)             # short sleep so we do not use 100% CPU

            # hard failure (port vanished, or a write timed out)
            except (serial.SerialException, OSError):
                self.rx_q.put(("status", "Connection lost - reconnecting"))
                self._close_port()
                self.stop_event.wait(RETRY_DELAY_S)

        self._close_port()                                     # tidy up when the program exits

# --------------------------------------------------------------------------
# Display helpers
# --------------------------------------------------------------------------
def trim_buffers(now_t):
    """
    Purpose : Delete points older than each graph's time window.
    Why     : keeps memory use and plotting time constant however long it runs.
    """
    for key, buf in data.items():
        cutoff = now_t - VIEWS[key]["window_s"]
        while buf and buf[0][0] < cutoff:
            buf.popleft()


def filter_button_text():
    """Text for the filter on/off button."""
    return "Bandpass Filter: ON" if FILTER_ON else "Bandpass Filter: OFF"


def bandpass(values, fs):
    """
    Purpose : Band-pass filter the waveform between 0.5 and 4 Hz.
    Req     : 19 (deals with low-frequency drift) and noise reduction
    Input   : values = samples, fs = sample rate in Hz
    Output  : filtered samples
    Why     : 0.5 Hz removes slow baseline drift (breathing, movement); 4 Hz
              (240 BPM) removes high-frequency noise. sosfiltfilt filters
              forwards and backwards, so the pulse is not shifted in time.
              This only changes the plot; the BPM comes from the ESP32.
    """
    nyq = fs / 2                                       # Nyquist frequency
    if HIGH_CUT_HZ >= nyq:               # sample rate too low for this band
        return values
    sos = butter(FILTER_ORDER, [LOW_CUT_HZ / nyq, HIGH_CUT_HZ / nyq],
                 btype="band", output="sos")           # Butterworth, second-order sections (numerically stable)
    return sosfiltfilt(sos, values)


def refresh_plot(ax, canvas, view_key, now_t):
    """
    Purpose : Redraw the selected graph (pulse rate or waveform).
    Req     : 7 (BPM and waveform graphs), 19 (optional filtering)
    Input   : view_key = "pulse" or "waveform", now_t = current time (s)
    Why     : x is shown as "seconds before now", so the graph scrolls left.
              The waveform view band-pass filters the data when the filter is
              on; the threshold line is only shown when the filter is off,
              because it belongs to the unfiltered signal.
    """
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
        xs = [r[0] - now_t for r in rows]              # time relative to now (negative = in the past)
        if view_key == "pulse":
            ax.plot(xs, [r[1] for r in rows], color=view["color"])
        else:
            raws = [r[1] for r in rows]
            thrs = [r[2] for r in rows]
            filtered = False
            if FILTER_ON and len(rows) > 50:
                span = rows[-1][0] - rows[0][0]
                if span > 0:
                    fs = (len(rows) - 1) / span          # samples per second
                    raws = bandpass(np.array(raws, dtype=float), fs)
                    filtered = True
            ax.plot(xs, raws, color=view["color"], label="Signal")
            if filtered:
                ax.set_ylabel("Filtered PPG (a.u.)")
                lo, hi = min(raws), max(raws)
            else:
                ax.plot(xs, thrs, "--", color="grey", linewidth=1, label="Threshold")
                lo, hi = min(min(raws), min(thrs)), max(max(raws), max(thrs))
            pad = max(0.1 * (hi - lo), 1)                # 10% margin so the trace never touches the edge
            ax.set_ylim(lo - pad, hi + pad)
            ax.legend(loc="upper right")
    else:
        ax.text(0.5, 0.5, "No data yet", transform=ax.transAxes,
                ha="center", va="center", color="grey", fontsize=14)
    canvas.draw_idle()


def update_bpm(window, bpm):
    """Show the current BPM as large text with exactly 1 decimal place (Req 9, 10)."""
    window["-BPM-"].update("--" if bpm is None else f"{bpm:.1f}")


def get_alarm_state(bpm):
    """
    Purpose : Classify a BPM reading against the limits.
    Output  : "idle" (no data), "low", "high" or "ok"
    Why     : returning a state (not a colour) lets the same result drive the
              alarm panel and the log.
    """
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
    """Colour and label the visual pulse alarm panel (Req 11)."""
    s = ALARM_STYLES[get_alarm_state(bpm)]
    window["-ALARM-"].update(s["text"], background_color=s["bg"], text_color=s["fg"])


def update_comm_alarm(window, comm_lost):
    """Light up (or clear) the communication-loss warning box (Req 14)."""
    s = COMM_ALARM_STYLES["lost" if comm_lost else "ok"]
    window["-COMM-ALARM-"].update(s["text"], background_color=s["bg"], text_color=s["fg"])

# Display function updating the colour on screen
def update_rec(window, recording):
    s = REC_STYLES["on" if recording else "off"]
    window["-REC-"].update(s["text"], text_color=s["fg"])

def refresh_display(window, ax, canvas, view_key, bpm, now_t):
    """Update everything the user sees: BPM number, alarm panel and graph."""
    update_bpm(window, bpm)
    update_alarm(window, bpm)
    refresh_plot(ax, canvas, view_key, now_t)


def log_event(window, message):
    """
    Purpose : Add one line to the system log.
    Req     : 13 (log format "<time date stamp>: <message string>")
    Example : Thu Sep 19 17:46:50 2024: Pulse Low
    """
    # Format as Day Mon DD HH:MM:SS YYYY using a 24-hour clock
    timestamp = datetime.now().strftime("%a %b %d %H:%M:%S %Y")

    # Combine the timestamp and event message
    log_message = f"{timestamp}: {message}"

    # Append the event to the system log without overwriting old entries
    window["-SYSTEM-LOG-"].update(
        value=log_message + "\n",
        append=True
    )


def log_alarm_transition(window, previous_state, current_state):
    """
    Purpose : Log pulse alarm changes, once per change.
    Req     : 13 (events appear in the log)
    Why     : comparing the previous and current state means a long alarm
              produces one entry, not one entry every 0.2 s refresh.
    """
    # Log entry when the pulse first enters the low state
    if current_state == "low" and previous_state != "low":
        log_event(window, "Pulse Low")

    # Log entry when the pulse first enters the high state
    elif current_state == "high" and previous_state != "high":
        log_event(window, "Pulse High")

    # Log recovery when a low or high pulse returns to normal
    elif current_state == "ok" and previous_state in ("low", "high"):
        log_event(window, "Pulse Normal")

    # Log when pulse data becomes unavailable after an alarm
    elif current_state == "idle" and previous_state in ("low", "high"):
        log_event(window, "Pulse Data Unavailable")


def log_comm_transition(window, previous_lost, current_lost):
    """
    Purpose : Log when communication is lost and when it returns.
    Req     : 14 (alarm when no packet for 5 s) and 13 (log events)
    Why     : logs only on a change, so there is one line per outage.
    """
    # Log once when the 5 s timeout is first exceeded
    if current_lost and not previous_lost:
        log_event(window, "Data Not Received for 5 Seconds - Communication May Be Interrupted")
    # Log once when a packet arrives again and the alarm clears
    elif previous_lost and not current_lost:
        log_event(window, "Data Received - Communication Restored")


def build_layout():
    """
    Purpose : Define every element of the single window.
    Req     : 6 (all information in one window), 7, 9, 11, 13, 14
    Layout  : title + theme, view and filter buttons, graph beside the alarm
              panel / limit sliders / big BPM, then the system log beside the
              communication warning box.
    """
    idle = ALARM_STYLES["idle"]
    bpm_panel = [
        [sg.Text(idle["text"], key="-ALARM-", size=(18, 2), justification="center",
                 font=("Any", 16, "bold"), background_color=idle["bg"],
                 text_color=idle["fg"], pad=(0, (0, 10)))],
        # User-adjustable alarm limits (Req 11): the GUI alarm and the ESP32 LED both use these values
        [sg.Text("Low limit (BPM)")],
        [sg.Slider(range=(MIN_VALID_BPM, MAX_VALID_BPM - 1), default_value=LOW_LIMIT,
                   resolution=1, orientation="h", size=(20, 12),
                   key="-LOW-LIMIT-", enable_events=True)],
        [sg.Text("High limit (BPM)")],
        [sg.Slider(range=(MIN_VALID_BPM + 1, MAX_VALID_BPM), default_value=HIGH_LIMIT,
                   resolution=1, orientation="h", size=(20, 12),
                   key="-HIGH-LIMIT-", enable_events=True, pad=(0, (0, 20)))],
        [sg.Text("Current pulse", font=("Any", 14))],
        [sg.Text("--", key="-BPM-", font=("Any", 72, "bold"), size=(6, 1),
                 justification="center")],      # large text so it stands out at a glance (Req 9)
        [sg.Text("BPM", font=("Any", 18))],
    ]

    rec = REC_STYLES["off"]
    plot_col = [
        [sg.Button(VIEWS["pulse"]["button_text"], key="-TOGGLE-", size=(42, 1)),
         sg.Button(filter_button_text(), key="-FILTER-", size=(18, 1)),
         sg.Push(),
         sg.Text(rec["text"], key="-REC-", size=(8, 1), justification="center",
                 font=("Any", 11, "bold"), text_color=rec["fg"],
                 relief=sg.RELIEF_RIDGE, border_width=2, pad=(0, 5))],
        [sg.Canvas(key="-CANVAS-")],
    ]

    # Communication-loss warning box (sits to the right of the system log)
    comm_ok = COMM_ALARM_STYLES["ok"]
    comm_panel = [
        [sg.Text(comm_ok["text"], key="-COMM-ALARM-", size=(32, 6),
                 justification="center", font=("Any", 12, "bold"),
                 background_color=comm_ok["bg"], text_color=comm_ok["fg"])],
    ]

    return [
        [sg.Text("Pulse Monitor", font=("Any", 20, "bold")),
         sg.Push(),
         sg.Text("Theme:"),
         sg.Combo(Themes, default_value=sg.theme(), key="-THEME-",
                  enable_events=True, readonly=True, size=(24, 1))],
        [sg.Column(plot_col, vertical_alignment="top", pad=0),
         sg.Column(bpm_panel, element_justification="center",
                   vertical_alignment="center", pad=(20, 0))],

        # System log heading
        [sg.Text("System Log", font=("Any", 12, "bold"))],
        # Read-only system log window at the bottom of the GUI
        [sg.Multiline(
            "",
            key="-SYSTEM-LOG-",
            size=(98, 12),
            disabled=True,          # so logs are read only
            autoscroll=True,        # keeps new entries visible
            expand_y=False
        ),
         sg.Column(comm_panel, element_justification="center",
                   vertical_alignment="top", pad=(20, 0))]
    ]

# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------
def make_window(log_text=""):
    """
    Purpose : Build the window, the matplotlib figure and the canvas using the
              current theme.
    Why     : a separate function, because changing the theme means rebuilding
              the whole window. log_text restores the old system log afterwards.
    """
    window = sg.Window("Pulse Monitor", build_layout(),
                       size=(1100, 800), resizable=True, finalize=True)
    window["-COMM-ALARM-"].Widget.config(anchor="center")
    fig, ax = plt.subplots(figsize=(7, 4))
    fig.tight_layout()
    canvas = FigureCanvasTkAgg(fig, master=window["-CANVAS-"].TKCanvas)   # embed matplotlib in the window
    canvas.get_tk_widget().pack(side="top", fill="both", expand=1)
    if log_text:
        window["-SYSTEM-LOG-"].update(log_text)    # restore the old log
    return window, fig, ax, canvas


def main():
    """
    Purpose : The GUI's main loop.
    Req     : 8 (refresh at least once per second), 12 (latency under 2 s),
              14 (5 s communication alarm), 13 (log), 11 (alarm limits)
    Steps each pass (about every 0.1 s):
      1. Handle user events (theme, filter, view, alarm limit sliders)
      2. Drain the queue filled by the SerialReader thread
      3. If 0.2 s has passed, redraw the display and check the alarms
    Why     : redrawing is driven by a timer, not by incoming data, so the
              display still updates (and the 5 s alarm still triggers) if the
              data stops. Latency is about 1 s of packet batching plus at most
              0.2 s until the next redraw, so under the 2 s limit.
    """
    global FILTER_ON, LOW_LIMIT, HIGH_LIMIT
    window, fig, ax, canvas = make_window()

    current_view = "pulse"                 # which graph is showing
    rx_q = queue.Queue()                   # reader thread -> GUI (packets, status, events)
    tx_q = queue.Queue()                   # GUI -> reader thread (alarm limits)
    reader = SerialReader(rx_q, tx_q)
    reader.start()                         # begin connecting in the background

    latest_bpm = None                      # most recent valid BPM (None = no pulse)
    last_data_time = None                  # time of the newest sample
    conn_status = "Connecting..."          # last status message (so repeats are not logged)

    # Store the previous alarm state to detect state changes
    previous_alarm_state = "idle"

    # Record that the GUI has started
    log_event(window, "Pulse Monitor Started")

    previous_comm_lost = False             # previous state of the 5 s communication alarm
    recording = False

    start = last_refresh = time.monotonic()
    worst_gap, missed = 0.0, 0             # refresh-timing self-test results
    self_test_pending = SELF_TEST

    while True:
        # ---- 1. user events ----
        event, values = window.read(timeout=100)     # wait up to 0.1 s for a click, then carry on
        if event == sg.WIN_CLOSED:
            break

        # ---- theme change: rebuild the window, keep everything else ----
        if event == "-THEME-":
            new_theme = values["-THEME-"]
            if new_theme in Themes and new_theme != sg.theme():
                log_text = window["-SYSTEM-LOG-"].get().rstrip("\n") + "\n"   # save the log
                window.close()
                plt.close(fig)
                sg.theme(new_theme)
                window, fig, ax, canvas = make_window(log_text)
                window["-TOGGLE-"].update(VIEWS[current_view]["button_text"])
                update_rec(window, recording)         # window was rebuilt, restore REC state
                log_event(window, f"Theme changed to {new_theme}")
                last_refresh = 0          # redraw straight away
                continue

        # Toggle bandpass filter on and off
        if event == "-FILTER-":
            FILTER_ON = 1 - FILTER_ON          # flips 1 -> 0 -> 1
            window["-FILTER-"].update(filter_button_text())
            last_refresh = 0                   # redraw immediately

        # User moved an alarm-limit slider (Req 11)
        if event in ("-LOW-LIMIT-", "-HIGH-LIMIT-"):
            LOW_LIMIT = int(values["-LOW-LIMIT-"])
            HIGH_LIMIT = int(values["-HIGH-LIMIT-"])
            if LOW_LIMIT >= HIGH_LIMIT:           # keep low below high
                if event == "-LOW-LIMIT-":
                    LOW_LIMIT = HIGH_LIMIT - 1
                    window["-LOW-LIMIT-"].update(LOW_LIMIT)
                else:
                    HIGH_LIMIT = LOW_LIMIT + 1
                    window["-HIGH-LIMIT-"].update(HIGH_LIMIT)
            tx_q.put((LOW_LIMIT, HIGH_LIMIT))     # two-way alarm: pass the new limits to the ESP32
            last_refresh = 0                      # re-check the alarm straight away

        # Switch between the pulse-rate graph and the waveform graph
        if event == "-TOGGLE-":
            current_view = "waveform" if current_view == "pulse" else "pulse"
            window["-TOGGLE-"].update(VIEWS[current_view]["button_text"])
            fig.tight_layout()
            last_refresh = 0          # redraw immediately

        # ---- 2. drain everything the serial thread has received ----
        while True:
            try:
                item = rx_q.get_nowait()                 # never wait: take what is there and move on
            except queue.Empty:
                break
            if item[0] == "status":
                if item[1] != conn_status:            # connection messages go to the system log
                    log_event(window, item[1])
                conn_status = item[1]
                continue
            if item[0] == "event":                    # button actions from the ESP32 (cmd field)
                log_event(window, item[1])
                if item[1] == CMD_TEXT[1]:            # record start
                    recording = True
                    update_rec(window, True)
                elif item[1] == CMD_TEXT[2]:          # record stop
                    recording = False
                    update_rec(window, False)
                continue
            _, t, raw, thr, bpm = item                # otherwise it is a waveform sample
            last_data_time = t
            data["waveform"].append((t, raw, thr))
            if MIN_VALID_BPM <= bpm <= MAX_VALID_BPM:
                latest_bpm = bpm
                data["pulse"].append((t, bpm))
            else:
                latest_bpm = None     # ESP32 reports 0 when no finger/pulse

        # ---- 3. timer-driven redraw: happens whether or not data arrived ----
        now = time.monotonic()
        if now - last_refresh >= REFRESH_INTERVAL_S:
            gap = now - last_refresh if last_refresh else REFRESH_INTERVAL_S   # time since the last redraw
            stale = last_data_time is None or (now - last_data_time) > STALE_AFTER_S
            trim_buffers(now)

            # Use the current valid BPM, or None when the data is stale
            current_bpm = None if stale else latest_bpm

            # Determine the current alarm state
            current_alarm_state = get_alarm_state(current_bpm)

            # Log any transition into or out of a pulse alarm state
            log_alarm_transition(window, previous_alarm_state, current_alarm_state)
            previous_alarm_state = current_alarm_state

            # 5 s since the last packet? (Before the first packet, count from program start.)
            reference_time = last_data_time if last_data_time is not None else start
            comm_lost = (now - reference_time) > COMM_TIMEOUT_S

            # REC STATUS: ESP32 clears its recording flag when the link drops, so do the same here
            if comm_lost and recording:
                recording = False
                update_rec(window, False)

            # Light up / clear the warning box
            update_comm_alarm(window, comm_lost)

            # Write to the system log only when the state changes
            log_comm_transition(window, previous_comm_lost, comm_lost)
            previous_comm_lost = comm_lost

            # BPM, alarm and graph updates
            refresh_display(window, ax, canvas, current_view, current_bpm, now)
            last_refresh = now

            worst_gap = max(worst_gap, gap)           # track the slowest redraw (for the self-test)
            if gap > MAX_ALLOWED_GAP_S:
                missed += 1

        # One-off self-test: after 10 s, report whether any refresh took longer than 1 s (Req 8)
        if self_test_pending and now - start >= SELF_TEST_DURATION_S:
            verdict = "PASS" if worst_gap <= MAX_ALLOWED_GAP_S else "FAIL"
            print(f"SELF-TEST {verdict}: worst gap {worst_gap:.3f} s over "
                  f"{SELF_TEST_DURATION_S} s (limit {MAX_ALLOWED_GAP_S} s)")
            self_test_pending = False

    reader.stop_event.set()                           # ask the reader thread to stop
    window.close()


if __name__ == "__main__":
    main()
