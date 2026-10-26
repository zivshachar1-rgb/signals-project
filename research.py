"""
research.py - additional features for the Pulse Monitor GUI
  Researcher tab : draw_research() redraws 4 live panels from the last 60 s
  Summary report : Recorder writes raw samples to a CSV while recording is on;
                   make_report() processes that CSV offline into a PDF on exit
Uses only the raw ADC samples from the packets (ESP32 code unchanged).
"""
import csv
import os
import textwrap
from datetime import datetime

import numpy as np
from scipy.signal import butter, sosfiltfilt, find_peaks, spectrogram, welch
from matplotlib.figure import Figure
from matplotlib.backends.backend_pdf import PdfPages

FS = 50.0                                     # sample rate (20 ms)
BEAT_WIN = (int(0.3 * FS), int(0.5 * FS))     # samples before / after a peak = one beat


# ---------------------------------------------------------------- signal processing
def bandpass(x, lo, hi, order=2):
    sos = butter(order, [lo / (FS / 2), hi / (FS / 2)], btype="band", output="sos")
    return sosfiltfilt(sos, x)


def detect_beats(raw):
    """Band-pass 0.5-4 Hz, then find pulse peaks at least 0.3 s apart (max 200 BPM)."""
    y = bandpass(np.asarray(raw, float), 0.5, 4.0)
    peaks, _ = find_peaks(y, distance=int(0.3 * FS), prominence=0.5 * np.std(y))
    return y, peaks


def intervals(peaks):
    """Beat-to-beat intervals in ms, keeping only physiological ones (30-200 BPM)."""
    ibi = np.diff(peaks) / FS * 1000
    return ibi[(ibi > 300) & (ibi < 2000)]


def hrv(ibi):
    """Time-domain HRV: mean HR, SDNN, RMSSD, pNN50."""
    d = np.diff(ibi)
    return {"hr": 60000 / ibi.mean(), "sdnn": ibi.std(ddof=1),
            "rmssd": np.sqrt(np.mean(d ** 2)), "pnn50": 100 * np.mean(np.abs(d) > 50)}


def resp_rate(raw):
    """Respiration from the slow PPG baseline (0.1-0.5 Hz): dominant frequency x 60."""
    if len(raw) < 30 * FS:
        return None, None
    r = bandpass(np.asarray(raw, float), 0.1, 0.5)
    f, p = welch(r, FS, nperseg=min(len(r), int(60 * FS)))
    band = (f >= 0.1) & (f <= 0.5)
    return f[band][np.argmax(p[band])] * 60, r


# ---------------------------------------------------------------- Researcher tab
def draw_research(axes, raw_buf):
    """Redraw the 4 Researcher panels. Returns a one-line stats summary."""
    for ax in axes.flat:
        ax.clear()
    x = np.asarray(raw_buf, float)
    if len(x) < 10 * FS:
        return "Collecting data - the Researcher tab needs 10 s of signal"

    y, peaks = detect_beats(x)
    t = (np.arange(len(x)) - len(x)) / FS              # seconds before now
    stats = []

    # (a) spectrogram, frequency shown in BPM
    ax = axes[0, 0]
    f, ts, S = spectrogram(y, FS, nperseg=256, noverlap=231)
    keep = f <= 4
    ax.pcolormesh(ts + t[0], f[keep] * 60, 10 * np.log10(S[keep] + 1e-12),
                  shading="auto", cmap="magma")
    ax.set(title="Spectrogram", xlabel="Time (s)", ylabel="Frequency (BPM)")

    # (b) beat-to-beat intervals + HRV and relative amplitude
    ax = axes[0, 1]
    ibi = intervals(peaks)
    if len(ibi) >= 3:
        h = hrv(ibi)
        ax.plot(ibi, "o-", ms=3)
        amp = y[peaks]
        stats.append(f"HR {h['hr']:.1f} BPM   SDNN {h['sdnn']:.0f} ms   RMSSD {h['rmssd']:.0f} ms")
        stats.append(f"Amplitude {100 * amp[-3:].mean() / np.median(amp):.0f}% of 60 s median")
    ax.set(title="Beat-to-beat intervals", xlabel="Beat", ylabel="IBI (ms)")

    # (c) pulse shape and its second derivative (last 5 s)
    ax = axes[1, 0]
    n = int(5 * FS)
    apg = np.gradient(np.gradient(y))
    for sig, label in ((y, "PPG"), (apg, "2nd derivative")):
        seg = sig[-n:]
        ax.plot(t[-n:], seg / (np.std(seg) + 1e-9), label=label)
    ax.set(title="Pulse shape (last 5 s)", xlabel="Time (s)", ylabel="Normalised")
    ax.legend(loc="upper right", fontsize=7)

    # (d) respiration from the baseline
    ax = axes[1, 1]
    rate, r = resp_rate(x)
    if rate:
        ax.plot(t, r, color="tab:green")
        stats.append(f"Resp {rate:.0f} breaths/min")
    else:
        ax.text(0.5, 0.5, "Needs 30 s of signal", transform=ax.transAxes,
                ha="center", color="grey")
    ax.set(title="Respiration (0.1-0.5 Hz)", xlabel="Time (s)")

    axes[0, 0].figure.tight_layout()
    return "     |     ".join(stats) or "No clear pulse"


# ---------------------------------------------------------------- recording
class Recorder:
    """Writes raw samples to sessions/session_<date>_<time>.csv while recording is on."""

    def __init__(self, folder="sessions"):
        os.makedirs(folder, exist_ok=True)
        self.path = os.path.join(folder, datetime.now().strftime("session_%Y%m%d_%H%M%S.csv"))
        self.file = self.writer = self.t0 = None
        self.segment, self.active, self.marker = 0, False, ""

    def start(self):
        if self.file is None:                      # file only created on the first recording
            self.file = open(self.path, "w", newline="")
            self.writer = csv.writer(self.file)
            self.writer.writerow(["time_s", "segment", "raw", "bpm_device", "marker"])
        self.segment += 1
        self.active = True

    def stop(self):
        self.active = False
        if self.file:
            self.file.flush()

    def mark(self, text):
        self.marker = text                         # attached to the next sample

    def add(self, t, raw, bpm):
        if not self.active:
            return
        if self.t0 is None:
            self.t0 = t
        self.writer.writerow([f"{t - self.t0:.3f}", self.segment, raw, f"{bpm:.1f}", self.marker])
        self.marker = ""

    def close(self):
        """Close the file. Returns the CSV path, or None if nothing was recorded."""
        if self.file is None:
            return None
        self.file.close()
        self.file = None
        return self.path


# ---------------------------------------------------------------- summary report
def load_session(path):
    t, seg, raw, bpm, marks = [], [], [], [], []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            t.append(float(row["time_s"]))
            seg.append(int(row["segment"]))
            raw.append(float(row["raw"]))
            bpm.append(float(row["bpm_device"]))
            if row["marker"]:
                marks.append(float(row["time_s"]))
    return np.array(t), np.array(seg), np.array(raw), np.array(bpm), marks


def caption(fig, text, y):
    fig.text(0.08, y, "\n".join(textwrap.wrap(text, 105)), fontsize=9, style="italic", va="top")


def make_report(csv_path):
    """Offline analysis of one session -> <csv name>_report.pdf. Returns the path, or None."""
    t, seg, raw, bpm_dev, marks = load_session(csv_path)

    # beats per segment (each segment filtered separately so gaps are never joined)
    segs, ibi, beat_t, beats = [], [], [], []
    w0, w1 = BEAT_WIN
    for s in np.unique(seg):
        m = seg == s
        if m.sum() < 10 * FS:
            continue
        y, pk = detect_beats(raw[m])
        segs.append((t[m], raw[m], y))
        d = np.diff(pk) / FS * 1000
        ok = (d > 300) & (d < 2000)
        ibi.append(d[ok])
        beat_t.append(t[m][pk[1:]][ok])
        beats += [y[p - w0:p + w1] for p in pk if w0 <= p <= len(y) - w1]
    if not segs or sum(map(len, ibi)) < 3:
        return None
    ibi, beat_t = np.concatenate(ibi), np.concatenate(beat_t)

    # signal quality: correlation of each beat with the average beat shape
    B = np.array([(b - b.mean()) / (b.std() + 1e-9) for b in beats])
    r = np.array([np.corrcoef(b, B.mean(axis=0))[0, 1] for b in B])
    good = B[r > 0.9] if np.any(r > 0.9) else B

    h, hr = hrv(ibi), 60000 / ibi
    dev = bpm_dev[bpm_dev > 0]
    rates = [rr for rr, _ in map(resp_rate, (s[1] for s in segs)) if rr]
    start = datetime.strptime(os.path.basename(csv_path)[8:23], "%Y%m%d_%H%M%S")
    lines = [
        f"Session start      {start:%a %d %b %Y, %H:%M:%S}",
        f"Recorded           {len(raw) / FS:.0f} s in {len(np.unique(seg))} segment(s)",
        f"Event markers      {len(marks)}" + (f"  (at {', '.join(f'{m:.0f} s' for m in marks)})" if marks else ""),
        f"Beats detected     {len(beats)}  ({100 * np.mean(r > 0.9):.0f}% good quality, r > 0.9)",
        "",
        f"Heart rate         mean {h['hr']:.1f}   min {hr.min():.1f}   max {hr.max():.1f} BPM",
        f"Device BPM (ESP32) mean {dev.mean():.1f} BPM" if len(dev) else "Device BPM (ESP32) no valid reading",
        f"HRV                SDNN {h['sdnn']:.0f} ms   RMSSD {h['rmssd']:.0f} ms   pNN50 {h['pnn50']:.0f}%",
        f"Respiratory rate   {np.median(rates):.0f} breaths/min" if rates else
        "Respiratory rate   n/a (needs a 30 s segment)",
    ]

    pdf_path = csv_path.replace(".csv", "_report.pdf")
    with PdfPages(pdf_path) as pdf:
        # page 1: summary + heart rate over time
        fig = Figure(figsize=(8.27, 11.69))                       # A4
        fig.text(0.08, 0.95, "Pulse Monitor - Session Report", fontsize=18, weight="bold")
        fig.text(0.08, 0.91, "\n".join(lines), va="top", family="monospace", fontsize=10, linespacing=1.8)
        ax = fig.add_axes([0.1, 0.14, 0.82, 0.38])
        ax.plot(beat_t, hr, ".", ms=3, label="Beat-to-beat (host, offline)")
        ax.plot(t[bpm_dev > 0], dev, ".", ms=1, label="Device BPM (ESP32, live)")
        for mt in marks:
            ax.axvline(mt, ls="--", color="grey")
        ax.set(xlabel="Recording time (s)", ylabel="Heart rate (BPM)")
        ax.legend(loc="upper right")
        ax.grid(alpha=0.3)
        caption(fig, "Figure 1. Heart rate over the session. Points are beat-to-beat rates from offline "
                     "peak detection; the ESP32's live estimate is shown for comparison. Dashed lines "
                     "are event markers.", 0.08)
        pdf.savefig(fig)

        # page 2: spectrogram, Poincare, IBI histogram, ensemble-averaged beat
        fig = Figure(figsize=(8.27, 11.69))
        axs = fig.subplots(2, 2)
        fig.subplots_adjust(left=0.1, right=0.95, top=0.95, bottom=0.22, hspace=0.3, wspace=0.35)

        ts, _, y = max(segs, key=lambda s: len(s[0]))            # longest segment
        nps = min(512, len(y))
        f, st, S = spectrogram(y, FS, nperseg=nps, noverlap=nps - 50)
        keep = f <= 4
        axs[0, 0].pcolormesh(st + ts[0], f[keep] * 60, 10 * np.log10(S[keep] + 1e-12),
                             shading="auto", cmap="magma")
        axs[0, 0].set(xlabel="Recording time (s)", ylabel="Frequency (BPM)")
        axs[0, 1].scatter(ibi[:-1], ibi[1:], s=6)
        axs[0, 1].set(xlabel="IBI n (ms)", ylabel="IBI n+1 (ms)")
        axs[1, 0].hist(ibi, bins=30)
        axs[1, 0].set(xlabel="IBI (ms)", ylabel="Count")
        tb = np.arange(-w0, w1) / FS * 1000
        mu, sd = good.mean(axis=0), good.std(axis=0)
        axs[1, 1].plot(tb, mu)
        axs[1, 1].fill_between(tb, mu - sd, mu + sd, alpha=0.3)
        axs[1, 1].set(xlabel="Time from peak (ms)", ylabel="Normalised amplitude")
        for ax, lab in zip(axs.flat, "abcd"):
            ax.text(-0.18, 1.04, f"({lab})", transform=ax.transAxes, weight="bold")
        for ax in list(axs.flat)[1:]:
            ax.grid(alpha=0.3)
        caption(fig, "Figure 2. (a) Spectrogram of the longest recording segment; the bright band is the "
                     "pulse rate. (b) Poincaré plot of successive beat intervals; spread across the identity "
                     "line reflects short-term variability. (c) Distribution of beat intervals. "
                     "(d) Ensemble-averaged pulse (mean ± SD of good-quality beats, aligned on their peaks).", 0.15)
        pdf.savefig(fig)
    return pdf_path


if __name__ == "__main__":
    # Self-test without the sensor: 2 min synthetic PPG, ~72 BPM, 15 breaths/min
    rec = Recorder()
    rec.start()
    n = int(120 * FS)
    tt = np.arange(n) / FS
    pulse_hz = 1.2 + 0.05 * np.sin(2 * np.pi * 0.25 * tt)          # respiratory sinus arrhythmia
    sig = (2000 + 150 * np.sin(2 * np.pi * np.cumsum(pulse_hz) / FS)
           + 60 * np.sin(2 * np.pi * 0.25 * tt) + 10 * np.random.randn(n))
    for i in range(n):
        if i == n // 2:
            rec.mark("Event marker")
        rec.add(tt[i], int(sig[i]), 72.0)
    print(make_report(rec.close()))