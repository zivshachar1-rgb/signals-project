import numpy as np
from scipy.signal import find_peaks
import matplotlib.pyplot as plt
from datetime import datetime

FS = 50  # Hz (20 ms sampling)


def beat_intervals(raw):
    x = np.asarray(raw, float)
    if len(x) < 2 * FS:
        return np.array([])
    peaks, _ = find_peaks(x, distance=int(0.3 * FS), prominence=0.5 * x.std())
    ibi = np.diff(peaks) * 1000 / FS
    return ibi[(ibi > 300) & (ibi < 2000)]


def hrv(ibi):
    if len(ibi) < 3:
        return None
    d = np.diff(ibi)
    return {"SDNN": ibi.std(ddof=1),
            "RMSSD": np.sqrt(np.mean(d ** 2)),
            "pNN50": 100 * np.mean(np.abs(d) > 50)}


def save_session(raw, bpm, events, low, high):
    raw, bpm = np.asarray(raw, float), np.asarray(bpm, float)
    ok = ~np.isnan(bpm)
    if len(raw) < 10 * FS or not ok.any():
        return
    t = np.arange(len(raw)) / FS
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    np.savetxt(f"session_{stamp}.csv", np.column_stack([t, raw, bpm]), delimiter=",",
               header="time_s,ppg,bpm", comments="", fmt=["%.2f", "%d", "%.1f"])

    ibi = beat_intervals(raw)
    alarm = ok & ((bpm < low) | (bpm > high))
    lines = [f"Duration: {t[-1]:.0f} s",
             f"BPM min/mean/max: {np.nanmin(bpm):.1f} / {np.nanmean(bpm):.1f} / {np.nanmax(bpm):.1f}",
             f"Valid pulse: {100 * ok.mean():.0f} %",
             f"Time in alarm: {alarm.sum() / FS:.0f} s"]
    lines += [f"{k}: {v:.1f}" for k, v in (hrv(ibi) or {}).items()]
    lines += ["", "Events:"] + [f"{te:.1f} s  {msg}" for te, msg in events]
    open(f"session_{stamp}.txt", "w").write("\n".join(lines))

    fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 4))
    a1.plot(t, bpm, color="#d6336c")
    a1.axhline(low, ls="--", c="grey")
    a1.axhline(high, ls="--", c="grey")
    for te, _ in events:
        a1.axvline(te, c="k", alpha=0.2)
    a1.set(xlabel="Time (s)", ylabel="Heart rate (BPM)")
    a2.scatter(ibi[:-1], ibi[1:], s=10)
    a2.set(xlabel="IBI n (ms)", ylabel="IBI n+1 (ms)", aspect="equal")
    fig.tight_layout()
    fig.savefig(f"session_{stamp}.png", dpi=200)
    plt.close(fig)