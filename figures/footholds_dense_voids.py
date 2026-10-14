"""Footholds of GaitNet and of OCS2's trot on the same 60 %-void Gazebo world (ground-truth terrain, 0.1 m/s).

Data: campaign trials cp_gt_gnfull_d60_v0.10_t5 and cp_gt_trot_d60_v0.10_t5 (gaitnet-runs/idm/campaign_1), world
gridvoid24_60_s5.world. Touchdowns come from the 50 Hz ground-truth foot positions (monitor.csv) with the rule of
inv/loop/idm/landing_events_z.py: a foot is down on a tile top at z = 0.32 m (0.30 m pillar + 2 cm foot radius); a
swing starts when z rises above 0.33 m and ends when it comes back below. A foot in a void is the first sample below
0.25 m (the trial's termination rule); the marker sits where that foot crossed the tile-top level on the way down.

    python footholds_dense_voids.py [out_stem]
"""
import re
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

RUNS = Path("/scratch/sselvaraj/gaitnet-runs")
WORLD = RUNS / "worlds/gridvoid24_60_s5.world"
TRIALS = [("cp_gt_gnfull_d60_v0.10_t5", "GaitNet (GaitNet-T + IDM)", "#2a78d6"),
          ("cp_gt_trot_d60_v0.10_t5", "OCS2 trot", "#eb6834")]
FAIL_MARK = INK_MARK = "#0b0b0b"   # ink, not status red: red sits too close to the trot orange (validator: normal-vision dE 10.8)
SURFACE, INK, INK2 = "#fcfcfb", "#0b0b0b", "#52514e"
TILE_FILL, VOID_FILL = "#e6e4de", "#55534e"
SURF_Z, DOWN_Z, VOID_Z = 0.30, 0.33, 0.25
FEET = ("fl", "fr", "rl", "rr")
X_LIM, Y_LIM = (-0.35, 1.25), (-0.40, 0.40)


def read_world(path):
    txt = path.read_text()
    tiles = [(float(x), float(y)) for x, y in re.findall(r'<link name="t\d+">\s*<pose>([-\d.]+) ([-\d.]+) ', txt)]
    pad = re.search(r'<link name="start_pad">\s*<pose>([-\d.]+) ([-\d.]+) [-\d.]+ .*?</pose>\s*<collision name="c">'
                    r'<geometry><box><size>([\d.]+) ([\d.]+)', txt, re.S)
    return tiles, tuple(float(v) for v in pad.groups())


def read_monitor(path):
    with open(path) as f:
        cols = f.readline().strip().split(",")
    a = np.genfromtxt(path, delimiter=",", skip_header=1)
    a = a[a[:, cols.index("t")] > 0]
    return {c: a[:, i] for i, c in enumerate(cols)}


def touchdowns(m):
    """Touchdowns before the trial ends (first foot in a void, if any): the foot comes back below 0.33 m and stays on
    the tile-top level (0.29-0.335 m) for the next 3 samples (60 ms), so a foot passing down into a void is not one."""
    void = None
    for f in FEET:
        z, x, y = m[f"foot_{f}_z"], m[f"foot_{f}_x"], m[f"foot_{f}_y"]
        below = np.where(z < VOID_Z)[0]
        if len(below) and (void is None or m["t"][below[0]] < void[3]):
            k = below[0]
            while k > 0 and z[k - 1] < SURF_Z:
                k -= 1
            void = (x[k], y[k], f, m["t"][below[0]])
    t_end = m["t"][-1] if void is None else void[3]
    out = []
    for f in FEET:
        z, x, y = m[f"foot_{f}_z"], m[f"foot_{f}_x"], m[f"foot_{f}_y"]
        up = z > DOWN_Z
        for k in np.where(up[:-1] & ~up[1:])[0] + 1:
            stays = z[k:k + 3]
            if m["t"][k] < t_end and len(stays) == 3 and np.all((stays > SURF_Z - 0.01) & (stays < DOWN_Z + 0.005)):
                out.append((x[k], y[k], f))
    return out, void


def main(stem):
    tiles, (pad_x, pad_y, pad_l, pad_w) = read_world(WORLD)
    plt.rcParams.update({"font.size": 7, "axes.titlesize": 7.5, "axes.labelsize": 7, "xtick.labelsize": 6.5,
                         "ytick.labelsize": 6.5, "font.family": "DejaVu Sans"})
    fig, axes = plt.subplots(2, 1, figsize=(3.45, 3.55), sharex=True, constrained_layout=True)
    fig.patch.set_facecolor(SURFACE)
    for ax, (name, label, color) in zip(axes, TRIALS):
        ax.set_facecolor(VOID_FILL)
        ax.add_patch(Rectangle((pad_x - pad_l / 2, pad_y - pad_w / 2), pad_l, pad_w, fc=TILE_FILL, ec="none", zorder=1))
        for tx, ty in tiles:
            ax.add_patch(Rectangle((tx - 0.12, ty - 0.12), 0.24, 0.24, fc=TILE_FILL, ec=VOID_FILL, lw=0.3, zorder=1))
        m = read_monitor(RUNS / "idm/campaign_1" / name / "monitor.csv")
        tds, void = touchdowns(m)
        end = len(m["t"]) if void is None else np.searchsorted(m["t"], void[3])
        ax.plot(m["x"][:end], m["y"][:end], color=color, lw=1.0, alpha=0.9, zorder=2, solid_capstyle="round")
        xs = np.array([t[0] for t in tds]); ys = np.array([t[1] for t in tds])
        ax.scatter(xs, ys, s=11, color=color, edgecolors=SURFACE, linewidths=0.5, zorder=3)
        dist = (m["x"][end - 1] - m["x"][0])
        if void is None:
            status = f"walked {dist:.2f} m, no fall"
        else:
            vx, vy, vf, vt = void
            ax.scatter([vx], [vy], s=58, marker="X", color=FAIL_MARK, edgecolors=SURFACE, linewidths=1.0, zorder=4)
            ax.annotate(f"{vf.upper()} foot steps into a void", (vx, vy), xytext=(vx - 0.02, Y_LIM[1] - 0.07),
                        ha="right", va="center", fontsize=6.5, color=INK, zorder=5,
                        bbox=dict(boxstyle="round,pad=0.2", fc=SURFACE, ec="none", alpha=0.95),
                        arrowprops=dict(arrowstyle="-", color=INK, lw=0.6))
            status = f"falls at {dist:.2f} m (t = {vt - m['t'][0]:.1f} s)"
        ax.set_title(f"{label}: {status}", loc="left", color=INK, pad=3)
        ax.set_xlim(*X_LIM); ax.set_ylim(*Y_LIM); ax.set_aspect("equal")
        ax.set_ylabel("y (m)", color=INK2)
        for s in ax.spines.values():
            s.set_visible(False)
        ax.tick_params(colors=INK2, length=2, width=0.5)
    axes[-1].set_xlabel("x (m), walking direction →", color=INK2)
    for ext in ("pdf", "png"):
        fig.savefig(f"{stem}.{ext}", dpi=300, facecolor=SURFACE)
    print(f"{stem}.pdf / .png")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else str(Path(__file__).with_name("footholds_dense_voids")))
