#!/usr/bin/env python3
"""Plot recorded needle-injection data (position and external force).

Usage:
  python3 plot_injection.py                      # newest CSV in Needle_experiment/
  python3 plot_injection.py FILE.csv [FILE.csv ...]
  python3 plot_injection.py Needle_experiment/correct/0deg   # every rep in a folder
Options:
  --smooth MS   moving average over MS milliseconds for force (default 0 = raw)
  --save        write PNGs next to the CSV instead of only showing windows
  --out DIR     write PNGs into DIR (implies --save), e.g. --out build/figures

One file  -> figure 1: depth, TCP position, external force over time
             figure 2: force along the needle vs. depth (insert / retract)
More files -> additionally a comparison of force vs. depth during insertion.
"""
import argparse
import glob
import os
import sys

import matplotlib

if "--out" in sys.argv and "--save" not in sys.argv:
    sys.argv.append("--save")
if not os.environ.get("DISPLAY") and "--save" not in sys.argv:
    print("No display found -> saving PNGs instead (--save).")
    sys.argv.append("--save")
if "--save" in sys.argv:
    matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
DATA_ROOT = os.path.join(HERE, "Needle_experiment")

# Validated categorical palette (fixed order) + neutral inks.
C1, C2, C3 = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK_2, GRID = "#0b0b0b", "#52514e", "#e4e3df"

plt.rcParams.update({
    "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb",
    "axes.edgecolor": INK_2, "axes.labelcolor": INK, "axes.titlecolor": INK,
    "xtick.color": INK_2, "ytick.color": INK_2, "text.color": INK,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8,
    "axes.spines.top": False, "axes.spines.right": False,
    "lines.linewidth": 1.6, "legend.frameon": False, "font.size": 10,
})


def find_files(args):
    if not args:
        files = glob.glob(os.path.join(DATA_ROOT, "**", "*.csv"), recursive=True)
        if not files:
            sys.exit("No CSV files found in " + DATA_ROOT)
        return [max(files, key=os.path.getmtime)]
    files = []
    for a in args:
        if os.path.isdir(a):
            found = glob.glob(os.path.join(a, "**", "rep*.csv"), recursive=True)
            files += sorted(found, key=lambda f: int("".join(c for c in os.path.basename(f) if c.isdigit()) or 0))
        else:
            files.append(a)
    return files


def load(path, smooth_ms):
    df = pd.read_csv(path)
    # Continuous time: retraction starts where insertion ended.
    t_insert_end = df.loc[df.phase == 1, "time"].max() if (df.phase == 1).any() else 0.0
    df["t"] = np.where(df.phase == 2, df.time + t_insert_end, df.time)
    # Force columns: TCP frame if recorded (new format), else base frame (old format).
    tcp = "F_z_tcp" in df.columns
    cols = ["F_x_tcp", "F_y_tcp", "F_z_tcp"] if tcp else ["F_x", "F_y", "F_z"]
    for c, name in zip(cols, ["Fx", "Fy", "Fz"]):
        f = df[c]
        if smooth_ms > 1:
            f = f.rolling(int(smooth_ms), center=True, min_periods=1).mean()
        df[name] = f
    df.attrs["frame"] = "TCP frame (z = needle axis)" if tcp else "base frame"
    df.attrs["tcp"] = tcp
    df.attrs["t_insert_end"] = t_insert_end
    return df


def label_for(path):
    # condition/program/repN, e.g. correct/0deg/rep7
    parts = os.path.normpath(os.path.abspath(path)).split(os.sep)
    return "/".join(parts[-3:]).replace(".csv", "")


def mark_phases(ax, df):
    ax.axvline(df.attrs["t_insert_end"], color=INK_2, lw=0.8, ls=":")


OUT_DIR = None  # set by --out


def png_path(path, suffix):
    """Next to the CSV, or in OUT_DIR as condition_program_repN_<suffix>.png."""
    if OUT_DIR:
        return os.path.join(OUT_DIR, label_for(path).replace("/", "_") + "_" + suffix + ".png")
    return os.path.splitext(path)[0] + "_" + suffix + ".png"


def plot_single(path, df, save):
    name = label_for(path)
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 8.5), constrained_layout=True)
    fig.suptitle(name, fontweight="bold")

    # 1) Depth along the needle: measured vs. planned.
    ax = axes[0]
    ax.plot(df.t, df.depth_meas * 1000, color=C1, label="measured")
    if "s_cmd" in df:
        ax.plot(df.t, df.s_cmd * 1000, color=INK_2, ls="--", lw=1.2, label="planned (S-curve)")
    ax.set_ylabel("depth along needle [mm]")
    ax.set_title("Insertion depth", loc="left")
    ax.legend(loc="upper right")
    mark_phases(ax, df)
    ymax = ax.get_ylim()[1]
    ax.text(df.attrs["t_insert_end"] / 2, ymax, "insert", ha="center", va="top", color=INK_2)
    ax.text(df.attrs["t_insert_end"] * 1.5, ymax, "retract", ha="center", va="top", color=INK_2)

    # 2) TCP position relative to the start of insertion.
    ax = axes[1]
    for c, color in zip("xyz", (C1, C2, C3)):
        ax.plot(df.t, (df[c] - df[c].iloc[0]) * 1000, color=color, label=c)
    ax.set_ylabel("Δ position, base frame [mm]")
    ax.set_title("Needle tip position (relative to start)", loc="left")
    ax.legend(loc="upper right", ncol=3)
    mark_phases(ax, df)

    # 3) Estimated external force.
    ax = axes[2]
    for c, color in zip(("Fx", "Fy", "Fz"), (C1, C2, C3)):
        ax.plot(df.t, df[c], color=color, lw=1.0, label=c)
    ax.axhline(0, color=INK_2, lw=0.8)
    ax.set_ylabel("force [N]")
    ax.set_xlabel("time [s]")
    ax.set_title("Estimated external force, " + df.attrs["frame"], loc="left")
    ax.legend(loc="upper right", ncol=3)
    mark_phases(ax, df)

    # Figure 2: force along the needle vs. depth.
    fig2, ax = plt.subplots(figsize=(7, 5), constrained_layout=True)
    for phase, color, lbl in ((1, C1, "insert"), (2, C2, "retract")):
        d = df[df.phase == phase]
        ax.plot(d.depth_meas * 1000, d.Fz, color=color, lw=1.0, label=lbl)
    ax.axhline(0, color=INK_2, lw=0.8)
    ax.set_xlabel("depth along needle [mm]")
    ax.set_ylabel("Fz [N]")
    ax.set_title(name + ": force along needle vs. depth", loc="left")
    ax.legend()

    if save:
        for f, suffix in ((fig, "time"), (fig2, "force_depth")):
            out = png_path(path, suffix)
            f.savefig(out, dpi=130)
            plt.close(f)
            print("Saved", out)


def plot_compare(files, dfs, save):
    # Only files with force along the needle (TCP frame); old base-frame files have another sign.
    pairs = [(f, d) for f, d in zip(files, dfs) if d.attrs["tcp"]]
    for f, d in zip(files, dfs):
        if not d.attrs["tcp"]:
            print("  compare: skipping %s (old format, force in base frame)" % label_for(f))
    if len(pairs) < 2:
        return
    fig, ax = plt.subplots(figsize=(8, 5), constrained_layout=True)
    # Mean only over the depth range that every rep reached.
    common = min(d.loc[d.phase == 1, "depth_meas"].max() for _, d in pairs)
    grid = np.linspace(0, common, 400)
    curves = []
    for f, df in pairs:
        d = df[df.phase == 1].sort_values("depth_meas")
        ax.plot(d.depth_meas * 1000, d.Fz, color="#b9b8b2", lw=0.8)
        curves.append(np.interp(grid, d.depth_meas, d.Fz))
        print("  %-22s depth %5.1f mm   Fz max %6.2f N" % (label_for(f), d.depth_meas.max() * 1000, d.Fz.max()))
    ax.plot(grid * 1000, np.mean(curves, axis=0), color=C1, lw=2.0, label="mean (0-%.0f mm)" % (common * 1000))
    ax.plot([], [], color="#b9b8b2", lw=0.8, label="single reps (%d)" % len(pairs))
    ax.axhline(0, color=INK_2, lw=0.8)
    ax.set_xlabel("depth along needle [mm]")
    ax.set_ylabel("Fz [N]")
    ax.set_title("Insertion force vs. depth, %d reps" % len(pairs), loc="left")
    ax.legend()
    if save:
        folder = os.path.dirname(pairs[0][0])
        if OUT_DIR:
            out = os.path.join(OUT_DIR, "_".join(label_for(pairs[0][0]).split("/")[:2]) + "_compare_force_depth.png")
        else:
            out = os.path.join(folder, "compare_force_depth.png")
        fig.savefig(out, dpi=130)
        print("Saved", out)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("files", nargs="*")
    p.add_argument("--smooth", type=float, default=0, help="force moving average [ms]")
    p.add_argument("--save", action="store_true")
    p.add_argument("--out", help="folder for PNGs (implies --save)")
    a = p.parse_args()
    global OUT_DIR
    if a.out:
        OUT_DIR = os.path.abspath(a.out)
        os.makedirs(OUT_DIR, exist_ok=True)

    files = find_files(a.files)
    dfs = [load(f, a.smooth) for f in files]
    for f, df in zip(files, dfs):
        print("%s: %d samples, final depth %.2f mm" % (f, len(df), df.loc[df.phase == 1, "depth_meas"].iloc[-1] * 1000))
        if len(files) <= 3 or a.save:  # saving: one figure set per rep
            plot_single(f, df, a.save)
    if len(files) > 1:
        plot_compare(files, dfs, a.save)
    if not a.save:
        plt.show()


if __name__ == "__main__":
    main()
