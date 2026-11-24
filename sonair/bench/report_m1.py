"""M1 report: tracking of the simulated baselines on the M1 command set (before any real data exists).
Writes reports/m1_summary.json, reports/m1_tracking_by_speed.png, reports/m1_lag_example.png."""
import glob, json, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from .. import cell, runlog
from . import metrics as M

OUT = os.path.join(cell.ROOT, "reports")
C_B1, C_B2, C_T = "#eb6834", "#2a78d6", "#8a8984"          # categorical slots 2 / 1 + neutral ink
LABEL = {"B1_menagerie_pd": "B1 Menagerie PD", "B2_pd_invdyn": "B2 PD + inverse-dynamics feed-forward"}


def _font():
    for f in font_manager.findSystemFonts():
        if "NotoSansCJK" in f or "NotoSansSC" in f or "msyh" in f.lower():
            font_manager.fontManager.addfont(f); return font_manager.FontProperties(fname=f).get_name()
    return None


def main():
    os.makedirs(OUT, exist_ok=True)
    fn = _font()
    plt.rcParams.update({"font.family": [fn, "DejaVu Sans"] if fn else ["DejaVu Sans"], "axes.spines.top": False, "axes.spines.right": False,
                         "axes.edgecolor": "#b9b8b2", "axes.labelcolor": "#52514e", "xtick.color": "#52514e", "ytick.color": "#52514e",
                         "axes.grid": True, "grid.color": "#e6e5e0", "grid.linewidth": 0.6, "font.size": 10})
    summary = {}
    for d in sorted(glob.glob(os.path.join(cell.ROOT, "runs", "*"))):
        meta, df = runlog.read(d)
        if meta["variant"] == "B0_kinematic": continue
        summary[meta["run_id"]] = dict(command=meta["command_id"], variant=meta["variant"], payload=meta["payload"], **M.tracking(df))
    json.dump(summary, open(os.path.join(OUT, "m1_summary.json"), "w"), ensure_ascii=False, indent=1)

    # ---- figure 1: tool0 tracking RMS by speed bin, excitation (train seed), one panel per payload
    pays = [("gripper_open", "gripper, empty"), ("camtool", "Mode B: camera tool"), ("s2", "Mode A: specimen S2")]
    fig, axes = plt.subplots(1, 3, figsize=(11, 3.6), sharey=True)
    for ax, (pay, title) in zip(axes, pays):
        for var, col in (("B1_menagerie_pd", C_B1), ("B2_pd_invdyn", C_B2)):
            acc = {b: [] for b in M.BIN_NAMES}
            for rid, r in summary.items():
                if r["variant"] == var and r["command"].startswith(f"exc_{pay}_") and r["command"].endswith("_s1"):
                    for b, v in r["by_speed"].items(): acc[b].append(v["tcp_rms_mm"])
            xs = [i for i, b in enumerate(M.BIN_NAMES) if acc[b]]; ys = [float(np.mean(acc[M.BIN_NAMES[i]])) for i in xs]
            ax.plot(xs, ys, color=col, lw=2, marker="o", ms=7, mec="white", mew=1.5, label=LABEL[var])
            ax.annotate(f"{ys[-1]:.2g} mm", (xs[-1], ys[-1]), textcoords="offset points", xytext=(6, 4), fontsize=9, color="#3d3d3a")
        ax.set_yscale("log"); ax.set_xticks(range(len(M.BIN_NAMES))); ax.set_xticklabels(M.BIN_NAMES); ax.set_title(title, fontsize=11, color="#0b0b0b")
        ax.set_xlabel("commanded joint speed level (rad/s)"); ax.axvline(1.5, color="#c3c2b7", lw=1, ls=":")
    axes[0].set_ylabel("flange tracking error RMS (mm, log)")
    axes[0].legend(frameon=False, loc="center left", fontsize=9)
    fig.text(0.5, -0.02, "Dotted line: 0.6 rad/s, the elbow speed above which the Isaac Sim pilot diverged. Same excitation set, seed s1, mean per level.", ha="center", fontsize=9, color="#52514e")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "m1_tracking_by_speed.png"), dpi=200, bbox_inches="tight"); plt.close(fig)

    # ---- figure 2: why the controller model matters (lag), elbow joint of one excitation run
    rid = "exc_camtool_v0.9_s1"
    _, d1 = runlog.read(os.path.join(cell.ROOT, "runs", rid + "__B1_menagerie_pd"))
    _, d2 = runlog.read(os.path.join(cell.ROOT, "runs", rid + "__B2_pd_invdyn"))
    s = (d1["t"] > 8) & (d1["t"] < 12)
    fig, ax = plt.subplots(figsize=(8, 3.2))
    ax.plot(d1["t"][s], d1["tq2"][s], color=C_T, lw=2, ls="--", label="target_q")
    ax.plot(d1["t"][s], d1["q2"][s], color=C_B1, lw=2, label=LABEL["B1_menagerie_pd"])
    ax.plot(d2["t"][s], d2["q2"][s], color=C_B2, lw=2, label=LABEL["B2_pd_invdyn"] + " (on top of the target)")
    lag = summary[rid + "__B1_menagerie_pd"]["lag_ms"][2]
    ax.set_xlabel("time (s)"); ax.set_ylabel("elbow q3 (rad)")
    ax.set_title(f"{rid}: the Menagerie PD lags the target by about {lag:.0f} ms", fontsize=11, color="#0b0b0b")
    ax.legend(frameon=False, fontsize=9, loc="best")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "m1_lag_example.png"), dpi=200, bbox_inches="tight"); plt.close(fig)
    print("wrote", OUT)


if __name__ == "__main__":
    main()
