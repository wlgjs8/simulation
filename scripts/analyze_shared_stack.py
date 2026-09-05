"""Inspect commanded motion and plant tracking separately; no success scoring."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
from scipy.signal import butter, sosfiltfilt


def analyze(directory):
    directory = Path(directory)
    with open(directory / "joints.csv") as stream:
        rows = list(csv.DictReader(stream))
    summary = json.loads((directory / "summary.json").read_text())
    report = {"episode_exit_code": summary["exit_code"],
              "fault_reason": summary.get("fault_reason"),
              "band_hz": [3, 15], "filter_edge_trim_sec": 0.5,
              "note": "Band RMS is tracking error, not task success or an A/B improvement score.",
              "arms": {}}
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(12, 6), sharex=True)
    for r, side in enumerate(("left", "right")):
        arm = [x for x in rows if x["side"] == side]
        ticks = np.asarray([int(x["time_ns"]) for x in arm], dtype=np.int64)
        if len(ticks) < 3 or not np.all(np.diff(ticks) == 2_000_000):
            raise ValueError("log must contain consecutive 2 ms physical ticks")
        t = (ticks - 1_000_000_000) * 1e-9
        def values(prefix):
            return np.asarray([[float(x[f"{prefix}{i}"]) for i in range(1, 7)] for x in arm])
        q, target = values("q_deg_"), values("target_deg_")
        error = q - target
        stats = {"max_abs_error_deg": float(np.abs(error).max()),
                 "rms_error_per_joint_deg": np.sqrt(np.mean(error**2, axis=0)).tolist(),
                 "max_command_speed_deg_s": float(np.abs(np.diff(target, axis=0) / 0.002).max()),
                 "max_measured_speed_deg_s": float(np.abs(values("dq_deg_s_")).max())}
        if len(ticks) > 1000:
            band = sosfiltfilt(butter(4, [3, 15], btype="bandpass", fs=500, output="sos"), error, axis=0)
            stats["tracking_band_rms_per_joint_deg"] = np.sqrt(np.mean(band[250:-250]**2, axis=0)).tolist()
        report["arms"][side] = stats
        axes[r, 0].plot(t, target[:, 5], label="C++ target", linewidth=1.3)
        axes[r, 0].plot(t, q[:, 5], label="PhysX measured", linewidth=0.8, alpha=0.85)
        axes[r, 0].set_title(f"{side.title()} wrist J6 (deg)")
        axes[r, 0].legend()
        for j in range(6):
            axes[r, 1].plot(t, error[:, j], label=f"J{j+1}", linewidth=0.7)
        axes[r, 1].set_title(f"{side.title()} measured - target (deg)")
        axes[r, 1].legend(ncol=3, fontsize=8)
    for ax in axes.flat:
        ax.grid(alpha=0.2)
        ax.set_xlabel("Simulation time (s)")
    fig.suptitle(f"Shared C++ controller / PhysX plant — {directory.name}")
    fig.tight_layout()
    fig.savefig(directory / "tracking.png", dpi=150)
    plt.close(fig)
    force_path = directory/'force.csv'
    if force_path.exists():
        force_rows = list(csv.DictReader(force_path.open()))
        start_ns = summary.get('episode_start_time_ns', 1_000_000_000)
        report['force'] = {}
        fig, axes = plt.subplots(2, 3, figsize=(13, 6), sharex=True)
        for i, side in enumerate(('left', 'right')):
            rows = [r for r in force_rows if r['side'] == side and int(r['time_ns']) >= start_ns]
            t = (np.array([int(r['time_ns']) for r in rows])-start_ns)*1e-9
            force = np.linalg.norm([[float(r[k]) for k in ('fx','fy','fz')] for r in rows], axis=1)
            gate = np.array([float(r['gate']) for r in rows])
            dev = np.array([float(r['deviation_m']) for r in rows])
            lead = np.array([float(r['lead_m']) for r in rows])
            report['force'][side] = dict(
                peak_post_deadzone_wrench_force_n=float(force.max()),
                p95_post_deadzone_wrench_force_n=float(np.percentile(force,95)),
                max_actual_lead_mm=float(lead.max()*1000),
                max_deviation_mm=float(dev.max()*1000),
                fraction_at_40mm_fence=float(np.mean(dev >= 0.04-1e-6)),
                fraction_gate_below_0p1=float(np.mean(gate < 0.1)),
                fraction_covered=float(np.mean([r['covered'] == 'True' for r in rows])))
            axes[i,0].plot(t,force,lw=0.8)
            axes[i,0].set_ylabel(side)
            axes[i,0].set_title('Measured force norm after deadzone (N)')
            axes[i,1].plot(t,gate,lw=0.8)
            axes[i,1].set_title('Existing C++ force gate')
            axes[i,2].plot(t,lead*1000,label='Actual lead',lw=0.8)
            axes[i,2].plot(t,dev*1000,label='Force deviation',lw=0.8)
            axes[i,2].axhline(35,color='r',ls='--',lw=0.7,label='Lead limit')
            axes[i,2].set_title('Distance (mm)')
            axes[i,2].legend(fontsize=8)
        for ax in axes.flat:
            ax.set_xlabel('Policy elapsed time (s)')
            ax.grid(alpha=0.2)
        fig.tight_layout()
        fig.savefig(directory/'force_tracking.png',dpi=150)
        plt.close(fig)
    (directory / "tracking_metrics.json").write_text(json.dumps(report, indent=2))
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("directory", type=Path)
    print(json.dumps(analyze(parser.parse_args().directory), indent=2))
