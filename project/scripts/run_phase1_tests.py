#!/usr/bin/env python3
"""Run the MRV Phase 1 verification (Test 1 / 2 / 3) and write the report.

    cd ~/space_robotics_bench
    ~/isaac-sim/python.sh project/scripts/run_phase1_tests.py            # both scenarios, headless
    ~/isaac-sim/python.sh project/scripts/run_phase1_tests.py --report   # only rebuild the report

Each scenario runs in its own Isaac Sim process (`vision_capture.py`):
- static  -> TEST 1 (pose accuracy)
- dynamic (0.01 and 0.02 m/s) -> TEST 2 (drift intercept + capture) and TEST 3 (holding)

Writes `project/logs/vision_capture_metrics.csv` (all runs, `scenario` column = run name)
and `project/logs/phase1_result.md`. Exit code 0 only if all three tests pass.
"""

import argparse
import csv
import json
import math
import os
import platform
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[1]
LOG_DIR = PROJECT / "logs"
OUT_DIR = LOG_DIR / "vision_capture"
SCRIPT = PROJECT / "scripts" / "vision_capture.py"
PYTHON = os.environ.get("ISAAC_PYTHON", os.path.expanduser("~/isaac-sim/python.sh"))


# (run name, scenario, extra arguments): the dynamic test is run at both ends of the
# required 0.01 - 0.02 m/s drift range
RUNS = [
    ("static", "static", []),
    ("dynamic", "dynamic", ["--set", "mep.motion_mode=translation_only", "--set", "mep.linear_velocity_mps=0.01"]),
    ("dynamic_v002", "dynamic", ["--set", "mep.motion_mode=translation_only", "--set", "mep.linear_velocity_mps=0.02"]),
]


def run_scenario(name: str, scenario: str, extra: list) -> dict:
    log = OUT_DIR / f"{name}_console.log"
    cmd = [PYTHON, str(SCRIPT), "--scenario", scenario, "--tag", name, "--headless", *extra]
    print(f"[RUN] {' '.join(cmd)}  (log: {log})", flush=True)
    t0 = time.time()
    with open(log, "w") as f:
        proc = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT, cwd=PROJECT.parent)
    print(f"[RUN] {name}: exit code {proc.returncode} after {time.time() - t0:.0f} s", flush=True)
    return load_result(name, proc.returncode)


def load_result(name: str, returncode=None) -> dict:
    path = OUT_DIR / f"{name}_result.json"
    if not path.exists():
        return {"scenario": name, "missing": True, "returncode": returncode, "checks": {}, "metrics": {}}
    data = json.loads(path.read_text())
    data["returncode"] = returncode
    data["mtime"] = datetime.fromtimestamp(path.stat().st_mtime).isoformat(timespec="seconds")
    return data


def merge_csv(names=tuple(r[0] for r in RUNS)) -> Path:
    out = LOG_DIR / "vision_capture_metrics.csv"
    header, rows = None, []
    for n in names:
        p = OUT_DIR / f"{n}_metrics.csv"
        if not p.exists():
            continue
        with open(p) as f:
            r = csv.DictReader(f)
            header = header or r.fieldnames
            rows.extend(r)
    if header:
        with open(out, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=header)
            w.writeheader()
            w.writerows(rows)
    return out


def test_verdicts(static: dict, dynamics: list) -> dict:
    def group(results, prefix):
        items, reasons = {}, []
        for res in results:
            tag = res.get("run", res.get("scenario"))
            mine = {k: v for k, v in res.get("checks", {}).items() if k.startswith(prefix)}
            items.update({f"{k} ({tag})" if len(results) > 1 else k: v for k, v in mine.items()})
            reasons += [f"[{tag}] {k}: {v.get('detail', '')}" for k, v in mine.items() if not v["pass"]]
            if res.get("missing"):
                reasons.insert(0, f"[{tag}] result missing (run did not finish)")
            if res.get("failure"):
                reasons.append(f"[{tag}] final state {res.get('final_state')}: {res['failure']}")
        ok = bool(items) and not reasons
        return {"pass": ok, "reasons": reasons, "checks": items}

    t1 = group([static], "[TEST1]")
    t2 = group(dynamics, "[TEST2]")
    t3 = group(dynamics, "[TEST3]")
    # a failed setup check (mass, gravity, drift, layout, K, IK) fails the scenario's tests
    for res, tests in [(static, [t1])] + [(d, [t2, t3]) for d in dynamics]:
        for k, v in res.get("checks", {}).items():
            if not k.startswith("[") and not v["pass"]:
                for t in tests:
                    t["pass"] = False
                    t["reasons"].append(f"setup: {k}: {v.get('detail', '')}")
    return {"TEST 1": t1, "TEST 2": t2, "TEST 3": t3}


def _f(x, fmt="{:.3f}"):
    if x is None or (isinstance(x, float) and math.isnan(x)):
        return "n/a"
    return fmt.format(x)


def environment() -> dict:
    def cmd(c):
        try:
            return subprocess.run(c, shell=True, capture_output=True, text=True, timeout=20).stdout.strip()
        except Exception:
            return "unknown"

    isaac = Path(os.path.expanduser("~/isaac-sim/VERSION"))
    return {
        "Date": datetime.now().isoformat(timespec="seconds"),
        "OS": cmd("lsb_release -ds") or platform.platform(),
        "GPU": cmd("nvidia-smi --query-gpu=name,driver_version --format=csv,noheader"),
        "Isaac Sim": isaac.read_text().strip() if isaac.exists() else "unknown",
        "Isaac Lab": cmd("git -C ~/isaaclab describe --tags"),
        "Space Robotics Bench": "srb " + (cmd(f"grep -m1 '^version' {PROJECT}/pyproject.toml").split("=")[-1].strip().strip('"')),
        "Git": cmd(f"git -C {PROJECT} rev-parse --abbrev-ref HEAD") + " @ " + cmd(f"git -C {PROJECT} rev-parse --short HEAD")
        + (" (uncommitted changes)" if cmd(f"git -C {PROJECT} status --porcelain") else ""),
        "Task": "srb/debris_capture_vision (env.robot=canadarm3)",
    }


def write_report(static: dict, dynamics: list, verdicts: dict, csv_path: Path) -> Path:
    dynamic = dynamics[0]
    env = environment()
    setup = dynamic.get("metrics", {}).get("setup") or static.get("metrics", {}).get("setup") or {}
    cfg = setup.get("config", {})
    mep, cam, tag = cfg.get("mep", {}), cfg.get("camera", {}), cfg.get("apriltag", {})
    th = cfg.get("test", {})
    cap_cfg = cfg.get("capture", {})
    L = []
    L += ["# Phase 1 Verification Report", "",
          "MRV / Canadarm3 AprilTag vision capture of a linearly drifting 3 t MEP (no rotation).",
          "All values below are measured in the simulation runs listed at the end; nothing is filled in by hand.", ""]
    L += ["## Environment", "", "| Item | Value |", "|---|---|"] + [f"| {k} | {v} |" for k, v in env.items()] + [""]
    s_setup = static.get("metrics", {}).get("setup", {})
    d_setup = dynamic.get("metrics", {}).get("setup", {})
    L += ["## MEP Configuration", "", "| Item | Static run | Dynamic run |", "|---|---|---|",
          f"| Mass (PhysX) | {_f(s_setup.get('mep_mass_kg'), '{:.1f}')} kg | {_f(d_setup.get('mep_mass_kg'), '{:.1f}')} kg |",
          f"| Gravity | {s_setup.get('gravity')} | {d_setup.get('gravity')} |",
          f"| Initial linear velocity | {_vec(s_setup.get('mep_initial_linear_velocity_mps'))} m/s | {_vec(d_setup.get('mep_initial_linear_velocity_mps'))} m/s |",
          f"| Initial angular velocity | {_vec(s_setup.get('mep_initial_angular_velocity_deg_s'))} deg/s | {_vec(d_setup.get('mep_initial_angular_velocity_deg_s'))} deg/s |",
          f"| Drift direction (config) | — | {_vec(mep.get('drift_direction'))} |",
          f"| Ground-truth docking point | `debris/gripper_fixture/Cylinder_01` (r {_f(setup.get('cylinder01_radius_m', math.nan) * 1000, '{:.1f}')} mm, h {_f(setup.get('cylinder01_height_m', math.nan) * 1000, '{:.1f}')} mm) | same |", ""]
    k = setup.get("K") or [[math.nan] * 3] * 3
    L += ["## Camera Configuration", "", "| Item | Value |", "|---|---|",
          f"| Prim | `{setup.get('usd_camera', {}).get('prim', 'n/a')}` |",
          f"| Model | RGB pinhole, {cam.get('width')}x{cam.get('height')}, HFOV {cam.get('horizontal_fov_deg')} deg |",
          f"| Rendering | anti-aliasing {cam.get('antialiasing')}, supersample {cam.get('supersample')}x (area-downsampled to {cam.get('width')}x{cam.get('height')}) |",
          f"| USD focal length / apertures | {_f(setup.get('usd_camera', {}).get('focal_length_mm'), '{:.4f}')} mm / {_f(setup.get('usd_camera', {}).get('horizontal_aperture_mm'), '{:.3f}')} x {_f(setup.get('usd_camera', {}).get('vertical_aperture_mm'), '{:.3f}')} mm |",
          f"| K (used) | fx {_f(k[0][0])}, fy {_f(k[1][1])}, cx {_f(k[0][2])}, cy {_f(k[1][2])} px |",
          f"| Mount (link `canadarm3_large_7`) | pos {_vec(setup.get('t_link_cam', {}).get('pos'))} m, quat(wxyz) {_vec(setup.get('t_link_cam', {}).get('quat_wxyz'))} (optical +Z = EE approach axis) |",
          ""]
    L += ["## AprilTag Configuration", "", "| Item | Value |", "|---|---|",
          f"| Family / IDs | {tag.get('family')} / {tag.get('ids')} |",
          f"| Tag size (black square) | {tag.get('tag_size_m')} m |",
          f"| Constellation half width d | {tag.get('constellation_half_width_m')} m (Tag0..3 at (-d,+d), (+d,+d), (+d,-d), (-d,-d)) |",
          f"| Layout check | {setup.get('tag_layout', 'n/a')} |",
          f"| Constellation -> Cylinder_01 (fixed) | pos {_vec(setup.get('t_tags_cylinder01', {}).get('pos'), 5)} m, quat {_vec(setup.get('t_tags_cylinder01', {}).get('quat_wxyz'), 5)} |",
          f"| Pose | 16-point PnP (IPPE -> RANSAC -> LM), min tags {tag.get('min_tags_for_pose')} |", ""]

    st = static.get("metrics", {}).get("static", [])
    c1 = static.get("checks", {}).get("[TEST1] Cylinder_01 center estimation", {})
    pe, ae = c1.get("position_error_mm", math.nan), c1.get("angle_error_deg", math.nan)
    pth, ath = th.get("static_position_threshold_mm", 5.0), th.get("static_angle_threshold_deg", 0.5)
    L += ["## Pose Estimation Accuracy", "", "Static MEP; worst value over all images at all standoffs.", "",
          "| Metric | Result | Threshold | PASS/FAIL |", "|---|---:|---:|---|",
          f"| Position Error | {_f(pe)} mm | < {pth} mm | {_pf(pe < pth if pe == pe else False)} |",
          f"| Angle Error | {_f(ae, '{:.4f}')} deg | < {ath} deg | {_pf(ae < ath if ae == ae else False)} |", ""]
    if st:
        L += ["| Standoff | Camera - Cylinder_01 | Images | Position mean / max [mm] | Angle mean / max [deg] | Reproj. RMS [px] |", "|---:|---:|---:|---:|---:|---:|"]
        for s in st:
            L.append(f"| {s['standoff_m']:.2f} m | {s['camera_to_cylinder01_m']:.3f} m | {s['samples']} | {s['position_error_mm_mean']:.3f} / {s['position_error_mm_max']:.3f} | "
                     f"{s['angle_error_deg_mean']:.4f} / {s['angle_error_deg_max']:.4f} | {s['reproj_rms_px_mean']:.3f} |")
        L.append("")

    dm = dynamic.get("metrics", {})
    cap = dm.get("capture", {}) or {}
    t_cap = cap.get("time_s", math.nan)
    rel = (cap.get("gt") or {}).get("rel_vel", math.nan)
    rel_est = (cap.get("vision") or {}).get("rel_vel", math.nan)
    to = th.get("dynamic_timeout_sec", 60.0)
    vmax = cap_cfg.get("max_relative_velocity_mps", 0.05)
    L += ["## Dynamic Intercept", "", f"MEP drifting at {mep.get('linear_velocity_mps')} m/s along {_vec(mep.get('drift_direction'))}, no rotation.", "",
          "| Metric | Result | Threshold | PASS/FAIL |", "|---|---:|---:|---|",
          f"| Capture Time | {_f(t_cap, '{:.2f}')} s | < {to:.0f} s | {_pf(t_cap < to if t_cap == t_cap else False)} |",
          f"| Relative Velocity | {_f(rel, '{:.4f}')} m/s (vision estimate {_f(rel_est, '{:.4f}')}) | < {vmax} m/s | {_pf(rel < vmax if rel == rel else False)} |"]
    if cap:
        g, v = cap["gt"], cap["vision"]
        L += [f"| Distance EE - Cylinder_01 at capture | {g['distance']:.4f} m (vision {v['distance']:.4f}) | <= {cap_cfg.get('max_distance_m')} m | {_pf(g['distance'] <= cap_cfg.get('max_distance_m', 0))} |",
              f"| Angle EE axis - face normal | {g['angle']:.3f} deg (vision {v['angle']:.3f}) | <= {cap_cfg.get('max_angle_deg')} deg | {_pf(g['angle'] <= cap_cfg.get('max_angle_deg', 0))} |",
              f"| Lateral offset from the Cylinder_01 axis | {g['lateral']*1000:.2f} mm (vision {v['lateral']*1000:.2f}) | <= {cap_cfg.get('max_lateral_m', 0)*1000:.0f} mm | {_pf(g['lateral'] <= cap_cfg.get('max_lateral_m', 0))} |",
              f"| EE face - MEP face gap | {g['gap']*1000:.1f} mm (vision {v['gap']*1000:.1f}) | > 0 | {_pf(g['gap'] > 0)} |"]
    pe_ = dm.get("prediction_error", {})
    tr = dm.get("tracking_error", {})
    L += [f"| {cfg.get('prediction', {}).get('horizon_sec')} s prediction vs GT | mean {_f(pe_.get('mean_mm'), '{:.2f}')} / max {_f(pe_.get('max_mm'), '{:.2f}')} mm | (reported) | — |",
          f"| Vision pose error while tracking | mean {_f(tr.get('position_error_mm_mean'), '{:.2f}')} / max {_f(tr.get('position_error_mm_max'), '{:.2f}')} mm, mean {_f(tr.get('angle_error_deg_mean'), '{:.3f}')} / max {_f(tr.get('angle_error_deg_max'), '{:.3f}')} deg | (reported) | — |", ""]

    h = dm.get("holding", {}) or {}
    rt = dm.get("retreat", {}) or {}
    hd = h.get("duration_s", math.nan)
    stable = dynamic.get("checks", {}).get("[TEST3] Physics stability (MEP-EE relative pose)", {}).get("pass", False)
    L += ["## Holding Stability", "", "| Metric | Result | Threshold | PASS/FAIL |", "|---|---:|---:|---|",
          f"| Holding Duration | {_f(hd, '{:.2f}')} s | >= {th.get('holding_duration_sec', 10.0)} s | {_pf(hd >= th.get('holding_duration_sec', 10.0) - 1e-6 if hd == hd else False)} |",
          f"| Physics Stability | {'Stable' if stable else 'Unstable / not reached'} (max MEP-EE drift {_f(h.get('max_drift_mm'))} mm / {_f(h.get('max_drift_deg'), '{:.4f}')} deg, max MEP rate {_f(h.get('max_mep_rate_deg_s'), '{:.4f}')} deg/s) | Stable | {_pf(stable)} |",
          f"| FixedJoint valid | {h.get('joint_valid', 'n/a')} | True | {_pf(bool(h.get('joint_valid')))} |",
          f"| Robot EE stability | max EE excursion {_f(h.get('max_ee_motion_mm'), '{:.1f}')} mm; MEP speed {_f(cap.get('mep_speed_mps', math.nan) * 1000, '{:.2f}')} mm/s at capture -> max {_f(h.get('max_mep_speed_second_half', math.nan) * 1000, '{:.2f}')} mm/s in the 2nd half, {_f(h.get('final_mep_speed', math.nan) * 1000, '{:.2f}')} mm/s at the end | bounded (< 50 mm, decaying) | "
          f"{_pf(dynamic.get('checks', {}).get('[TEST3] Robot EE stability (motion bounded, not diverging)', {}).get('pass', False))} |"]
    if rt:
        L.append(f"| Slow retreat with MEP attached | EE {_f(rt.get('ee_moved_m', math.nan) * 1000, '{:.1f}')} mm, MEP {_f(rt.get('mep_moved_m', math.nan) * 1000, '{:.1f}')} mm, max drift {_f(rt.get('max_drift_mm'))} mm | MEP follows EE | "
                 f"{_pf(dynamic.get('checks', {}).get('[TEST3] Retreat with the MEP attached', {}).get('pass', False))} |")
    L.append("")

    L += ["## Drift Velocity Range", "", "The dynamic test (TEST 2 + TEST 3) is run at both ends of the required 0.01 - 0.02 m/s range; both must pass.", "",
          "| Run | MEP speed | Capture time | Rel. velocity (GT / vision) | Lateral at capture | Pred. error mean / max | Max MEP-EE drift (hold) | Max EE excursion (hold) | Checks |",
          "|---|---:|---:|---:|---:|---:|---:|---:|---|"]
    for d in dynamics:
        m = d.get("metrics", {})
        c = m.get("capture") or {}
        g, v = c.get("gt") or {}, c.get("vision") or {}
        hh = m.get("holding") or {}
        pe2 = m.get("prediction_error") or {}
        v0 = (m.get("setup") or {}).get("mep_initial_linear_velocity_mps")
        n_ok = sum(1 for x in d.get("checks", {}).values() if x["pass"])
        L.append(f"| {d.get('run', d.get('scenario'))} | {_f(math.sqrt(sum(x * x for x in v0)) if v0 else math.nan, '{:.4f}')} m/s | {_f(c.get('time_s', math.nan), '{:.2f}')} s | "
                 f"{_f(g.get('rel_vel', math.nan), '{:.4f}')} / {_f(v.get('rel_vel', math.nan), '{:.4f}')} m/s | {_f(g.get('lateral', math.nan) * 1000, '{:.2f}')} mm | "
                 f"{_f(pe2.get('mean_mm'), '{:.2f}')} / {_f(pe2.get('max_mm'), '{:.2f}')} mm | {_f(hh.get('max_drift_mm'))} mm | {_f(hh.get('max_ee_motion_mm'), '{:.1f}')} mm | "
                 f"{n_ok}/{len(d.get('checks', {}))} |")
    L.append("")

    L += ["## Final Result", ""]
    for name, v in verdicts.items():
        L.append(f"{name}: {'PASS' if v['pass'] else 'FAIL'}")
    overall = all(v["pass"] for v in verdicts.values())
    L += ["", f"Overall: {'PASS' if overall else 'FAIL'}", ""]
    for name, v in verdicts.items():
        if not v["pass"]:
            L += [f"{name}: FAIL", "Reason:"] + [f"- {r}" for r in v["reasons"]] + [""]

    L += ["## Checks", "", "| Scenario | Check | Result | Detail |", "|---|---|---|---|"]
    for res in [static] + dynamics:
        for k_, v in res.get("checks", {}).items():
            L.append(f"| {res.get('run', res.get('scenario'))} | {k_} | {_pf(v['pass'])} | {v.get('detail', '').replace('|', '/')} |")
    L += ["", "## Artifacts", "",
          f"- Telemetry CSV (both scenarios): `{csv_path.relative_to(PROJECT)}`",
          "- Per-run results: " + ", ".join(f"`logs/vision_capture/{r[0]}_result.json`" for r in RUNS),
          f"- Camera overlays: `{Path(static.get('metrics', {}).get('overlay_dir', 'logs/vision_capture/static_overlay')).name}/`, `{Path(dm.get('overlay_dir', 'logs/vision_capture/dynamic_overlay')).name}/` and `*_overlay.mp4` in `logs/vision_capture/`",
          f"- Console logs: `logs/vision_capture/*_console.log`", "",
          "| Scenario | Result file time | Sim time | Wall time | Exit code |", "|---|---|---:|---:|---:|"]
    for res in [static] + dynamics:
        m = res.get("metrics", {})
        L.append(f"| {res.get('run', res.get('scenario'))} | {res.get('mtime', 'n/a')} | {_f(m.get('sim_time_s'), '{:.1f}')} s | {_f(m.get('wall_time_s'), '{:.0f}')} s | {res.get('returncode', 'n/a')} |")
    path = LOG_DIR / "phase1_result.md"
    path.write_text("\n".join(L) + "\n")
    return path


def _pf(ok) -> str:
    return "PASS" if ok else "FAIL"


def _vec(v, nd=4) -> str:
    if v is None:
        return "n/a"
    return "(" + ", ".join(f"{x:.{nd}f}" for x in v) + ")"


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--report", action="store_true", help="Only rebuild the report from existing results")
    p.add_argument("--only", choices=tuple(r[0] for r in RUNS), default=None, help="Run only this one, reuse the others")
    p.add_argument("--set", dest="sets", action="append", default=[], help="Passed to both scenarios")
    a = p.parse_args()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    extra = [x for s in a.sets for x in ("--set", s)]
    results = {}
    for name, scenario, args in RUNS:
        if a.report or a.only not in (None, name):
            results[name] = load_result(name)
        else:
            results[name] = run_scenario(name, scenario, args + extra)
    static = results["static"]
    dynamics = [results[n] for n, sc, _ in RUNS if sc == "dynamic"]
    csv_path = merge_csv()
    verdicts = test_verdicts(static, dynamics)
    report = write_report(static, dynamics, verdicts, csv_path)
    for name, v in verdicts.items():
        print(f"{name}: {'PASS' if v['pass'] else 'FAIL'}")
        if not v["pass"]:
            print("Reason:")
            for r in v["reasons"]:
                print(f"- {r}")
    print(f"Report: {report}")
    print(f"CSV:    {csv_path}")
    sys.exit(0 if all(v["pass"] for v in verdicts.values()) else 1)


if __name__ == "__main__":
    main()
